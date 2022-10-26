
#include "../common/plugin.hpp"
#include "../common/phonebook.hpp"
#include "../common/switchboard.hpp"
#include "../common/data_format.hpp"
#include "../common/relative_clock.hpp"
#include "../common/pose_prediction.hpp"

#include <iomanip>
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../InputSource/ImageSourceEngine.h"
#include "../InputSource/IMUSourceEngine.h"
#include "../ITMLib/Core/ITMBasicEngine.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ORUtils/FileUtils.h"
#include "../ORUtils/NVTimer.h"

#include "../InputSource/OpenNIEngine.h"
#include "../InputSource/Kinect2Engine.h"

#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Core/ITMBasicEngine.h"

using namespace ILLIXR;
using namespace ITMLib;

class infiniTAM : public plugin {
public:
    infiniTAM(std::string name_, phonebook* pb_)
        : plugin{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , pp{pb->lookup_impl<pose_prediction>()}
        , _m_slow_pose{sb->get_reader<pose_type>("slow_pose")}
        , _m_reconstruction{sb->get_writer<reconstruction_type>("scene_reconstruction")}
    {
        // internal setting 
        internalSettings = new ITMLib::ITMLibSettings();
        internalSettings->useICP = false; // comment if using poses externally 
        internalSettings->useApproximateDepthCheck = false;
        internalSettings->usePreviousVisibilityList = true;
        internalSettings->freqMode = ITMLib::ITMLibSettings::FreqMode::FREQMODE_CONSTANT;
        internalSettings->fusionFreq = 30.0;
        internalSettings->useDecoupledRaycasting = false;
        internalSettings->raycastingFreq=1.0;

        // calibration
        std::string calib_source = "/home/illixr/henry/InfiniTAM/InfiniTAM/scripts/calibration/ETH3D.txt"; 
        calib = new ITMLib::ITMRGBDCalib();
        if(!readRGBDCalib(calib_source.c_str(), *calib)){
            printf("Read RGBD caliberation file failed\n");
        }

        std::cout << "Main engine begins initializing" << std::endl;
        // create main engine
        mainEngine = new ITMLib::ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(
            internalSettings,
            *calib,
            calib->intrinsics_rgb.imgSize,
            calib->intrinsics_d.imgSize
        );
        bool allocateGPU = false;
	    if (internalSettings->deviceType == ITMLibSettings::DEVICE_CUDA) allocateGPU = true;

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaThreadSynchronize());
#endif

        sdkCreateTimer(&timer_instant);
        sdkCreateTimer(&timer_average);
        sdkResetTimer(&timer_average);

        freqControl.freqDivisor = mainEngine->GetFreqDivisor();
        freqControl.framesSinceFreqChange = 0;
        freqControl.processed = new std::vector<unsigned>;
        freqControl.frequencies = new std::vector<double>;
        freqControl.newBricks = new std::vector<unsigned>;

        raycastingFreqDivisor = ITMLibSettings::MAX_FREQ / static_cast<unsigned>(internalSettings->raycastingFreq);
        printf("initialised with ILLIXR.\n");

        inputRawDepthImage = new ITMShortImage(calib->intrinsics_d.imgSize, true, allocateGPU);
        inputRGBImage = new ITMUChar4Image(calib->intrinsics_rgb.imgSize, true, allocateGPU);
        reconstructedImage = new ITMUChar4Image(calib->intrinsics_d.imgSize, true, false);
        inputIMUMeasurement = new ITMIMUMeasurement();

        std::cout << "Main engine initialized" << std::endl;
        output_mesh_name = "mesh.obj";

        sb->schedule<imu_cam_type>(id, "imu_cam", [&](switchboard::ptr<const imu_cam_type> datum, std::size_t iteration_no) {
			this->ProcessFrame(datum, iteration_no);
		});
    }

    
    // void ProcessFrame(switchboard::ptr<const rgb_depth_type> datum, std::size_t iteration_no){
    void ProcessFrame(switchboard::ptr<const imu_cam_type> datum, std::size_t iteration_no){
        if (datum == NULL && buffer.empty()) {
            return;
        }
        if ((!datum->img1.has_value() || !datum->depth.has_value()) && buffer.empty()) {
            return;
        }
        freqControl.frequencies->push_back(ITMLibSettings::MAX_FREQ / static_cast<double>(freqControl.freqDivisor));
        bool shouldSkip = (freqControl.framesSinceFreqChange % freqControl.freqDivisor) != 0;
        if (shouldSkip)
        {
            std::cout << "============================= Skip frame "<< count<<" ==============================" << std::endl;
        }
        else
        {
            std::cout << "============================= Run frame "<< count<<" ==============================" << std::endl;
            sdkResetTimer(&timer_instant);
            sdkStartTimer(&timer_instant); 
            sdkStartTimer(&timer_average);

            // get slow poses from ILLIXR
            switchboard::ptr<const pose_type> slow_pose_ptr = _m_slow_pose.get_ro_nullable();
            if (slow_pose_ptr->position == Eigen::Vector3f::Zero()) {
                std::cout << "Invalid pose: " << iteration_no << std::endl;
                prev_datum = datum;
                return;
            }
            if (prev_datum != NULL) {
                buffer.push(prev_datum);
                prev_datum = datum;
            }

            // check if new pose is received
            auto start = std::chrono::steady_clock::now();
            while (prev_timestamp == slow_pose_ptr->sensor_time) {
                slow_pose_ptr = _m_slow_pose.get_ro_nullable();
            }
            auto end = std::chrono::steady_clock::now();
            double duration = std::chrono::duration<double>(end-start).count();
            total_time+= duration;
            min_runtime = std::min(min_runtime, duration);
            max_runtime = std::max(max_runtime, duration);
            count++;
            std::cout <<  "While loop waiting time: current: " << duration << " min: " << min_runtime << " max: " << max_runtime << " avg: " << total_time / count << std::endl;

            // match pose with corresponding cam images
            switchboard::ptr<const imu_cam_type> current_datum = buffer.front();
            std::cout << "Using cam " << duration2double(current_datum->time.time_since_epoch()) << " for pose " << duration2double(slow_pose_ptr->sensor_time.time_since_epoch()) << std::endl;
            
            // grab rgb and depth images 
            cv::Mat cur_depth{current_datum->depth.value()};        
            cv::Mat cur_rgb{current_datum->img1.value()};
            
            // convert rgb and depth into InfiniTAM expected data type
            const short* depth_frame = reinterpret_cast<const short*>(cur_depth.datastart);
            short *cur_depth_head = inputRawDepthImage->GetData(MEMORYDEVICE_CPU);
            const Vector4u* color_frame = reinterpret_cast<const Vector4u*>(cur_rgb.datastart);
            Vector4u *cur_rgb_head = inputRGBImage->GetData(MEMORYDEVICE_CPU);
            std::memcpy(cur_rgb_head, color_frame, sizeof(Vector4u) * inputRGBImage->dataSize);
            std::memcpy(cur_depth_head, depth_frame, sizeof(short) * inputRawDepthImage->dataSize);
            
            // feed poses to main engine 
            std::vector<double> cur_pose = {duration2double(current_datum->time.time_since_epoch()), 
                                            double(slow_pose_ptr->position.x()),
                                            double(slow_pose_ptr->position.y()),
                                            double(slow_pose_ptr->position.z()),
                                            double(slow_pose_ptr->orientation.x()),
                                            double(slow_pose_ptr->orientation.y()),
                                            double(slow_pose_ptr->orientation.z()),
                                            double(slow_pose_ptr->orientation.w())};
            mainEngine->input_pose = cur_pose;

            std::cout << "datum_time: " << duration2double(datum->time.time_since_epoch()) << std::endl;
            std::cout << "current_datum_time: " << std::to_string(duration2double(current_datum->time.time_since_epoch())) << std::endl;
            
            //actual processing on the mainEngine
            mainEngine->currentTimeStamp = std::to_string(duration2double(current_datum->time.time_since_epoch()));
            mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);
            prev_timestamp = slow_pose_ptr->sensor_time;
            buffer.pop();
            std::cout << "============================= Finished frame "<< count<<" ==============================" << std::endl;

#ifndef COMPILE_WITHOUT_CUDA
        ORcudaSafeCall(cudaThreadSynchronize());
#endif
            sdkStopTimer(&timer_instant); sdkStopTimer(&timer_average);

            float processedTime_inst = sdkGetTimerValue(&timer_instant);
            float processedTime_avg = sdkGetAverageTimerValue(&timer_average);

            printf("frame %i: time %.2f, avg %.2f\n", count, processedTime_inst, processedTime_avg);

            freqControl.processed->push_back(1);
            freqControl.newBricks->push_back(mainEngine->GetNumNewBricks());
            unsigned newDivisor = mainEngine->GetFreqDivisor();
            if (newDivisor != freqControl.freqDivisor)
            {
                freqControl.freqDivisor = newDivisor;
                freqControl.framesSinceFreqChange = 0;
            }

            // Publish reconstructed image
            mainEngine->GetImage(reconstructedImage, reconstructedImageType);
            _m_reconstruction.put(_m_reconstruction.allocate(
               cv::Mat{reconstructedImage->noDims.y,
                       reconstructedImage->noDims.x,
                       CV_8UC4,
                       reconstructedImage->GetData(MEMORYDEVICE_CPU)
               }
            ));
        }

        freqControl.framesSinceFreqChange++;
        std::cout << "\n";
        
    }
    
    virtual ~infiniTAM() override {
        std::cout<<"producing mesh: "<<output_mesh_name<<std::endl;
        mainEngine->SaveSceneToMesh(output_mesh_name.c_str());

        // Free pointers
        delete internalSettings;
        delete calib;
        delete mainEngine;

        // Free images
        delete inputRawDepthImage;
        delete inputRGBImage;
        delete reconstructedImage;
        delete inputIMUMeasurement;

        // Free PID data
        delete freqControl.processed;
        delete freqControl.frequencies;
        delete freqControl.newBricks;
    }

private:
    const std::shared_ptr<switchboard> sb;
    const std::shared_ptr<pose_prediction> pp;
    switchboard::reader<pose_type> _m_slow_pose;
    switchboard::writer<reconstruction_type> _m_reconstruction;

    ITMLib::ITMRGBDCalib *calib;
    ITMLib::ITMMainEngine *mainEngine;
    ITMUChar4Image *inputRGBImage;
    ITMShortImage *inputRawDepthImage;
    ITMLib::ITMIMUMeasurement *inputIMUMeasurement;

    // Reconstruction data
    ITMUChar4Image *reconstructedImage;
    ITMLib::ITMMainEngine::GetImageType reconstructedImageType{ITMLib::ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME};

    std::string output_mesh_name;

    struct FrequencyControl {
        unsigned freqDivisor;
        int framesSinceFreqChange;
        std::vector<unsigned> *processed;
        std::vector<double> *frequencies;
        std::vector<unsigned> *newBricks;
    };

    std::queue<switchboard::ptr<const imu_cam_type>> buffer;
    time_point prev_timestamp;
    switchboard::ptr<const imu_cam_type> prev_datum;

    FrequencyControl freqControl;
    unsigned raycastingFreqDivisor;
    StopWatchInterface *timer_instant;
    StopWatchInterface *timer_average;
    ITMLib::ITMLibSettings *internalSettings;

    int count = 1;
    double total_time = 0;
    double min_runtime = -9999999;
    double max_runtime = 99999999;
};

PLUGIN_MAIN(infiniTAM);
