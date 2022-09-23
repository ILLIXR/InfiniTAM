
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
        std::string calib_source = "/home/henrydc/tinker/InfiniTAM-ILLIXR/InfiniTAM/InfiniTAM/scripts/calibration/ETH3D.txt"; 
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
        inputIMUMeasurement = new ITMIMUMeasurement();

        std::cout << "Main engine initialized" << std::endl;
        output_mesh_name = "mesh.obj";

        count = 1;
        // sb->schedule<rgb_depth_type>(id, "rgb_depth", [&](switchboard::ptr<const rgb_depth_type> datum, std::size_t iteration_no) {
		// 	this->ProcessFrame(datum, iteration_no);
		// });
        sb->schedule<imu_cam_type>(id, "imu_cam", [&](switchboard::ptr<const imu_cam_type> datum, std::size_t iteration_no) {
			this->ProcessFrame(datum, iteration_no);
		});
    }

    
    // void ProcessFrame(switchboard::ptr<const rgb_depth_type> datum, std::size_t iteration_no){
    void ProcessFrame(switchboard::ptr<const imu_cam_type> datum, std::size_t iteration_no){
        if (datum == NULL) {
            return;
        }


        if (!datum->img1.has_value() || !datum->depth.has_value()) {
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

            // grab rgb and depth images 
            cv::Mat cur_depth{datum->depth.value()};
            cv::Mat cur_rgb{datum->img1.value()};

            // convert rgb and depth into InfiniTAM expected data type
            const short* depth_frame = reinterpret_cast<const short*>(cur_depth.datastart);
            short *cur_depth_head = inputRawDepthImage->GetData(MEMORYDEVICE_CPU);

            const Vector4u* color_frame = reinterpret_cast<const Vector4u*>(cur_rgb.datastart);
            Vector4u *cur_rgb_head = inputRGBImage->GetData(MEMORYDEVICE_CPU);

            std::memcpy(cur_rgb_head, color_frame, sizeof(Vector4u) * inputRGBImage->dataSize);
            std::memcpy(cur_depth_head, depth_frame, sizeof(short) * inputRawDepthImage->dataSize);

            switchboard::ptr<const pose_type> slow_pose_ptr = _m_slow_pose.get_ro_nullable();
            if (!slow_pose_ptr) {
                std::cout << "InfiniTAM: pose not available" << std::endl;
                return;
            }
            // pose_type swapped_pose = pp->correct_pose(*slow_pose_ptr);
            // std::cout << "ORB_SLAM Pose: " << double(slow_pose_ptr->position.x()) << " "
            //                             << double(slow_pose_ptr->position.x()) << " "
            //                             << double(slow_pose_ptr->position.x()) << " "
            //                             << double(slow_pose_ptr->orientation.x()) << " "
            //                             << double(slow_pose_ptr->orientation.x()) << " "
            //                             << double(slow_pose_ptr->orientation.x()) << " "
            //                             << double(slow_pose_ptr->orientation.x()) << std::endl;
            // if (datum->dataset_time < 2995*1e9) {
            //     std::cout << "Skipping poses" << datum->dataset_time << " " << duration2double(datum->time.time_since_epoch()) << std::endl;
            //     return;
            // }
            
            std::vector<double> cur_pose = {duration2double(datum->time.time_since_epoch()), 
                                            double(slow_pose_ptr->position.x()),
                                            double(slow_pose_ptr->position.y()),
                                            double(slow_pose_ptr->position.z()),
                                            double(slow_pose_ptr->orientation.x()),
                                            double(slow_pose_ptr->orientation.y()),
                                            double(slow_pose_ptr->orientation.z()),
                                            double(slow_pose_ptr->orientation.w())};
            mainEngine->input_pose = cur_pose;

            std::cout << "rgbd_timestamp " << duration2double(datum->time.time_since_epoch()) << std::endl;
            // ORUtils::Matrix4<float> input_pose;
            // Eigen::Vector3f cur_pos = slow_pose_ptr->position;
            // Eigen::Quaternionf ori = slow_pose_ptr->orientation;
            // Eigen::Matrix3f cur_ori = ori.normalized().toRotationMatrix();

            // input_pose.m[0] = cur_ori(0,0); input_pose.m[4] = cur_ori(0,1); input_pose.m[8] = cur_ori(0,2);  input_pose.m[12] = cur_pos(0);
            // input_pose.m[1] = cur_ori(1,0); input_pose.m[5] = cur_ori(1,1); input_pose.m[9] = cur_ori(1,2);  input_pose.m[13] = cur_pos(1);
            // input_pose.m[2] = cur_ori(2,0); input_pose.m[6] = cur_ori(2,1); input_pose.m[10] = cur_ori(2,2); input_pose.m[14] = cur_ori(2);
            // input_pose.m[3] = 0.0f;         input_pose.m[7] = 0.0f;         input_pose.m[11] = 0.0f;         input_pose.m[15] = 1.0f;

            // mainEngine->input_mat = input_pose;
            //actual processing on the mainEngine
            std::cout << "time " << std::to_string(duration2double(datum->time.time_since_epoch())) << std::endl;
            mainEngine->currentTimeStamp = std::to_string(duration2double(datum->time.time_since_epoch()));

            mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);
            
            if(count == 1017)
            {
                std::cout<<"producing mesh: "<<output_mesh_name<<std::endl;
                mainEngine->SaveSceneToMesh(output_mesh_name.c_str());      
                std::exit(0);
            }

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
        }
        freqControl.framesSinceFreqChange++;
        std::cout << "\n";
        count++;
    }
    
    virtual ~infiniTAM() override{
        std::cout<<"producing mesh: "<<output_mesh_name<<std::endl;
        mainEngine->SaveSceneToMesh(output_mesh_name.c_str());
    }
private:
    const std::shared_ptr<switchboard> sb;
    const std::shared_ptr<pose_prediction> pp;
    switchboard::reader<pose_type>    _m_slow_pose;

    ITMLib::ITMRGBDCalib *calib;
    ITMLib::ITMMainEngine *mainEngine;
    ITMUChar4Image *inputRGBImage;
    ITMShortImage *inputRawDepthImage;
    ITMLib::ITMIMUMeasurement *inputIMUMeasurement;

    std::string output_mesh_name;

    struct FrequencyControl {
        unsigned freqDivisor;
        int framesSinceFreqChange;
        std::vector<unsigned> *processed;
        std::vector<double> *frequencies;
        std::vector<unsigned> *newBricks;
    };

    FrequencyControl freqControl;
    unsigned raycastingFreqDivisor;
    StopWatchInterface *timer_instant;
    StopWatchInterface *timer_average;
    ITMLib::ITMLibSettings *internalSettings;

    int count;
};

PLUGIN_MAIN(infiniTAM);
