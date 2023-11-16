#include "illixr/plugin.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/data_format.hpp"
#include "illixr/phonebook.hpp"
#include "illixr/opencv_data_types.hpp"

//add infinitam libraries
#include "Apps/InfiniTAM_cli/CLIEngine.h"
#include "Apps/InfiniTAM/UIEngine.h"
#include "ITMLib/ITMLibDefines.h"
#include "ITMLib/Core/ITMBasicEngine.h"
#include "ORUtils/FileUtils.h"
#include <signal.h>
#include <unistd.h>

//pyh: this is a plugin that uses the original ICP-version of InfiniTAM
//Current tested dataset include TUM-RGBD and ScanNet
using namespace ILLIXR;
class infinitam : public plugin {
public:
	infinitam(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
                , _m_scene_recon_data{sb->get_reader<scene_recon_type>("scene_recon_data")}
    {
        //pyh for ScanNet, the calibration already exists for each data sequence
        //However, for TUM-RGBD, one needs to create a custom one inside each sequence. We have provided one for freiburg1_desk
        const char* illixr_data_c_str = std::getenv("ILLIXR_DATA");
        std::string illixr_data = std::string{illixr_data_c_str};
        const std::string calib_subpath = "/calibration.txt";
        std::string calib_source{illixr_data + calib_subpath};
        printf("calibration file path: %s\n", calib_source.c_str());

        //pyh: modified from ITMLib/Objects/Camera/ITMCalibIo::readRGBDCalib(
        calib = new ITMLib::ITMRGBDCalib();
        if(!readRGBDCalib(calib_source.c_str(), *calib))
        {
            printf("Read RGBD caliberation file failed\n");
        }

        ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();
        //pyh for testing parameter purpose
        std::cout<<"rgb calibration: "<<calib->intrinsics_rgb.projectionParamsSimple.all<<"\n";
        std::cout<<"depth calibration: "<<calib->intrinsics_d.projectionParamsSimple.all<<"\n";

        mainEngine = new ITMLib::ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(
                        internalSettings,
                        *calib,
                        calib->intrinsics_rgb.imgSize,
                        calib->intrinsics_d.imgSize
                        );

        //pyh: the three parameters passed are dims, allocate_cpu?, allocate_gpu?
        inputRawDepthImage = new ITMShortImage(calib->intrinsics_d.imgSize, true, false);
        inputRGBImage = new ITMUChar4Image(calib->intrinsics_rgb.imgSize, true, false);

        sb->schedule<scene_recon_type>(id, "scene_recon_data", [&](switchboard::ptr<const scene_recon_type> datum, std::size_t){ 
            this->ProcessFrame(datum);
        });
	printf("================================InfiniTAM: setup finished==========================\n");
        //pyh mesh generation destination
        //When running ILLIXR, the mesh will be generated in the ILLIXR home directory (one level up)
       output_mesh_name="your_mesh_name.obj"; 
	}

    void ProcessFrame(const switchboard::ptr<const scene_recon_type> datum)
	{
            printf("================================InfiniTAM: frame %d received==========================\n", frame_count);
                cv::Mat cur_depth, cur_rgb;
		if(!datum->depth.empty()){
                    frame_count++;
		    cur_depth = datum->depth.clone();
                    //pyh converting to the InfiniTAM expected data structure 
                    const uint16_t *depth_frame = reinterpret_cast<const uint16_t*>(cur_depth.datastart);

                    short *cur_depth_head = inputRawDepthImage->GetData(MEMORYDEVICE_CPU);
                    std::memcpy(cur_depth_head, depth_frame, sizeof(short) * inputRawDepthImage->dataSize);
                } else {
                    std::cerr<<"no depth image\n";
                    std::exit(0);
                }
                if(!datum->rgb.value().empty()) {
                    //enable color
                    cv::Mat cur_rgb = datum->rgb.value().clone();
                    const Vector4u *color_frame = reinterpret_cast<const Vector4u*>(cur_rgb.datastart);
                    Vector4u *cur_rgb_head = inputRGBImage->GetData(MEMORYDEVICE_CPU);
                    std::memcpy(cur_rgb_head, color_frame, sizeof(Vector4u) *inputRGBImage->dataSize);
                }
                
                ITMLib::ITMTrackingState::TrackingResult tracking_status = mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);

                if (tracking_status == ITMLib::ITMTrackingState::TRACKING_FAILED)
                {
                    std::cerr<<"tracking failed at frame "<<frame_count<<std::endl;
                    std::cout<<"producing mesh: "<<output_mesh_name<<std::endl;
                    mainEngine->SaveSceneToMesh(output_mesh_name.c_str());
                    std::exit(0);
                }

                ORcudaSafeCall(cudaThreadSynchronize());
                if (datum->last_frame) {
                    std::cout<<"Data sequence finished,\n producing mesh: "<<output_mesh_name<<std::endl;
                    mainEngine->SaveSceneToMesh(output_mesh_name.c_str());
                }
        }

    virtual ~infinitam() override{
        std::cout<<"producing mesh: "<<output_mesh_name<<std::endl;
        mainEngine->SaveSceneToMesh(output_mesh_name.c_str());
    }
    
private:
    const std::shared_ptr<switchboard> sb;
    switchboard::reader<scene_recon_type> _m_scene_recon_data;
    ITMUChar4Image *inputRGBImage;
    ITMShortImage *inputRawDepthImage;
    ITMLib::ITMMainEngine *mainEngine;
    ITMLib::ITMRGBDCalib *calib;
    unsigned frame_count=0;
    std::ofstream adjusted_file;
    std::vector<std::vector<float>> gt_array;
    std::string output_mesh_name;
};

PLUGIN_MAIN(infinitam)
