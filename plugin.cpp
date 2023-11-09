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
using namespace ILLIXR;
class infinitam : public plugin {
public:
	infinitam(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
    {
        //pyh still hardcode to read the calib file
        char cur_path[256];
        getcwd(cur_path,256);
        std::string calib_source = std::string(cur_path) + "/InfiniTAM/calib.txt";

        //read from calib.txt and convert to ITMRGBDCalib format
        //modified from ITMLib/Objects/Camera/ITMCalibIo::readRGBDCalib(
        calib = new ITMLib::ITMRGBDCalib();
        if(!readRGBDCalib(calib_source.c_str(), *calib))
        {
            printf("Read RGBD caliberation file failed\n");
        }

        ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();

        mainEngine = new ITMLib::ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(
                        internalSettings,
                        *calib,
                        calib->intrinsics_rgb.imgSize,
                        calib->intrinsics_d.imgSize
                        );

        //dims, allocate_cpu?, allocate_gpu?
        inputRawDepthImage = new ITMShortImage(calib->intrinsics_d.imgSize, true, false);
        inputRGBImage = new ITMUChar4Image(calib->intrinsics_rgb.imgSize, true, false);


        sb->schedule<rgb_depth_type>(id, "rgb_depth", [&](switchboard::ptr<const rgb_depth_type> datum, std::size_t){ 
            this->ProcessFrame(datum);
        });
		printf("================================InfiniTAM: setup finished==========================\n");
	}

    void ProcessFrame(switchboard::ptr<const rgb_depth_type> datum)
	{
		printf("================================InfiniTAM: Info received==========================\n");
		if(!datum->depth.empty() && !datum->rgb.empty())
		{
		    cv::Mat cur_depth = datum->depth;
		    cv::Mat cur_rgb = datum->rgb;
            
		    std::string depth_type = type2str(cur_depth.type());
            
		    std::string rgb_type = type2str(cur_rgb.type());
            
		    printf("depth type %s\n", depth_type.c_str());
		    printf("rgb type%s\n", rgb_type.c_str());


		    const Vector4u *color_frame = reinterpret_cast<const Vector4u*>(cur_rgb.datastart);
		    const uint16_t *depth_frame = reinterpret_cast<const uint16_t*>(cur_depth.datastart);

		    short *cur_depth_head = inputRawDepthImage->GetData(MEMORYDEVICE_CPU);
            Vector4u *cur_rgb_head = inputRGBImage->GetData(MEMORYDEVICE_CPU);


		    std::memcpy(cur_rgb_head, color_frame, sizeof(Vector4u) *inputRGBImage->dataSize);
		    std::memcpy(cur_depth_head, depth_frame, sizeof(short)  * inputRawDepthImage->dataSize);
		    mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);
#ifndef COMPILE_WITHOUT_CUDA
            ORcudaSafeCall(cudaThreadSynchronize());
#endif
		}
		else
		{
		    printf("missing either rgb or depth or both\n");
		}
	}

    virtual ~infinitam() override{
        mainEngine->SaveSceneToMesh("infinitam.obj");
    }
    std::string type2str(int type) 
    {
        std::string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch ( depth ) 
        {
            case CV_8U:  r = "8U"; break;
            case CV_8S:  r = "8S"; break;
            case CV_16U: r = "16U"; break;
            case CV_16S: r = "16S"; break;
            case CV_32S: r = "32S"; break;
            case CV_32F: r = "32F"; break;
            case CV_64F: r = "64F"; break;
            default:     r = "User"; break;
        }
        r += "C";
        r += (chans+'0');

         return r;
    }

    
private:
	const std::shared_ptr<switchboard> sb;

	ITMUChar4Image *inputRGBImage;
    ITMShortImage *inputRawDepthImage;
    ITMLib::ITMMainEngine *mainEngine;
    ITMLib::ITMRGBDCalib *calib;

};

PLUGIN_MAIN(infinitam)
