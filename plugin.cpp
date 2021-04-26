
#include "common/threadloop.hpp"
#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"


//3/18 use RBGD dataset
#include "data_loading.hpp"

//add infinitam libraries
#include "Apps/InfiniTAM_cli/CLIEngine.h"
#include "Apps/InfiniTAM/UIEngine.h"
#include "ITMLib/ITMLibDefines.h"
#include "ITMLib/Core/ITMBasicEngine.h"

#include <signal.h>
using namespace ILLIXR;
//yihan: using plugin
class infinitam : public plugin {
public:
	infinitam(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, _m_rgb_depth{sb->subscribe_latest<rgb_depth_type>("rgb_depth")}
		, sb{pb->lookup_impl<switchboard>()}
    {
        InputSource::ImageSourceEngine *imageSource = NULL;
        InputSource::IMUSourceEngine *imuSource = NULL;
        std::string calib_source = "/home/yihan/ILLIXR/infinitam/real_calib.txt";
        ITMLib::ITMRGBDCalib *calib = new ITMLib::ITMRGBDCalib();
//        readRGBDCalib(calib_source.c_str(), calib);
        //manual porting calib
        
        calib->intrinsics_d.projectionParamsSimple.fx = 385.466644;
        calib->intrinsics_d.projectionParamsSimple.fy = 385.466644;
        calib->intrinsics_d.projectionParamsSimple.px = 315.551788;
        calib->intrinsics_d.projectionParamsSimple.py = 243.214706;

        calib->intrinsics_rgb.projectionParamsSimple.fx = 611.514404;
        calib->intrinsics_rgb.projectionParamsSimple.fy = 611.393982;
        calib->intrinsics_rgb.projectionParamsSimple.px = 318.288635;
        calib->intrinsics_rgb.projectionParamsSimple.py = 247.232956;
        
        Matrix4f extrinsics;
        extrinsics.m00 = 0.999934; //rotation[0]
        extrinsics.m01 = -0.011091; //rotation[3]
        extrinsics.m02 = -0.002991; //rotation[6]
        extrinsics.m03 = 0.0f;
        extrinsics.m10 = 0.011078; //rotation[1]
        extrinsics.m11 = 0.999928; //rotation[4]
        extrinsics.m12 = -0.004641; //rotation[7]
        extrinsics.m13 = 0.0f;
        extrinsics.m20 = 0.003042; //rotation[2]
        extrinsics.m21 = 0.004608; //rotation[5]
        extrinsics.m22 = 0.999985; //rotation[8]
        extrinsics.m23 = 0.0f;
        extrinsics.m30 = -0.014601; //translation[0]
        extrinsics.m31 = -0.000422; //translation[1]
        extrinsics.m32 = -0.000426; //translation[2]
        extrinsics.m33 = 1.0f;

        calib->trafo_rgb_to_depth.SetFrom(extrinsics);
        
        ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();
        mainEngine = new ITMLib::ITMBasicEngine<ITMVoxel,ITMVoxelIndex>(
            internalSettings, *calib, Vector2i(640,480), Vector2i(640,480)
        );

//        InfiniTAM::Engine::CLIEngine::Instance()->Initialise(imageSource, imuSource, mainEngine, internalSettings->deviceType);
        inputRGBImage = new ITMUChar4Image(Vector2i(640,480), true, true);
        inputRawDepthImage = new ITMShortImage(Vector2i(640,480), true, true);


//        inputRGBImage = new ITMUChar4Image(Vector2i(640,480), true, true); 
//        inputRawDepthImage = new ITMShortImage(Vector2i(640,480), true, true);;
        
		printf("================================InfiniTAM: setup finished==========================\n");
	}


    virtual void start() override{
        plugin::start();
        sb->schedule<rgb_depth_type>(id, "rgb_depth", [&](const rgb_depth_type *datum) { 
            this->ProcessFrame(datum);
        });
    }
    void ProcessFrame(const rgb_depth_type *datum)
	{
		printf("================================InfiniTAM: Info received==========================\n");
        cv::Mat rgb{*datum->rgb.value()};
        cv::Mat depth{*datum->depth.value()};
        std:: string old_ty =  type2str( depth.type() );
        printf("old rgb Matrix: %s,  %d x %d \n", old_ty.c_str(), rgb.cols, rgb.rows );

        cv::imwrite("cur_rgb.png", rgb);
        cv::imwrite("cur_depth.png", depth);
        system("convert /home/yihan/ILLIXR/cur_rgb.png /home/yihan/ILLIXR/cur_rgb.ppm");
        system("convert /home/yihan/ILLIXR/cur_depth.png -flatten /home/yihan/ILLIXR/cur_depth.pgm");
//        std::string cur_rgb_location = "/home/yihan/ILLIXR/demo_data/rgbd_dataset_freiburg1_rpy/rgb_sort/0000.ppm";
//        std::string test_depth_location = "/home/yihan/ILLIXR/cur_depth.png";
//        cv::Mat depth_info = cv::imread(test_depth_location.c_str(), cv::IMREAD_UNCHANGED);
//        std:: string ty =  type2str( depth_info.type() );
 //       printf("new rgb Matrix: %s,  %d x %d \n", ty.c_str(), depth_info.cols, depth_info.rows );
        std::string cur_rgb_location = "/home/yihan/ILLIXR/cur_rgb.ppm";
        std::string cur_depth_location = "/home/yihan/ILLIXR/cur_depth.pgm";
        
        if(!ReadImageFromFile(inputRGBImage, cur_rgb_location.c_str()))
        {
            printf("rgb read is not valid\n");
        }
        else
        {
            printf("rgb read is valid\n");
        }
        if(!ReadImageFromFile(inputRawDepthImage, cur_depth_location.c_str()))
        {
            printf("depth read is not valid\n");
        }
        else
        {
            printf("depth read is valid\n");
        }
        
//        ReadImageFromFile(inputRawDepthImage, "cur_depth.pgm");
//		printf("datum rgb size %d %d \n",rgb.rows, rgb.cols);
//		printf("datum depth size %d %d \n",depth.rows, depth.cols);
		//printf("datum depth size %d %d \n", datum->depth->value().rows, datum->depth->value().cols);
		if(inputRGBImage ==NULL || inputRawDepthImage ==NULL)
		{
		    printf("somethign is wrong\n");
		}
        mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);
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

	virtual ~infinitam() override {
        //need to implement UIEngine::Instance()->shutdown()
        mainEngine->SaveSceneToMesh("mesh.stl");
	}

    
private:
	const std::shared_ptr<switchboard> sb;
	std::unique_ptr<reader_latest<rgb_depth_type>> _m_rgb_depth;
	ITMUChar4Image *inputRGBImage;
    ITMShortImage *inputRawDepthImage;
    ITMLib::ITMMainEngine *mainEngine;
};

PLUGIN_MAIN(infinitam)
