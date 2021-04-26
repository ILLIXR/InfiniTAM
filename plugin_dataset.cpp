
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


using namespace ILLIXR;
//yihan: using threadloop
class infinitam : public threadloop {
public:
	infinitam(std::string name_, phonebook* pb_)
		: threadloop{name_, pb_}
		, _m_sensor_data{load_data()}
		, _m_sensor_data_it{_m_sensor_data.cbegin()}
		, dataset_first_time{_m_sensor_data.cbegin()->first}
		, sb{pb->lookup_impl<switchboard>()}
    {
        cur_timestamp = 0;
    
        
        //assume using TUM dataset with has imu data
        //using ILLIXR_data
        const char* illixr_data_c_str = std::getenv("ILLIXR_DATA");
        std::string illixr_data = std::string{illixr_data_c_str};
        //pyh bandage solution
        std::string rgb_source = illixr_data + "/rgb_sort/%04i.ppm";
        std::string depth_source = illixr_data + "/depth_sort/%04i.pgm";
        
        InputSource::ImageMaskPathGenerator pathGenerator(rgb_source.c_str(), depth_source.c_str());
        //test if the image is reading correctly
        cv::Mat depth_info = cv::imread(pathGenerator.getRgbImagePath(0).c_str(), cv::IMREAD_UNCHANGED);
        std:: string ty =  type2str( depth_info.type() );
        printf("rgb Matrix: %s %dx%d \n", ty.c_str(), depth_info.cols, depth_info.rows );
        depth_info = cv::imread(pathGenerator.getDepthImagePath(0).c_str(), cv::IMREAD_UNCHANGED);
        ty =  type2str( depth_info.type() );
        printf("depth Matrix: %s %dx%d \n", ty.c_str(), depth_info.cols, depth_info.rows );
//        printf("cols: %d, rows: %d, dims: %d\n", depth_info.cols,depth_info.rows, depth_info.dims);
        InputSource::ImageSourceEngine *imageSource = NULL;
        std::string calib_source = "/home/yihan/ILLIXR/infinitam/calib.txt";
        imageSource = new InputSource::ImageFileReader<InputSource::ImageMaskPathGenerator>(calib_source.c_str(), pathGenerator);

        InputSource::IMUSourceEngine *imuSource = NULL;

        ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();
        printf("rgb image size %d , %d\n",  imageSource->getRGBImageSize()[0], imageSource->getRGBImageSize()[1]);
        ITMLib::ITMMainEngine *mainEngine = new ITMLib::ITMBasicEngine<ITMVoxel,ITMVoxelIndex>(
            internalSettings, imageSource->getCalib(), imageSource->getRGBImageSize(), imageSource->getDepthImageSize()
        );
//        int fake_argc=0;
//        InfiniTAM::Engine::UIEngine::Instance()->Initialise(fake_argc, NULL, imageSource, imuSource, mainEngine, "./Files/Out", internalSettings->deviceType);
        InfiniTAM::Engine::CLIEngine::Instance()->Initialise(imageSource, imuSource, mainEngine, internalSettings->deviceType);

        infinitam_stop=false;
        /*four function need to implement
        CreateDefaultImageSource
        ITMLib::ITMLibSettings
        switch case
        UIEnginge::Instance()->Initialise

        */
        


/*
        //setup should mimick InfiniTAM.cpp
        InputSource::ImageMaskPathGenerator pathGenerator(rgb_source.c_str(), depth_source.c_str());
        //pyh not robust caliberation file
        InputSource::ImageSourceEngine *imageSource = new InputSource::ImageFileReader<InputSource::ImageMaskPathGenerator>("calib.txt", pathGenerator);

        //IMUSourceEnginge *imuSource;
        ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();

        //assume no pose
        InputSource::IMUSourceEngine *imuSource = NULL;

        //depth size is assumed to have the same size as image, fourth input has no effect

        printf("========reached here\n");
        InfiniTAM::Engine::CLIEngine::Instance()->Initialise(imageSource, imuSource, mainEngine, internalSettings->deviceType);
*/

        //we should finish everything up to UIEngine::Instance()->Initialise() up to here
        _m_start_of_time = std::chrono::high_resolution_clock::now();
		printf("================================InfiniTAM: setup finished==========================\n");

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


    virtual skip_option _p_should_skip() override{
        if(infinitam_stop==true)
        {
            return skip_option::skip_and_yield;
        }
        if(_m_sensor_data_it != _m_sensor_data.end())
        {
            //for dataset no need to yield, simply process the all dataset
            if(cur_timestamp < _m_sensor_data_it->first){
                cur_timestamp = _m_sensor_data_it->first;
                return skip_option::run;
            }
            else{
                ++_m_sensor_data_it;
                return skip_option::skip_and_yield;
            }
        }
        else {
            return skip_option::skip_and_yield;
        }
    }
    void _p_one_iteration() override 
	{
		printf("================================InfiniTAM: Info received==========================\n");
        if(!InfiniTAM::Engine::CLIEngine::Instance()->ProcessFrame())
        {
            printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!reached here\n");
            infinitam_stop=true;
        }
//      3/18 get latest pose from loaded dataset data
        auto latest_pose = _m_sensor_data.find(cur_timestamp);
        if(latest_pose == _m_sensor_data.end())
        {
            printf("pose for this timestamp is not found\n");
        }

	}

	virtual ~infinitam() override {
        //need to implement UIEngine::Instance()->shutdown()
        InfiniTAM::Engine::CLIEngine::Instance()->Shutdown();
	}

private:
	const std::shared_ptr<switchboard> sb;
    //3/18 use dataset
    const std::map<ullong, sensor_types> _m_sensor_data;
    std::map<ullong, sensor_types>::const_iterator _m_sensor_data_it;
    ullong cur_timestamp;
    ullong dataset_first_time;
    time_type _m_start_of_time;
    bool infinitam_stop;
};

PLUGIN_MAIN(infinitam)
