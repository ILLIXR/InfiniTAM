#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"

//add infinitam libraries
#include "Apps/InfiniTAM_cli/CLIEngine.h"
#include "Apps/InfiniTAM/UIEngine.h"
#include "ITMLib/ITMLibDefines.h"
#include "ITMLib/Core/ITMBasicEngine.h"
#include "ORUtils/FileUtils.h"
#include <signal.h>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
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

            //std::string calib_source = std::string(cur_path) + "/InfiniTAM/calib_vcu.txt";
            //std::string calib_source = std::string(cur_path) + "/InfiniTAM/calib_vcu.txt";
            //std::string calib_source = std::string(cur_path) + "/InfiniTAM/calib_fr1.txt";
            //std::string calib_source = std::string(cur_path) + "/InfiniTAM/calib_fr3_website.txt";
            //std::string calib_source = std::string(cur_path) + "/InfiniTAM/calib_fr3_website_wrong_switch.txt";

            //read from calib.txt and convert to ITMRGBDCalib format
            //modified from ITMLib/Objects/Camera/ITMCalibIo::readRGBDCalib(
            calib = new ITMLib::ITMRGBDCalib();
            if(!readRGBDCalib(calib_source.c_str(), *calib)){
                printf("Read RGBD caliberation file failed\n");
            }

            ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();
            //pyh testing
            std::cout<<"rgb calibration: "<< calib->intrinsics_rgb.projectionParamsSimple.all<<"\n";
            std::cout<<"depth calibration: "<< calib->intrinsics_d.projectionParamsSimple.all<<"\n";
            std::cout<<"BilaterFilter: "<<internalSettings->useBilateralFilter<<"\n";
            mainEngine = new ITMLib::ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(
                    internalSettings,
                    *calib,
                    calib->intrinsics_rgb.imgSize,
                    calib->intrinsics_d.imgSize
                    );

            //dims, allocate_cpu?, allocate_gpu?
            inputRawDepthImage = new ITMShortImage(calib->intrinsics_d.imgSize, true, false);
            inputRGBImage = new ITMUChar4Image(calib->intrinsics_rgb.imgSize, true, false);
            //pyh changed this to also get groundtruth
            sb->schedule<rgb_depth_pose_type>(id, "rgb_depth_pose", [&](switchboard::ptr<const rgb_depth_pose_type> datum, std::size_t){ 
                    this->ProcessFrame(datum);
                    });
            is_first_pose=true;
            //adjusted_file.open("85_fr3_cabinet_calibfr3_voxel10_gt_input.csv");
            printf("================================InfiniTAM: setup finished==========================\n");
            output_mesh_name="914_room_CPU_debug.stl";
        }

        void ProcessFrame(switchboard::ptr<const rgb_depth_pose_type> datum)
        {
            printf("================================InfiniTAM: frame %d received==========================\n", frame_count);
            if(datum->depth.has_value() && datum->rgb.has_value())
            //if(datum->depth.has_value())
            {
                frame_count++;
                cv::Mat cur_depth = datum->depth.value();
                //enable color 
                cv::Mat cur_rgb = datum->rgb.value();

                //test tx ty tz 
                //step 1 input quaternion and position vector are retrieved
                Eigen::Vector3f cur_pos = datum->position.value();
                Eigen::Quaternionf cur_ori = datum->orientation.value();

                ORUtils::Matrix4<float> cur_trans;
                ORUtils::Matrix4<float> test_trans;

                //uncomment this if you want to position relative to first frame(no usecase yet)
                //if(is_first_pose)//save the inverse of first pose and position
                //{
                //    orientation_offset = cur_ori.inverse();
                //    first_pos = cur_pos;
                //}
                //
                ////step need to feed in groundtruth
                //Eigen::Quaternionf adjusted_orientation = cur_ori* orientation_offset ;
                ////Eigen::Quaternionf adjusted_orientation = orientation_offset * cur_ori;
                //Eigen::Vector3f adjusted_pos = cur_pos - first_pos;
                ////Eigen::Quaternionf view_rot = adjusted_orientation.inverse().normalized();
                ////Eigen::Matrix3f adjusted_rot = adjusted_orientation.inverse().normalized().toRotationMatrix();
                //Eigen::Quaternionf view_rot = adjusted_orientation.normalized();
                //Eigen::Matrix3f adjusted_rot = adjusted_orientation.normalized().toRotationMatrix();

                //uncomment this if you want to use ICP tracker pose or directly feeding groundtruth pose
                Eigen::Quaternionf adjusted_orientation = cur_ori;
                Eigen::Vector3f adjusted_pos = cur_pos;
                Eigen::Matrix3f adjusted_rot = adjusted_orientation.normalized().toRotationMatrix();

                //818 VCU coord system
                //Eigen::Vector3f adjusted_pos;
                //adjusted_pos(0) = - cur_pos(1); //-ty
                //adjusted_pos(1) = - cur_pos(0); //-tx
                //adjusted_pos(2) = - cur_pos(2); //-tz
                //Eigen::Quaternionf adjusted_orientation;
                //adjusted_orientation.y() = cur_ori.x();
                //adjusted_orientation.x() = cur_ori.y();
                //adjusted_orientation.z() = -cur_ori.z();
                //adjusted_orientation.w() = -cur_ori.w();
                //Eigen::Matrix3f adjusted_rot = adjusted_orientation.normalized().toRotationMatrix();
                
                //step 3 put them in a transformation matrix 
                cur_trans.m[0] = adjusted_rot(0,0); cur_trans.m[1] = adjusted_rot(1,0); cur_trans.m[2] = adjusted_rot(2,0);
                cur_trans.m[3] = 0.0f;

                cur_trans.m[4] = adjusted_rot(0,1); cur_trans.m[5] = adjusted_rot(1,1); cur_trans.m[6] = adjusted_rot(2,1);
                cur_trans.m[7] = 0.0f;

                cur_trans.m[8] = adjusted_rot(0,2); cur_trans.m[9] = adjusted_rot(1,2); cur_trans.m[10] = adjusted_rot(2,2);
                cur_trans.m[11] = 0.0f;

                cur_trans.m[12] = adjusted_pos(0); cur_trans.m[13] = adjusted_pos(1); cur_trans.m[14] = adjusted_pos(2);
                cur_trans.m[15] = 1.0f;
                //if this is the first seeding or full seeding mode,  we initialise the first pose
                //if(is_first_pose)
                //{
                //    std::cout<<"setting first pose \n";
                //    mainEngine->SetInitialPose(cur_trans);
                //    is_first_pose=false;
                //}
                //used for only full seeding change
                //mainEngine->SetSeedingPose(cur_trans);
                
                //dump input transformation matrix info
                //cur_trans.inv(test_trans);
                //test_trans = cur_trans;
                //std::vector<float> single_trans;
                //single_trans.push_back(test_trans.m[12]); single_trans.push_back(test_trans.m[13]); single_trans.push_back(test_trans.m[14]);
                //Eigen::Matrix3f testr;
                //testr(0,0) = test_trans.m[0]; testr(0,1) = test_trans.m[4]; testr(0,2) = test_trans.m[8];
                //testr(1,0) = test_trans.m[1]; testr(1,1) = test_trans.m[5]; testr(1,2) = test_trans.m[9];
                //testr(2,0) = test_trans.m[2]; testr(2,1) = test_trans.m[6]; testr(2,2) = test_trans.m[10];
                //Eigen::Quaternionf testq(testr);
                //single_trans.push_back(testq.x()); single_trans.push_back(testq.y()); single_trans.push_back(testq.z()); single_trans.push_back(testq.w());
                //gt_array.push_back(single_trans);
                //end dumping
    
                const Vector4u *color_frame = reinterpret_cast<const Vector4u*>(cur_rgb.datastart);
                const uint16_t *depth_frame = reinterpret_cast<const uint16_t*>(cur_depth.datastart);

                short *cur_depth_head = inputRawDepthImage->GetData(MEMORYDEVICE_CPU);
                Vector4u *cur_rgb_head = inputRGBImage->GetData(MEMORYDEVICE_CPU);

                std::memcpy(cur_rgb_head, color_frame, sizeof(Vector4u) *inputRGBImage->dataSize);
                std::memcpy(cur_depth_head, depth_frame, sizeof(short)  * inputRawDepthImage->dataSize);
                //ICP mode
                ITMLib::ITMTrackingState::TrackingResult tracking_status = mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);
                
                if(tracking_status == ITMLib::ITMTrackingState::TRACKING_FAILED)
                {
                    printf("tracking failed at frame %d\n", frame_count);
                    std::cout<<"producing mesh: "<<output_mesh_name<<std::endl;
                    mainEngine->SaveSceneToMesh(output_mesh_name.c_str());
                    std::exit(0);
                }
                //ground truth mode
                //mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, cur_trans);
                //for GPU version only
                //ORcudaSafeCall(cudaThreadSynchronize());
            }
            else{   printf("missing either rgb or depth or both\n");}
        }

        virtual ~infinitam() override{
            for(auto i=0; i<gt_array.size(); i++)
            {
                for(auto j=0; j<(gt_array[i].size()-1); j++)
                {
                    //adjusted_file<<gt_array[i][j]<<" ";
                }
                //adjusted_file<<gt_array[i][(gt_array[i].size()-1)]<<std::endl;
            }
            //adjusted_file.close();
            //change mesh name

            std::cout<<"producing mesh: "<<output_mesh_name<<std::endl;
            mainEngine->SaveSceneToMesh(output_mesh_name.c_str());

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
        Eigen::Vector3f first_pos;
        Eigen::Quaternionf orientation_offset;
        bool is_first_pose=false;
        unsigned frame_count=0;
        std::ofstream adjusted_file;
        std::vector<std::vector<float>> gt_array;
        std::string output_mesh_name;
};

PLUGIN_MAIN(infinitam)
