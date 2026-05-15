#include "plugin.hpp"

#include "ITMLib/Utils/ITMLibSettings.h"
#include "ITMLib/ITMLibDefines.h"
#include "ORUtils/FileUtils.h"

#include <eigen3/Eigen/Dense>
#include <omp.h>
#include <set>
#include <tuple>
#include <iostream>
#include <memory>
#include <spdlog/spdlog.h>

#include "draco_illixr/io/ply_reader.h"
#include "draco_illixr/io/ply_property_writer.h"
#include "draco_illixr/io/file_utils.h"

using namespace ILLIXR;
using namespace ILLIXR::data_format;

infinitam::infinitam(const std::string& name_, phonebook *pb_)
        : plugin{name_, pb_}
        , switchboard_{phonebook_->lookup_impl<switchboard>()}
        , mesh_writer_{switchboard_->get_writer<mesh_type>("requested_scene")}
        , vb_list_{switchboard_->get_writer<vb_type>("unique_VB_list")} {
    //pyh For now, I just hardcode internal settings that exists in ScanNet.s
    //Later we might need to have a more intelligent way to get these variables

    internal_settings_ = new ITMLib::ITMLibSettings();
    internal_settings_->useICP = false;
    internal_settings_->useApproximateDepthCheck = false;
    internal_settings_->usePreviousVisibilityList = true;
    internal_settings_->freqMode = ITMLib::ITMLibSettings::FreqMode::FREQMODE_CONSTANT;
    internal_settings_->fusionFreq = 30.0;
    internal_settings_->useDecoupledRaycasting = true;
    internal_settings_->raycastingFreq = 1.0;

    thread_count_ = switchboard_->get_env_ulong("MESH_COMPRESS_PARALLELISM", 8);

    calib_ = new ITMLib::ITMRGBDCalib();
    std::string illixr_data = switchboard_->get_env("ILLIXR_DATA");

    if (illixr_data.empty()) {
        throw std::runtime_error("ILLIXR_DATA not set");
    }

    const std::string calib_subpath = "/calibration.txt";
    std::string calib_source{illixr_data + calib_subpath};
    if (!readRGBDCalib(calib_source.c_str(), *calib_)) {
        spdlog::get("illixr")->error("Read RGBD calibration file failed");
    }

    //pyh extract scene name
    std::size_t pos = illixr_data.find_last_of("/");
    scene_number_ = illixr_data.substr(pos + 1);
    spdlog::get("illixr")->debug("Scene number: {}", scene_number_);

    main_engine_ = new ITMLib::ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(
            internal_settings_,
            *calib_,
            calib_->intrinsics_rgb.imgSize,
            calib_->intrinsics_d.imgSize
    );



    //pyh first allocate for incoming depth & RGB image on CPU, then later copy to GPU
    input_raw_depth_image_ = new ITMShortImage(calib_->intrinsics_d.imgSize, true, false);
    input_RGB_image_ = new ITMUChar4Image(calib_->intrinsics_rgb.imgSize, true, false);

    if (internal_settings_->deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA) {
        spdlog::get("illixr")->info("Using the CUDA version of InfiniTAM");
    }

    switchboard_->schedule<scene_recon_type>(id_, "ScanNet_Data",
                                             [&](switchboard::ptr<const scene_recon_type> datum, std::size_t) {
                                                 this->process_frame(datum);
                                             });

    //pyh initialize mesh used for mesh extraction
    mesh_ = new ITMLib::ITMMesh(MEMORYDEVICE_CUDA, 0);

    //track how many frame InfiniTAM has processed
    frame_count_ = 0;

    if (!std::filesystem::exists(data_path_)) {
        if (!std::filesystem::create_directory(data_path_)) {
            spdlog::get("illixr")->error("Failed to create data directory.");
        }
    }
    sr_latency_.open(data_path_ + "/sr_latency.csv");

    try {
        uint temp = switchboard_->get_env_ulong("MESH_COMPRESS_PARALLELISM", 8);
        if (temp != 0) {
            thread_count_ = temp;
        } else {
            spdlog::get("illixr")->warn("infinitam: MESH_COMPRESS_PARALLELISM not set; using default {}",
                                         thread_count_);
        }
    } catch (...) {
        spdlog::get("illixr")->error("infinitam: MESH_COMPRESS_PARALLELISM invalid, using default {}",
                                     thread_count_);
    }

    try {
        uint temp = switchboard_->get_env_ulong("FPS");
        if (temp != 0) {
            fps_ = temp;
        } else {
            spdlog::get("illixr")->warn("infinitam: FPS not set; using default {}", fps_);
        }
    } catch (...) {
        spdlog::get("illixr")->error("infinitam: FPS invalid, using default {}", fps_);
    }

    spdlog::get("illixr")->info("================================InfiniTAM: setup finished==========================");
}

void infinitam::process_frame(switchboard::ptr<const scene_recon_type>& datum) {
    spdlog::get("illixr")->debug("================================InfiniTAM: frame %d received==========================", frame_count_);
    if (!datum->depth.empty()) {
        //pyh: convert to transformation matrix
        Eigen::Matrix3f rot = datum->pose.orientation.normalized().toRotationMatrix();
        ORUtils::Matrix4<float> cur_trans_matrix;
        cur_trans_matrix = {
                rot(0, 0), rot(1, 0), rot(2, 0), 0.0f,
                rot(0, 1), rot(1, 1), rot(2, 1), 0.0f,
                rot(0, 2), rot(1, 2), rot(2, 2), 0.0f,
                datum->pose.position.x(), datum->pose.position.y(), datum->pose.position.z(), 1.0f
        };

        // Set first pose
        if (frame_count_ == 0) {
            main_engine_->SetInitialPose(cur_trans_matrix);
        }

        cv::Mat cur_depth = datum->depth.clone();

        //pyh converting the to the InfiniTAM expected data structure
        auto depth_frame = reinterpret_cast<const short *>(cur_depth.datastart);
        short *cur_depth_head = input_raw_depth_image_->GetData(MEMORYDEVICE_CPU);
        std::memcpy(cur_depth_head, depth_frame, sizeof(short) * input_raw_depth_image_->dataSize);

        auto frame_start = std::chrono::high_resolution_clock::now();

        //pyh main reconstruction (volumetric fusion) function
        main_engine_->ProcessFrame(input_RGB_image_, input_raw_depth_image_, cur_trans_matrix);

        auto frame_end = std::chrono::high_resolution_clock::now();
        auto frame_duration = std::chrono::duration_cast<std::chrono::microseconds>(frame_end - frame_start).count();

        auto sinceEpoch = frame_start.time_since_epoch();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(sinceEpoch).count();
        sr_latency_ << "fuse " << frame_count_ << " " << (static_cast<double>(frame_duration) / 1000.0) << "\n";

        if ((frame_count_ % fps_) == 0 && frame_count_ > 0) {
            sr_latency_ << "start " << frame_count_ << " " << millis << "\n";
            auto start = std::chrono::high_resolution_clock::now();
#if !defined ACTIVE_SCENE
             mainEngine->GetMesh(mesh, 2);
#else
            main_engine_->GetMesh(mesh_, 1);
#endif
            //pyh This is for dumping out the mesh directly to file
            // std::string merge_name = this->scene_number_+ "_" + std::to_string(frame_count_) +".obj";
            // mesh_->WriteOBJ(merge_name.c_str());

            if (!cpu_triangles_ || cpu_triangles_->dataSize < mesh_->noTotalTriangles) {
                cpu_triangles_.reset(
                        new ORUtils::MemoryBlock<ITMLib::ITMMesh::Triangle>(mesh_->noTotalTriangles, MEMORYDEVICE_CPU));
            }

            cpu_triangles_->DirectSetFrom(
                    mesh_->triangles,
                    ORUtils::MemoryBlock<ITMLib::ITMMesh::Triangle>::CUDA_TO_CPU,
                    mesh_->noTotalTriangles);
            ITMLib::ITMMesh::Triangle *triangleArray = cpu_triangles_->GetData(MEMORYDEVICE_CPU);

            unsigned face_number = mesh_->noTotalTriangles;
            unsigned scene_id = (frame_count_ / fps_) - 1;

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            double duration_ms = static_cast<double>(duration) / 1000.0;
            sr_latency_ << "extract " << scene_id << " " << duration_ms << " " << face_number << "\n";


            auto VB_start = std::chrono::high_resolution_clock::now();
            std::set<std::tuple<int, int, int>> unique_VBs;
            for (unsigned i = 0; i < mesh_->updated_voxel_blocks.size(); i++) {
                unique_VBs.emplace(
                        (mesh_->updated_voxel_blocks[i][0]), (mesh_->updated_voxel_blocks[i][1]),
                                        (mesh_->updated_voxel_blocks[i][2]));
            }
            //pyh sending UVBL first
            vb_list_.put(vb_list_.allocate<vb_type>(vb_type{std::move(unique_VBs), scene_id}));

            auto VB_end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(VB_end - VB_start).count();
            duration_ms = static_cast<double>(duration) / 1000.0;
            sr_latency_ << "vb " << scene_id << " " << duration_ms << " " << unique_VBs.size() << "\n";

            auto roi_start = std::chrono::high_resolution_clock::now();
            bool set_active = false;
            omp_set_dynamic(0);
            omp_set_num_threads(static_cast<int>(thread_count_));
            unsigned numThreads = thread_count_;
            unsigned trianglesPerThread = (face_number + numThreads - 1) / numThreads;
            spdlog::get("illixr")->info("parallel compression, # of threads: {}, # of triangles/threads: {} ", numThreads,
                   trianglesPerThread);
#pragma omp parallel num_threads(numThreads)
            {
                unsigned thread_id = omp_get_thread_num();

                unsigned startTriangle = thread_id * trianglesPerThread;
                unsigned endTriangle = std::min((thread_id + 1) * trianglesPerThread, face_number);
                unsigned per_faces = endTriangle - startTriangle;
                unsigned per_vertices = per_faces * 3;
                std::unique_ptr<draco_illixr::PlyReader> ply_reader(new draco_illixr::PlyReader());

                ply_reader->format_ = draco_illixr::PlyReader::kAscii;
                ply_reader->element_index_["vertex"] = 0;
                ply_reader->elements_.emplace_back("vertex", per_vertices);
                ply_reader->elements_.back().AddProperty(draco_illixr::PlyProperty("x", draco_illixr::DT_FLOAT32, draco_illixr::DT_INVALID));
                ply_reader->elements_.back().AddProperty(draco_illixr::PlyProperty("y", draco_illixr::DT_FLOAT32, draco_illixr::DT_INVALID));
                ply_reader->elements_.back().AddProperty(draco_illixr::PlyProperty("z", draco_illixr::DT_FLOAT32, draco_illixr::DT_INVALID));

                ply_reader->element_index_["face"] = 1;
                ply_reader->elements_.emplace_back("face", per_faces);

                ply_reader->elements_.back().AddProperty(
                        draco_illixr::PlyProperty("vertex_indices", draco_illixr::DT_INT32, draco_illixr::DT_UINT8));

                ply_reader->elements_.back().AddProperty(
                        draco_illixr::PlyProperty("vb_x", draco_illixr::DT_INT32, draco_illixr::DT_INVALID));
                ply_reader->elements_.back().AddProperty(
                        draco_illixr::PlyProperty("vb_y", draco_illixr::DT_INT32, draco_illixr::DT_INVALID));
                ply_reader->elements_.back().AddProperty(
                        draco_illixr::PlyProperty("vb_z", draco_illixr::DT_INT32, draco_illixr::DT_INVALID));

                draco_illixr::PlyElement &vertex_element = ply_reader->elements_[0];
                draco_illixr::PlyElement &face_element = ply_reader->elements_[1];

                for (unsigned entry = startTriangle; entry < endTriangle; ++entry) {
                    for (int i = 0; i < vertex_element.num_properties(); ++i) {
                        draco_illixr::PlyProperty &prop = vertex_element.property(i);
                        draco_illixr::PlyPropertyWriter<float> prop_writer(&prop);
                        switch (i) {
                            case 0:
                                prop_writer.PushBackValue(triangleArray[entry].p0.x);
                                break;
                            case 1:
                                prop_writer.PushBackValue(triangleArray[entry].p0.y);
                                break;
                            case 2:
                                prop_writer.PushBackValue(triangleArray[entry].p0.z);
                                break;
                            default:
                                spdlog::get("illixr")->error("should not happen #1 ");
                                break;
                        }
                    }
                    for (int i = 0; i < vertex_element.num_properties(); ++i) {
                        draco_illixr::PlyProperty &prop = vertex_element.property(i);
                        draco_illixr::PlyPropertyWriter<float> prop_writer(&prop);
                        switch (i) {
                            case 0:
                                prop_writer.PushBackValue(triangleArray[entry].p1.x);
                                break;
                            case 1:
                                prop_writer.PushBackValue(triangleArray[entry].p1.y);
                                break;
                            case 2:
                                prop_writer.PushBackValue(triangleArray[entry].p1.z);
                                break;
                            default:
                                spdlog::get("illixr")->error("should not happen #1 ");
                                break;
                        }
                    }
                    for (int i = 0; i < vertex_element.num_properties(); ++i) {
                        draco_illixr::PlyProperty &prop = vertex_element.property(i);
                        draco_illixr::PlyPropertyWriter<float> prop_writer(&prop);
                        switch (i) {
                            case 0:
                                prop_writer.PushBackValue(triangleArray[entry].p2.x);
                                break;
                            case 1:
                                prop_writer.PushBackValue(triangleArray[entry].p2.y);
                                break;
                            case 2:
                                prop_writer.PushBackValue(triangleArray[entry].p2.z);
                                break;
                            default:
                                spdlog::get("illixr")->error("should not happen #1 ");
                                break;
                        }
                    }
                }

                for (int entry = 0; entry < face_element.num_entries(); ++entry) {
                    int actual_entry = static_cast<int>(startTriangle) + entry;
                    for (int i = 0; i < face_element.num_properties(); ++i) {
                        draco_illixr::PlyProperty &prop = face_element.property(i);
                        draco_illixr::PlyPropertyWriter<int32_t> prop_writer(&prop);
                        switch (i) {
                            case 0:
                                prop.list_data_.push_back(static_cast<long>(prop.data_.size()) / prop.data_type_num_bytes_);
                                prop.list_data_.push_back(3);
                                int val = entry * 3;
                                int val_1 = entry * 3 + 1;
                                int val_2 = entry * 3 + 2;
                                prop_writer.PushBackValue(val_2);
                                prop_writer.PushBackValue(val_1);
                                prop_writer.PushBackValue(val);
                                break;
                            case 1:
                                prop_writer.PushBackValue(triangleArray[actual_entry].vb_info.x);
                                break;
                            case 2:
                                prop_writer.PushBackValue(triangleArray[actual_entry].vb_info.y);
                                break;
                            case 3:
                                prop_writer.PushBackValue(triangleArray[actual_entry].vb_info.z);
                                break;
                            default:
                                break;
                        }
                    }
                }

                mesh_writer_.put(mesh_writer_.allocate<mesh_type>(
                        mesh_type{static_cast<uint>(omp_get_thread_num()), std::move(ply_reader), scene_id, 0,
                                  numThreads, per_faces, per_vertices, set_active}));

            }

            auto roi_end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(roi_end - roi_start).count();
            sr_latency_ << "gen " << scene_id << " " << (static_cast<double>(duration) / 1000.0) << "\n";

            sr_latency_.flush();
            //pyh reset tracking
            main_engine_->ResetActiveSceneTracking();
             spdlog::get("illixr")->info("================================InfiniTAM: frame {} finished==========================",
                    frame_count_);

        }
        ORcudaSafeCall(cudaDeviceSynchronize());
    } else {
        if (datum->depth.empty()) {
            spdlog::get("illixr")->info("depth empty");
        }
        if (datum->rgb.empty()) {
            spdlog::get("illixr")->info("rgb empty");
        }
    }
    if (datum->last_frame) {
        spdlog::get("illixr")->info("reached last frame at {}", frame_count_);
        sr_latency_.flush();
    }

    frame_count_++;
}


PLUGIN_MAIN(infinitam)
