// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMBasicEngine.h"

#include "../Engines/LowLevel/ITMLowLevelEngineFactory.h"
#include "../Engines/Meshing/ITMMeshingEngineFactory.h"
#include "../Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../Engines/Visualisation/ITMVisualisationEngineFactory.h"
#include "../Objects/RenderStates/ITMRenderStateFactory.h"
#include "../Trackers/ITMTrackerFactory.h"

#include "../../ORUtils/NVTimer.h"
#include "../../ORUtils/FileUtils.h"
#include <Eigen/Dense>
//#define OUTPUT_TRAJECTORY_QUATERNIONS

using namespace ITMLib;

    template <typename TVoxel, typename TIndex>
ITMBasicEngine<TVoxel,TIndex>::ITMBasicEngine(const ITMLibSettings *settings, const ITMRGBDCalib& calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
    this->settings = settings;

    if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

    MemoryDeviceType memoryType = settings->GetMemoryType();
    this->scene = new ITMScene<TVoxel,TIndex>(&settings->sceneParams, settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED, memoryType);

    const ITMLibSettings::DeviceType deviceType = settings->deviceType;

    lowLevelEngine = ITMLowLevelEngineFactory::MakeLowLevelEngine(deviceType);
    viewBuilder = ITMViewBuilderFactory::MakeViewBuilder(calib, deviceType);
    visualisationEngine = ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxel,TIndex>(deviceType);

    meshingEngine = NULL;
    if (settings->createMeshingEngine)
        meshingEngine = ITMMeshingEngineFactory::MakeMeshingEngine<TVoxel,TIndex>(deviceType);

    denseMapper = new ITMDenseMapper<TVoxel,TIndex>(settings);
    denseMapper->ResetScene(scene);

    imuCalibrator = new ITMIMUCalibrator_iPad();

    tracker = ITMTrackerFactory::Instance().Make(imgSize_rgb, imgSize_d, settings, lowLevelEngine, imuCalibrator, scene->sceneParams);
    trackingController = new ITMTrackingController(tracker, settings);

    Vector2i trackedImageSize = trackingController->GetTrackedImageSize(imgSize_rgb, imgSize_d);

    renderState_live = ITMRenderStateFactory<TIndex>::CreateRenderState(trackedImageSize, scene->sceneParams, memoryType);
    renderState_freeview = NULL; //will be created if needed

    trackingState = new ITMTrackingState(trackedImageSize, memoryType);
    trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
    tracker->UpdateInitialPose(trackingState);

    //pyh steps needed to dump pose
    //pose_count=0;
    //pose_file.open("85_cabinet_calibfr3_voxel10_be_gt_output.csv");
    //param_file.open("tracker_param.csv");

    view = NULL; // will be allocated by the view builder

    if (settings->behaviourOnFailure == settings->FAILUREMODE_RELOCALISE)
        relocaliser = new FernRelocLib::Relocaliser<float>(imgSize_d, Vector2f(settings->sceneParams.viewFrustum_min, settings->sceneParams.viewFrustum_max), 0.2f, 500, 4);
    else relocaliser = NULL;

    kfRaycast = new ITMUChar4Image(imgSize_d, memoryType);

    trackingActive = true;
    fusionActive = true;
    mainProcessingActive = true;
    trackingInitialised = false;
    //trackingInitialised = true;
    relocalisationCount = 0;
    framesProcessed = 0;

    //ICP timer
    ICP_time = std::chrono::nanoseconds(0);
}
//pyh setting initial pose
template<typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::SetInitialPose(ORUtils::Matrix4<float> init_transform)
{
    //trackingState->pose_d->SetM(init_transform);
    trackingState->pose_d->SetInvM(init_transform);
    
    //Matrix4f approxInvPose = trackingState->pose_d->GetInvM();
    //std::cout <<"initial pose translation: "<<approxInvPose.m[12] <<" "<<approxInvPose.m[13]<<" "<<approxInvPose.m[14]<<"\n";
    //Eigen::Matrix3f testr;  
    //testr(0,0) = approxInvPose.m[0]; testr(0,1) = approxInvPose.m[4]; testr(0,2) = approxInvPose.m[8];
    //testr(1,0) = approxInvPose.m[1]; testr(1,1) = approxInvPose.m[5]; testr(1,2) = approxInvPose.m[9];
    //testr(2,0) = approxInvPose.m[2]; testr(2,1) = approxInvPose.m[6]; testr(2,2) = approxInvPose.m[10];
    //Eigen::Quaternionf testq(testr);
    //std::cout <<"initial pose orentaion: "<< testq.normalized().x() <<" "<<testq.normalized().y()<<" "<<testq.normalized().z()<<" "<<testq.normalized().w()<<"\n";

    //pyh step to dump to a file
    //std::vector<float> single_pose;
    //single_pose.push_back(trackingState->pose_d->GetM().m[12]);
    //single_pose.push_back(trackingState->pose_d->GetM().m[13]);
    //single_pose.push_back(trackingState->pose_d->GetM().m[14]);
    //Eigen::Matrix3f testr;
    //testr(0,0) = trackingState->pose_d->GetM().m[0];
    //testr(1,0) = trackingState->pose_d->GetM().m[1];
    //testr(2,0) = trackingState->pose_d->GetM().m[2];
    //testr(0,1) = trackingState->pose_d->GetM().m[4];
    //testr(1,1) = trackingState->pose_d->GetM().m[5];
    //testr(2,1) = trackingState->pose_d->GetM().m[6];
    //testr(0,2) = trackingState->pose_d->GetM().m[8];
    //testr(1,2) = trackingState->pose_d->GetM().m[9];
    //testr(2,2) = trackingState->pose_d->GetM().m[10];

    //Eigen::Quaternionf testq(testr);
    //single_pose.push_back(testq.normalized().x());
    //single_pose.push_back(testq.normalized().y());
    //single_pose.push_back(testq.normalized().z());
    //single_pose.push_back(testq.normalized().w());
    //pose_array.push_back(single_pose);
}
//pyh set seeding pose
template<typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::SetSeedingPose(ORUtils::Matrix4<float> gt_pose)
{
    //Matrix4f approxInvPose = gt_pose;
    //std::cout <<"input seeding pose translation: "<<approxInvPose.m[12] <<" "<<approxInvPose.m[13]<<" "<<approxInvPose.m[14]<<"\n";
    //Eigen::Matrix3f testr;  
    //testr(0,0) = approxInvPose.m[0]; testr(0,1) = approxInvPose.m[4]; testr(0,2) = approxInvPose.m[8];
    //testr(1,0) = approxInvPose.m[1]; testr(1,1) = approxInvPose.m[5]; testr(1,2) = approxInvPose.m[9];
    //testr(2,0) = approxInvPose.m[2]; testr(2,1) = approxInvPose.m[6]; testr(2,2) = approxInvPose.m[10];
    //Eigen::Quaternionf testq(testr);
    //std::cout <<"input seeding pose orentaion: "<< testq.normalized().x() <<" "<<testq.normalized().y()<<" "<<testq.normalized().z()<<" "<<testq.normalized().w()<<"\n";

    trackingState->gt_seeding_pose->SetInvM(gt_pose);
}

    template <typename TVoxel, typename TIndex>
ITMBasicEngine<TVoxel,TIndex>::~ITMBasicEngine()
{
    delete renderState_live;
    if (renderState_freeview != NULL) delete renderState_freeview;

    delete scene;

    delete denseMapper;
    delete trackingController;

    delete tracker;
    delete imuCalibrator;

    delete lowLevelEngine;
    delete viewBuilder;

    delete trackingState;
    if (view != NULL) delete view;

    delete visualisationEngine;

    if (relocaliser != NULL) delete relocaliser;
    delete kfRaycast;

    if (meshingEngine != NULL) delete meshingEngine;
    //param_file.close();
}

    template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::SaveSceneToMesh(const char *objFileName)
{
    //pyh: need to close pose & param file
    //std::cout<<"pose array size"<<pose_array.size()<<std::endl;
    for(auto i=0; i<pose_array.size(); i++)
    {
        for(auto j=0; j<(pose_array[i].size()-1); j++)
        {
            //pose_file<<pose_array[i][j]<<" ";
        }
        //pose_file<<pose_array[i][(pose_array[i].size()-1)]<<std::endl;
    }
    //pose_file.close();
    //pyh print icp time
    std::cout<<"ICP loop total time: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(trackingController->tracker->GetICPLoopTime()).count()<<"ns\n";
    std::cout<<"ICP loop total iteration: "<<trackingController->tracker->GetICPIterNo()<<"\n";
    std::cout<<"ICP total time: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(ICP_time).count()<<"ns\n";
    //pyh added to dump tc pose
    trackingController->DumpRecordedPose();
    if (meshingEngine == NULL) return;

    ITMMesh *mesh = new ITMMesh(settings->GetMemoryType());

    meshingEngine->MeshScene(mesh, scene);
    mesh->WriteSTL(objFileName);

    delete mesh;
}

    template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel, TIndex>::SaveToFile()
{
    // throws error if any of the saves fail

    std::string saveOutputDirectory = "State/";
    std::string relocaliserOutputDirectory = saveOutputDirectory + "Relocaliser/", sceneOutputDirectory = saveOutputDirectory + "Scene/";

    MakeDir(saveOutputDirectory.c_str());
    MakeDir(relocaliserOutputDirectory.c_str());
    MakeDir(sceneOutputDirectory.c_str());

    if (relocaliser) relocaliser->SaveToDirectory(relocaliserOutputDirectory);

    scene->SaveToDirectory(sceneOutputDirectory);
}

    template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel, TIndex>::LoadFromFile()
{
    std::string saveInputDirectory = "State/";
    std::string relocaliserInputDirectory = saveInputDirectory + "Relocaliser/", sceneInputDirectory = saveInputDirectory + "Scene/";

    ////TODO: add factory for relocaliser and rebuild using config from relocaliserOutputDirectory + "config.txt"
    ////TODO: add proper management of case when scene load fails (keep old scene or also reset relocaliser)

    this->resetAll();

    try // load relocaliser
    {
        FernRelocLib::Relocaliser<float> *relocaliser_temp = new FernRelocLib::Relocaliser<float>(view->depth->noDims, Vector2f(settings->sceneParams.viewFrustum_min, settings->sceneParams.viewFrustum_max), 0.2f, 500, 4);

        relocaliser_temp->LoadFromDirectory(relocaliserInputDirectory);

        delete relocaliser; 
        relocaliser = relocaliser_temp;
    }
    catch (std::runtime_error &e)
    {
        throw std::runtime_error("Could not load relocaliser: " + std::string(e.what()));
    }

    try // load scene
    {
        scene->LoadFromDirectory(sceneInputDirectory);
    }
    catch (std::runtime_error &e)
    {
        denseMapper->ResetScene(scene);
        throw std::runtime_error("Could not load scene:" + std::string(e.what()));
    }
}

    template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::resetAll()
{
    denseMapper->ResetScene(scene);
    trackingState->Reset();
}

//#ifdef OUTPUT_TRAJECTORY_QUATERNIONS
static int QuaternionFromRotationMatrix_variant(const double *matrix)
{
    int variant = 0;
    if
        ((matrix[4]>-matrix[8]) && (matrix[0]>-matrix[4]) && (matrix[0]>-matrix[8]))
        {
            variant = 0;
        }
    else if ((matrix[4]<-matrix[8]) && (matrix[0]>
                matrix[4]) && (matrix[0]> matrix[8])) {
        variant = 1;
    }
    else if ((matrix[4]> matrix[8]) && (matrix[0]<
                matrix[4]) && (matrix[0]<-matrix[8])) {
        variant = 2;
    }
    else if ((matrix[4]<
                matrix[8]) && (matrix[0]<-matrix[4]) && (matrix[0]< matrix[8])) {
        variant = 3;
    }
    return variant;
}

static void QuaternionFromRotationMatrix(const double *matrix, double *q) {
    /* taken from "James Diebel. Representing Attitude: Euler
       Angles, Quaternions, and Rotation Vectors. Technical Report, Stanford
       University, Palo Alto, CA."
       */

    // choose the numerically best variant...
    int variant = QuaternionFromRotationMatrix_variant(matrix);
    double denom = 1.0;
    if (variant == 0) {
        denom += matrix[0] + matrix[4] + matrix[8];
    }
    else {
        int tmp = variant * 4;
        denom += matrix[tmp - 4];
        denom -= matrix[tmp % 12];
        denom -= matrix[(tmp + 4) % 12];
    }
    denom = sqrt(denom);
    q[variant] = 0.5*denom;

    denom *= 2.0;
    switch (variant) {
        case 0:
            q[1] = (matrix[5] - matrix[7]) / denom;
            q[2] = (matrix[6] - matrix[2]) / denom;
            q[3] = (matrix[1] - matrix[3]) / denom;
            break;
        case 1:
            q[0] = (matrix[5] - matrix[7]) / denom;
            q[2] = (matrix[1] + matrix[3]) / denom;
            q[3] = (matrix[6] + matrix[2]) / denom;
            break;
        case 2:
            q[0] = (matrix[6] - matrix[2]) / denom;
            q[1] = (matrix[1] + matrix[3]) / denom;
            q[3] = (matrix[5] + matrix[7]) / denom;
            break;
        case 3:
            q[0] = (matrix[1] - matrix[3]) / denom;
            q[1] = (matrix[6] + matrix[2]) / denom;
            q[2] = (matrix[5] + matrix[7]) / denom;
            break;
    }

    if (q[0] < 0.0f) for (int i = 0; i < 4; ++i) q[i] *= -1.0f;
}
//#endif

    template <typename TVoxel, typename TIndex>
ITMTrackingState::TrackingResult ITMBasicEngine<TVoxel,TIndex>::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
    //printf("basic engine\n");
    // prepare image and turn it into a depth image
    if (imuMeasurement == NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter);
    else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

    if (!mainProcessingActive) return ITMTrackingState::TRACKING_FAILED;

    // tracking, pyh: probably change here to feed directly
    ORUtils::SE3Pose oldPose(*(trackingState->pose_d));
    
    if (trackingActive) {
        //std::cout<<"tesing ICP timer: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(ICP_time).count()<<"\n"; 
        auto ICP_begin = std::chrono::high_resolution_clock::now();
        trackingController->Track(trackingState, view); 
        auto ICP_end = std::chrono::high_resolution_clock::now();
        //std::cout<<"current iteration ICP timer: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(ICP_end-ICP_begin).count()<<"\n"; 
        ICP_time += ICP_end - ICP_begin;  
    }

    //pyh used for dumping ICP pose
    //std::vector<float> single_pose;
    //single_pose.push_back(trackingState->pose_d->GetM().m[12]);
    //single_pose.push_back(trackingState->pose_d->GetM().m[13]);
    //single_pose.push_back(trackingState->pose_d->GetM().m[14]);
    //Eigen::Matrix3f testr;
    //testr(0,0) = trackingState->pose_d->GetM().m[0];
    //testr(1,0) = trackingState->pose_d->GetM().m[1];
    //testr(2,0) = trackingState->pose_d->GetM().m[2];
    //testr(0,1) = trackingState->pose_d->GetM().m[4];
    //testr(1,1) = trackingState->pose_d->GetM().m[5];
    //testr(2,1) = trackingState->pose_d->GetM().m[6];
    //testr(0,2) = trackingState->pose_d->GetM().m[8];
    //testr(1,2) = trackingState->pose_d->GetM().m[9];
    //testr(2,2) = trackingState->pose_d->GetM().m[10];

    //Eigen::Quaternionf testq(testr);
    //single_pose.push_back(testq.normalized().x());
    //single_pose.push_back(testq.normalized().y());
    //single_pose.push_back(testq.normalized().z());
    //single_pose.push_back(testq.normalized().w());
    //pose_array.push_back(single_pose);
    
    ITMTrackingState::TrackingResult trackerResult = ITMTrackingState::TRACKING_GOOD;

    switch (settings->behaviourOnFailure) {
        case ITMLibSettings::FAILUREMODE_RELOCALISE:
            trackerResult = trackingState->trackerResult;
            break;
        case ITMLibSettings::FAILUREMODE_STOP_INTEGRATION:
            if (trackingState->trackerResult != ITMTrackingState::TRACKING_FAILED)
                trackerResult = trackingState->trackerResult;
            else trackerResult = ITMTrackingState::TRACKING_POOR;
            break;
        default:
            break;
    }

    //relocalisation
    int addKeyframeIdx = -1;
    if (settings->behaviourOnFailure == ITMLibSettings::FAILUREMODE_RELOCALISE)
    {
        if (trackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount > 0) relocalisationCount--;

        int NN; float distances;
        view->depth->UpdateHostFromDevice();

        //find and add keyframe, if necessary
        bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, trackingState->pose_d, 0, 1, &NN, &distances, trackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount == 0);

        //frame not added and tracking failed -> we need to relocalise
        if (!hasAddedKeyframe && trackerResult == ITMTrackingState::TRACKING_FAILED)
        {
            relocalisationCount = 10;

            // Reset previous rgb frame since the rgb image is likely different than the one acquired when setting the keyframe
            view->rgb_prev->Clear();

            const FernRelocLib::PoseDatabase::PoseInScene & keyframe = relocaliser->RetrievePose(NN);
            trackingState->pose_d->SetFrom(&keyframe.pose);

            denseMapper->UpdateVisibleList(view, trackingState, scene, renderState_live, true);
            trackingController->Prepare(trackingState, scene, view, visualisationEngine, renderState_live); 
            trackingController->Track(trackingState, view);

            trackerResult = trackingState->trackerResult;
        }
    }

    bool didFusion = false;
    if ((trackerResult == ITMTrackingState::TRACKING_GOOD || !trackingInitialised) && (fusionActive) && (relocalisationCount == 0)) {
        // fusion
        denseMapper->ProcessFrame(view, trackingState, scene, renderState_live);
        didFusion = true;
        if (framesProcessed > 50) trackingInitialised = true;

        framesProcessed++;
    }
    if (trackerResult == ITMTrackingState::TRACKING_GOOD || trackerResult == ITMTrackingState::TRACKING_POOR)
    {
        if (!didFusion){
            denseMapper->UpdateVisibleList(view, trackingState, scene, renderState_live);
        }
        // raycast to renderState_live for tracking and free visualisation
        trackingController->Prepare(trackingState, scene, view, visualisationEngine, renderState_live);

        if (addKeyframeIdx >= 0)
        {
            ORUtils::MemoryBlock<Vector4u>::MemoryCopyDirection memoryCopyDirection =
                settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU;

            kfRaycast->SetFrom(renderState_live->raycastImage, memoryCopyDirection);
        }
    }
    else 
    {
        printf("shouldn't be using old Pose");
        *trackingState->pose_d = oldPose;
    }
    /*
#ifdef OUTPUT_TRAJECTORY_QUATERNIONS
const ORUtils::SE3Pose *p = trackingState->pose_d;
double t[3];
double R[9];
double q[4];
for (int i = 0; i < 3; ++i) t[i] = p->GetInvM().m[3 * 4 + i];
for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
R[r * 3 + c] = p->GetM().m[c * 4 + r];
QuaternionFromRotationMatrix(R, q);
fprintf(stderr, "%f %f %f %f %f %f %f\n", t[0], t[1], t[2], q[1], q[2], q[3], q[0]);
#endif
*/   
    return trackerResult;
}

    template <typename TVoxel, typename TIndex>
ITMTrackingState::TrackingResult ITMBasicEngine<TVoxel,TIndex>::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ORUtils::Matrix4<float> cur_transform, ITMIMUMeasurement *imuMeasurement)
{
    //printf("basic modified engine\n");

    if (imuMeasurement == NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter);
    else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

    if (!mainProcessingActive) return ITMTrackingState::TRACKING_FAILED;

    // tracking, pyh: probably change here to feed directly
    ORUtils::SE3Pose oldPose(*(trackingState->pose_d));

    if (trackingActive) 
    {
        //ORUtils::Matrix4<float> temp_trans_inv;
        //temp_trans.inv(temp_trans_inv);
        //cur_transform.inv(cur_trans_inv);
        //trackingState->pose_d->SetM(cur_transform);
        trackingState->pose_d->SetInvM(cur_transform);
        //trackingState->pose_d->SetGT(cur_transform);
        
        //trackingState->pose_d->PrintM(1);
        //trackingState->pose_d->SetInvM(temp_trans_inv);

        //trackingState->pose_d->PrintM(2);
        //trackingState->pose_d->M = test;
        //next_pose->PrintM(0);
        //Matrix4f inv_M;
        //cur_transform.inv(inv_M);
        //trackingState->pose_d->SetInvM(inv_M);
        //trackingState->pose_d->Coerce();
        //trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;

        //trackingState->pose_d->SetInvM(cur_transform);
        //trackingController->tracker->SetInvPose(cur_transform);
        //trackingController->Track(trackingState, view);
        //Matrix4f M_d;
        //std::cout<<"prior: "<<trackingState->pose_d->GetM().m[0]<<"\n";
        //M_d = trackingState->pose_d->GetM();
        //
        //M_d.m[0] =100;
        //std::cout<<"after: "<<trackingState->pose_d->GetM().m[0]<<"\n";

        std::vector<float> single_pose;
        single_pose.push_back(trackingState->pose_d->GetM().m[12]);
        single_pose.push_back(trackingState->pose_d->GetM().m[13]);
        single_pose.push_back(trackingState->pose_d->GetM().m[14]);
        Eigen::Matrix3f testr;
        testr(0,0) = trackingState->pose_d->GetM().m[0];
        testr(1,0) = trackingState->pose_d->GetM().m[1];
        testr(2,0) = trackingState->pose_d->GetM().m[2];
        testr(0,1) = trackingState->pose_d->GetM().m[4];
        testr(1,1) = trackingState->pose_d->GetM().m[5];
        testr(2,1) = trackingState->pose_d->GetM().m[6];
        testr(0,2) = trackingState->pose_d->GetM().m[8];
        testr(1,2) = trackingState->pose_d->GetM().m[9];
        testr(2,2) = trackingState->pose_d->GetM().m[10];

        Eigen::Quaternionf testq(testr);
        Eigen::Quaternionf testq_norm = testq.normalized();
        single_pose.push_back(testq_norm.x());
        single_pose.push_back(testq_norm.y());
        single_pose.push_back(testq_norm.z());
        single_pose.push_back(testq_norm.w());
        pose_array.push_back(single_pose);

    }

    ITMTrackingState::TrackingResult trackerResult = ITMTrackingState::TRACKING_GOOD;

    int addKeyframeIdx = -1;
    //2nd modification
    switch (settings->behaviourOnFailure) {
        case ITMLibSettings::FAILUREMODE_RELOCALISE:
            //pyh
            //trackerResult = temp_trackingState->trackerResult;
            trackerResult = trackingState->trackerResult;
            break;
        case ITMLibSettings::FAILUREMODE_STOP_INTEGRATION:
            if (trackingState->trackerResult != ITMTrackingState::TRACKING_FAILED)
                trackerResult = trackingState->trackerResult;
            else trackerResult = ITMTrackingState::TRACKING_POOR;
            // if (temp_trackingState->trackerResult != ITMTrackingState::TRACKING_FAILED)
            //     trackerResult = temp_trackingState->trackerResult;
            // else trackerResult = ITMTrackingState::TRACKING_POOR;
            break;
        default:
            break;
    }

    //relocalisation


    if (settings->behaviourOnFailure == ITMLibSettings::FAILUREMODE_RELOCALISE)
    {
        if (trackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount > 0) relocalisationCount--;

        int NN; float distances;
        view->depth->UpdateHostFromDevice();

        //find and add keyframe, if necessary
        bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, trackingState->pose_d, 0, 1, &NN, &distances, trackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount == 0);

        // bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, temp_trackingState->pose_d, 0, 1, &NN, &distances, trackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount == 0);

        //frame not added and tracking failed -> we need to relocalise
        if (!hasAddedKeyframe && trackerResult == ITMTrackingState::TRACKING_FAILED)
        {
            relocalisationCount = 10;

            // Reset previous rgb frame since the rgb image is likely different than the one acquired when setting the keyframe
            view->rgb_prev->Clear();

            const FernRelocLib::PoseDatabase::PoseInScene & keyframe = relocaliser->RetrievePose(NN);
            trackingState->pose_d->SetFrom(&keyframe.pose);

            denseMapper->UpdateVisibleList(view, trackingState, scene, renderState_live, true);
            trackingController->Prepare(trackingState, scene, view, visualisationEngine, renderState_live); 
            trackingController->Track(trackingState, view);

            trackerResult = trackingState->trackerResult;

            //temp_trackingState->pose_d->SetFrom(&keyframe.pose);

            //denseMapper->UpdateVisibleList(view, temp_trackingState, scene, renderState_live, true);
            //temp_trackingController->Prepare(temp_trackingState, scene, view, visualisationEngine, renderState_live); 
            //temp_trackingController->Track(temp_trackingState, view);

            //trackerResult = temp_trackingState->trackerResult;
        }
    }

    bool didFusion = false;
    if ((trackerResult == ITMTrackingState::TRACKING_GOOD || !trackingInitialised) && (fusionActive) && (relocalisationCount == 0)) {
        // fusion
        denseMapper->ProcessFrame(view, trackingState, scene, renderState_live);
        //denseMapper->ProcessFrame(view, temp_trackingState, scene, renderState_live);

        didFusion = true;
        //pyh we don't really need this
        if (framesProcessed > 50) trackingInitialised = true;

        framesProcessed++;
    }
    //2nd modification

    if (trackerResult == ITMTrackingState::TRACKING_GOOD || trackerResult == ITMTrackingState::TRACKING_POOR)
    {
        if (!didFusion){
            denseMapper->UpdateVisibleList(view, trackingState, scene, renderState_live);
            //denseMapper->UpdateVisibleList(view, temp_trackingState, scene, renderState_live);
        }
        // raycast to renderState_live for tracking and free visualisation
        //temp_trackingController->Prepare(temp_trackingState, scene, view, visualisationEngine, renderState_live);
        trackingController->Prepare(trackingState, scene, view, visualisationEngine, renderState_live);

        if (addKeyframeIdx >= 0)
        {
            std::cout<<"never triggered\n";
            ORUtils::MemoryBlock<Vector4u>::MemoryCopyDirection memoryCopyDirection =
                settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU;

            kfRaycast->SetFrom(renderState_live->raycastImage, memoryCopyDirection);
        }
    }
    else 
    {
        printf("shouldn't be using old Pose");
        *trackingState->pose_d = oldPose;
    }

    /*  :  
#ifdef OUTPUT_TRAJECTORY_QUATERNIONS
const ORUtils::SE3Pose *p = trackingState->pose_d;
double t[3];
double R[9];
double q[4];
for (int i = 0; i < 3; ++i) t[i] = p->GetInvM().m[3 * 4 + i];
for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
R[r * 3 + c] = p->GetM().m[c * 4 + r];
QuaternionFromRotationMatrix(R, q);
fprintf(stderr, "%f %f %f %f %f %f %f\n", t[0], t[1], t[2], q[1], q[2], q[3], q[0]);
#endif
*/    
    return trackerResult;
}
    template <typename TVoxel, typename TIndex>
ITMTrackingState::TrackingResult ITMBasicEngine<TVoxel,TIndex>::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ORUtils::Vector6<float> cur_tangent, ITMIMUMeasurement *imuMeasurement)
{
    printf("basic Euler modified engine\n");

    if (imuMeasurement == NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter);
    else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

    if (!mainProcessingActive) return ITMTrackingState::TRACKING_FAILED;

    // tracking, pyh: probably change here to feed directly
    ORUtils::SE3Pose oldPose(*(trackingState->pose_d));
    /*
       trackingController->Track(trackingState, view);    
       ORUtils::SE3Pose *next_pose = new ORUtils::SE3Pose(cur_transform);
       trackingState->pose_d->SetFrom(next_pose);
       ITMTrackingState::TrackingResult trackerResult = ITMTrackingState::TRACKING_GOOD;
       */

    if (trackingActive) 
    {
        trackingController->Track(trackingState, view);
        ORUtils::SE3Pose *next_pose = new ORUtils::SE3Pose(cur_tangent);

        std::cout<<"processed transformation matrix : "<<std::endl;
        std::cout<<trackingState->pose_d->GetM().m[0] <<" "<<trackingState->pose_d->GetM().m[4]<<" "<<trackingState->pose_d->GetM().m[8]<<" "<<trackingState->pose_d->GetM().m[12]<<std::endl;
        std::cout<<trackingState->pose_d->GetM().m[1] <<" "<<trackingState->pose_d->GetM().m[5]<<" "<<trackingState->pose_d->GetM().m[9]<<" "<<trackingState->pose_d->GetM().m[13]<<std::endl;
        std::cout<<trackingState->pose_d->GetM().m[2] <<" "<<trackingState->pose_d->GetM().m[6]<<" "<<trackingState->pose_d->GetM().m[10]<<" "<<trackingState->pose_d->GetM().m[14]<<std::endl;
        std::cout<<trackingState->pose_d->GetM().m[3] <<" "<<trackingState->pose_d->GetM().m[7]<<" "<<trackingState->pose_d->GetM().m[11]<<" "<<trackingState->pose_d->GetM().m[15]<<"\n\n";

        const ORUtils::SE3Pose *p = trackingState->pose_d;

        double R[9];
        double our_R[9];
        double q[4];
        double our_q[4];
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c){
                R[r * 3 + c] = p->GetM().m[c * 4 + r];
                our_R[r * 3 + c] = next_pose->GetM().m[c * 4 + r];
            }
        }
        QuaternionFromRotationMatrix(R, q);
        QuaternionFromRotationMatrix(our_R, our_q);
        printf("infinitam quaternion\n  %f %f %f %f\n\n",  q[1], q[2], q[3], q[0]);
        printf("our quaternion based on infinitam conversion\n %f %f %f %f\n\n", our_q[1], our_q[2], our_q[3], our_q[0]);

        printf("tracker tx: %f, ty: %f, tz: %f\n", p->params.each.tx, p->params.each.ty, p->params.each.tz);
        printf("our tx: %f, ty: %f, tz: %f\n", next_pose->params.each.tx, next_pose->params.each.ty, next_pose->params.each.tz);
        printf("tracker rx: %f, ry: %f, rz: %f\n", p->params.each.rx, p->params.each.ry, p->params.each.rz);
        printf("our rx: %f, ry: %f, rz: %f\n", next_pose->params.each.rx, next_pose->params.each.ry, next_pose->params.each.rz);
        trackingState->pose_d->SetFrom(next_pose);

    }

    ITMTrackingState::TrackingResult trackerResult = ITMTrackingState::TRACKING_GOOD;


    switch (settings->behaviourOnFailure) {
        case ITMLibSettings::FAILUREMODE_RELOCALISE:
            trackerResult = trackingState->trackerResult;
            break;
        case ITMLibSettings::FAILUREMODE_STOP_INTEGRATION:
            if (trackingState->trackerResult != ITMTrackingState::TRACKING_FAILED)
                trackerResult = trackingState->trackerResult;
            else trackerResult = ITMTrackingState::TRACKING_POOR;
            break;
        default:
            break;
    }

    //relocalisation
    int addKeyframeIdx = -1;
    if (settings->behaviourOnFailure == ITMLibSettings::FAILUREMODE_RELOCALISE)
    {
        if (trackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount > 0) relocalisationCount--;

        int NN; float distances;
        view->depth->UpdateHostFromDevice();

        //find and add keyframe, if necessary
        bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, trackingState->pose_d, 0, 1, &NN, &distances, trackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount == 0);

        //frame not added and tracking failed -> we need to relocalise
        if (!hasAddedKeyframe && trackerResult == ITMTrackingState::TRACKING_FAILED)
        {
            relocalisationCount = 10;

            // Reset previous rgb frame since the rgb image is likely different than the one acquired when setting the keyframe
            view->rgb_prev->Clear();

            const FernRelocLib::PoseDatabase::PoseInScene & keyframe = relocaliser->RetrievePose(NN);
            trackingState->pose_d->SetFrom(&keyframe.pose);

            denseMapper->UpdateVisibleList(view, trackingState, scene, renderState_live, true);
            trackingController->Prepare(trackingState, scene, view, visualisationEngine, renderState_live); 
            trackingController->Track(trackingState, view);

            trackerResult = trackingState->trackerResult;
        }
    }


    bool didFusion = false;
    if ((trackerResult == ITMTrackingState::TRACKING_GOOD || !trackingInitialised) && (fusionActive) && (relocalisationCount == 0)) {
        // fusion
        //denseMapper->ProcessFrame(view, trackingState, scene, renderState_live);
        didFusion = true;
        if (framesProcessed > 50) trackingInitialised = true;

        framesProcessed++;
    }

    if (trackerResult == ITMTrackingState::TRACKING_GOOD || trackerResult == ITMTrackingState::TRACKING_POOR)
    {
        if (!didFusion) {
            denseMapper->UpdateVisibleList(view, trackingState, scene, renderState_live);
        }
        // raycast to renderState_live for tracking and free visualisation
        trackingController->Prepare(trackingState, scene, view, visualisationEngine, renderState_live);

        if (addKeyframeIdx >= 0)
        {
            ORUtils::MemoryBlock<Vector4u>::MemoryCopyDirection memoryCopyDirection =
                settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU;

            kfRaycast->SetFrom(renderState_live->raycastImage, memoryCopyDirection);
        }
    }
    else 
    {
        printf("shouldn't be using old Pose");
        *trackingState->pose_d = oldPose;
    }

    /*    
#ifdef OUTPUT_TRAJECTORY_QUATERNIONS
const ORUtils::SE3Pose *p = trackingState->pose_d;
double t[3];
double R[9];
double q[4];
for (int i = 0; i < 3; ++i) t[i] = p->GetInvM().m[3 * 4 + i];
for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
R[r * 3 + c] = p->GetM().m[c * 4 + r];
QuaternionFromRotationMatrix(R, q);
fprintf(stderr, "%f %f %f %f %f %f %f\n", t[0], t[1], t[2], q[1], q[2], q[3], q[0]);
#endif
*/    
    return trackerResult;
}

template <typename TVoxel, typename TIndex>
Vector2i ITMBasicEngine<TVoxel,TIndex>::GetImageSize(void) const
{
    return renderState_live->raycastImage->noDims;
}

    template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::GetImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose, ITMIntrinsics *intrinsics)
{
    if (view == NULL) return;

    out->Clear();

    switch (getImageType)
    {
        case ITMBasicEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
            out->ChangeDims(view->rgb->noDims);
            if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) 
                out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
            else out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
            break;
        case ITMBasicEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
            out->ChangeDims(view->depth->noDims);
            if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
            ITMVisualisationEngine<TVoxel, TIndex>::DepthToUchar4(out, view->depth);

            break;
        case ITMBasicEngine::InfiniTAM_IMAGE_SCENERAYCAST:
        case ITMBasicEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
        case ITMBasicEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
        case ITMBasicEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
            {
                // use current raycast or forward projection?
                IITMVisualisationEngine::RenderRaycastSelection raycastType;
                if (trackingState->age_pointCloud <= 0) raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_RAYCAST;
                else raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_FORWARDPROJ;

                // what sort of image is it?
                IITMVisualisationEngine::RenderImageType imageType;
                switch (getImageType) {
                    case ITMBasicEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
                        imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;
                        break;
                    case ITMBasicEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
                        imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
                        break;
                    case ITMBasicEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
                        imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
                        break;
                    default:
                        imageType = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS;
                }

                visualisationEngine->RenderImage(scene, trackingState->pose_d, &view->calib.intrinsics_d, renderState_live, renderState_live->raycastImage, imageType, raycastType);

                ORUtils::Image<Vector4u> *srcImage = NULL;
                if (relocalisationCount != 0) srcImage = kfRaycast;
                else srcImage = renderState_live->raycastImage;

                out->ChangeDims(srcImage->noDims);
                if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
                    out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
                else out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

                break;
            }
        case ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
        case ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
        case ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
        case ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE:
            {
                IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
                if (getImageType == ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
                else if (getImageType == ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
                else if (getImageType == ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;

                if (renderState_freeview == NULL)
                {
                    renderState_freeview = ITMRenderStateFactory<TIndex>::CreateRenderState(out->noDims, scene->sceneParams, settings->GetMemoryType());
                }

                visualisationEngine->FindVisibleBlocks(scene, pose, intrinsics, renderState_freeview);
                visualisationEngine->CreateExpectedDepths(scene, pose, intrinsics, renderState_freeview);
                visualisationEngine->RenderImage(scene, pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

                if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
                    out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
                else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
                break;
            }
        case ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN:
            break;
    };
}

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::turnOnTracking() { trackingActive = true; }

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::turnOffTracking() { trackingActive = false; }

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::turnOnIntegration() { fusionActive = true; }

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::turnOffIntegration() { fusionActive = false; }

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::turnOnMainProcessing() { mainProcessingActive = true; }

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::turnOffMainProcessing() { mainProcessingActive = false; }
