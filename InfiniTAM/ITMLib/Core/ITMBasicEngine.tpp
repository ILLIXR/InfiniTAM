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
#include <fstream>
#include <sstream>

//#define OUTPUT_TRAJECTORY_QUATERNIONS

// #define VCU

#define ILLIXR_PLUGIN

using namespace ITMLib;

template <typename TVoxel, typename TIndex>
ITMBasicEngine<TVoxel,TIndex>::ITMBasicEngine(const ITMLibSettings *settings, const ITMRGBDCalib& calib, Vector2i imgSize_rgb, Vector2i imgSize_d, const char* pose_path)
{
	this->settings = settings;
	fpose.open("/home/henrydc/infinitam_pose.txt", std::fstream::out);

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
	tracker->UpdateInitialPose(trackingState);

	view = NULL; // will be allocated by the view builder
	
	if (settings->behaviourOnFailure == settings->FAILUREMODE_RELOCALISE)
		relocaliser = new FernRelocLib::Relocaliser<float>(imgSize_d, Vector2f(settings->sceneParams.viewFrustum_min, settings->sceneParams.viewFrustum_max), 0.2f, 500, 4);
	else relocaliser = NULL;

	kfRaycast = new ITMUChar4Image(imgSize_d, memoryType);

	std::vector<double> divisors{10.0f, 9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f};
	pidController = new ORUtils::PIDController(
		0.4f,       // Kp
		0.9f,       // Ki
		1.0f,       // Kd
		0.4f,       // alpha
		divisors    // Possible divisors
	);

	trackingActive = true;
	fusionActive = true;
	mainProcessingActive = true;
	trackingInitialised = false;
	relocalisationCount = 0;
	framesProcessed = 0;

	if (!settings->useICP)
	{
		// std::cout << "Path to groundtruth: " << pose_path << std::endl;
		// this->loadPoseQuat(pose_path);

		// std::cout << "Loaded queue size: " << this->seq_pose.size() << std::endl;
		// std::vector<double> loaded_pose = this->seq_pose.front();

		// ORUtils::Matrix4<float> assigned_M;
		// Quaternion2Matrix(loaded_pose, assigned_M);
		// trackingState->pose_d->SetInvM(assigned_M.m);
	}

#ifdef VCU
	// Define extrinsic matrix here.
	// Target coodinate system should be the same as GT.
	// Row-major order. .m[0] -> .m[15]

	// Timu2c_1 mostly used one
	matrix_TCtoI = {
		0.00193013, -0.999996, 0.00223829, 0.0,
		-0.999997, -0.0019327, -0.00114906, 0.0,
		0.00115338, -0.00223606, -0.999997, 0.0,
		-0.00817048, 0.015075, -0.0110795, 1.0};

	// Timu2c_2 the one mentioned here: https://vcu-rvi-dataset.github.io/
	// matrix_TCtoI = {
	// 	0.00193013, -0.999996, 0.00223829, 0.0,
	// 	-0.999997, -0.0019327, -0.00114906, 0.0,
	// 	0.00115338, -0.00223606, -0.999997, 0.0,
	// 	-0.007977467, -0.0849246, -0.010855671, 1.0};


	////////////////////////////////////////////////////////////////////////////////////////////////////
	// T': inverse of T
	// P_cam = T * P_model 	// defination from InfiniTAM
	// T' * P_cam = P_model
	////////////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// Mode 1: Assume extrinsic for cam -> imu
	// P_imu = T_c2i * P_cam
	// T'_c2i * P_imu = P_cam
	// T' * T'_c2i * P_imu = P_cam				// write: fetch pose T' from ITM, right multiply T'_c2i
	// T' * T'_c2i * T_c2i * P_cam = P_model	// read: load pose T' * T'_c2i, right multiply T_c2i

	/* Dump */
	// ORUtils::Matrix4<float> inv_CtoI;
	// matrix_TCtoI.inv(inv_CtoI);
	// in_pose = in_pose.GetM() * inv_CtoI;

	/* Load */
	// out_pose = out_pose * matrix_TCtoI;
	////////////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// Mode 2: Assume extrinsic for imu -> cam
	// P_cam = T_i2c * P_imu
	// T'_i2c * P_cam = P_imu
	// T' * T_i2c * P_imu = P_model				// write: fetch pose T' from ITM, right multiply T_i2c
	// T' * T_i2c * T'_i2c * P_cam = P_model	// read: load pose T' * T_i2c, right multiply T'_i2c

	/* Dump */
	// in_pose = in_pose.GetM() * matrix_TCtoI;

	/* Load */
    // ORUtils::Matrix4<float> inv_CtoI;
	// matrix_TCtoI.inv(inv_CtoI);
	// out_pose = out_pose * inv_CtoI;
	////////////////////////////////////////////////////////////////////////////////////////////////////
#endif
}

template <typename TVoxel, typename TIndex>
ITMBasicEngine<TVoxel,TIndex>::~ITMBasicEngine()
{

	if (settings->useICP)
	{
		this->dumpPoseQuat("ITM-ICP-Pose.txt");
		this->SaveSceneToMesh("ITM-ICP.obj");
	}
	else
	{
		this->SaveSceneToMesh("ITM-BE.obj");
	}

	{
		unsigned currBricks = 0;
		std::ofstream dataFile("brick_data.csv");
		dataFile << "#Frame,New Bricks,Visible Bricks,Average Confidence/Voxel\n";
		for (unsigned idx = 0; idx < newBricks.size(); idx++)
		{
			// `newBricks` contains cumulative data, so we need to subtract the
			// previous value to get the per-frame value
			dataFile << idx << ","
				<< newBricks[idx] - currBricks << ","
				<< visibleBricks[idx] << ","
				<< scene->averageConfidence[idx] << "\n";
			currBricks = newBricks[idx];
		}
		dataFile.close();
	}

	{
		unsigned currBricks = 0;
		std::ofstream dataFile("controller_data.csv");
		dataFile << "#Frame,New Bricks,Frequency\n";
		for (unsigned idx = 0; idx < newBricks.size(); idx++)
		{
			// `newBricks` contains cumulative data, so we need to subtract the
			// previous value to get the per-frame value
			dataFile << idx << ","
				<< newBricks[idx] - currBricks << ","
				<< frequency[idx] << "\n";
			currBricks = newBricks[idx];
		}
		dataFile.close();
	}

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
	delete pidController;
}

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::SaveSceneToMesh(const char *objFileName)
{
	if (meshingEngine == NULL) return;

	const int allocatedBricks = scene->index.getNumAllocatedVoxelBlocks() - scene->localVBA.lastFreeBlockId - 1;
	std::cout << "allocatedBricks: " << allocatedBricks << std::endl;
	const int allocatedVoxels = allocatedBricks * SDF_BLOCK_SIZE3;

#ifdef DEBUG
	std::cout << "[DEBUG] Generating mesh for " << allocatedBricks << " bricks (" << allocatedVoxels << " voxels)\n";
#endif

	ITMMesh *mesh = new ITMMesh(settings->GetMemoryType(), allocatedVoxels);
	meshingEngine->MeshScene(mesh, scene);
	mesh->WriteOBJ(objFileName);

	delete mesh;
	std::cout << "Mesh is written to " << objFileName << std::endl;
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

// #ifdef OUTPUT_TRAJECTORY_QUATERNIONS
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
// #endif

void printMatrix(ORUtils::SE3Pose pose)
{
	for (int r = 0; r < 16; r++)
	{
		std::cout << pose.GetM().m[r] << " ";
		if ((r+1)%4 == 0)
			std::cout << std::endl;
	}
	std::cout << std::endl;
}

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::dumpPoseQuat(std::string filename)
{
	std::cout << "Total num of poses: " << this->seq_pose.size() << std::endl;

	std::ofstream output (filename, std::ofstream::out);
	if (output.is_open())
	{
		while (this->seq_pose.size())
		{
			std::vector<double> p = this->seq_pose.front();
			this->seq_pose.pop();

#ifdef DEBUG
			fprintf(stderr, "Output pose: %f %f %f %f %f %f %f %f\n", p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
#endif
			// timestamp, tx, ty, tz, qx, qy, qz, qw
			output << std::to_string(p[0]) << " "
			       << p[1] << " "
			       << p[2] << " "
			       << p[3] << " "
			       << p[4] << " "
			       << p[5] << " "
			       << p[6] << " "
			       << p[7] << std::endl;
		}
	}
	output.close();
	std::cout << "Poses are written to " << filename << std::endl;
}

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::loadPoseQuat(const char *filename)
{
	// clear the queue before writing
	std::queue<std::vector<double>> empty;
	std::swap(this->seq_pose, empty);

	std::ifstream file;
	file.open(filename);
	if (!file.good())
		std::cerr << "[Error] Unable to load poses !!!" << std::endl;
	std::string line;
	std::vector<double> pose;

	while (file.good() && (getline(file, line)))
	{
#ifdef DEBUG
		std::cout << "[DEBUG] line: " << line << std::endl;
#endif

		pose.clear();

		if (line.find("#") != std::string::npos)
			continue;

		std::istringstream iss(line);
		double val;
		while (iss >> val)
		{
#ifdef DEBUG
			std::cout << "[DEBUG] val: " << val << std::endl;
#endif
			pose.push_back(val);
		}

		// Specific to the format of the associated file
		// ts, 7 poses, ts, path_depth, ts, path_color
		// need to drop the timestamp before path_depth
		pose.pop_back();

#ifdef DEBUG
		std::cout << "[DEBUG] Loading: ";
		for (auto val : pose)
			std::cout << val << " ";
		std::cout << std::endl;
#endif

		this->seq_pose.push(pose);
	}
	file.close();
}

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::Matrix2Quaternion(ORUtils::SE3Pose &in_pose)
{
	// Inversion: model2frame -> frame2model
	in_pose = in_pose.GetInvM();

#ifdef VCU
	// Mode 1: Assume extrinsic for cam -> imu
	ORUtils::Matrix4<float> inv_CtoI;
	matrix_TCtoI.inv(inv_CtoI);
	in_pose = in_pose.GetM() * inv_CtoI;

	// Mode 2: Assume extrinsic for imu -> cam
	// in_pose = in_pose.GetM() * matrix_TCtoI;
#endif

	double t[3];
	double R[9];
	double q[4];

#ifdef DEBUG
	std::cout << "[DEBUG] After applying extrinsic matrix: " << std::endl;
	printMatrix(in_pose);
#endif

	for (int i = 0; i < 3; ++i)
		t[i] = in_pose.GetM().m[3 * 4 + i];

	for (int r = 0; r < 3; ++r)
		for (int c = 0; c < 3; ++c)
			R[r * 3 + c] = in_pose.GetM().m[r * 4 + c];

	QuaternionFromRotationMatrix(R, q);

	// TUM order: timestamp, tx, ty, tz, rx, ry, rz, rw
	std::cout << "timestamp: " <<this->currentTimeStamp.c_str() << std::endl;
	std::vector<double> out_pose = {std::stod(this->currentTimeStamp.c_str()), t[0], t[1], t[2], q[1], q[2], q[3], q[0]};
	this->seq_pose.push(out_pose);

	fprintf(stderr, "Estimated pose: %f %f %f %f %f %f %f %f\n", out_pose[0], t[0], t[1], t[2], q[1], q[2], q[3], q[0]);
}

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::Quaternion2Matrix(std::vector<double> &in_pose, ORUtils::Matrix4<float> &out_pose)
{
	std::cout << "Loaded pose: ";
	for (auto val : in_pose)
		std::cout << val << " ";
	std::cout << std::endl;

	Eigen::Quaternionf quat;
	quat.w() = in_pose[7];
	quat.x() = in_pose[4];
	quat.y() = in_pose[5];
	quat.z() = in_pose[6];

	Eigen::Matrix3f rot = quat.normalized().toRotationMatrix();
	Eigen::Vector3f pos = {in_pose[1], in_pose[2], in_pose[3]};

	// ICP pose mapping
	out_pose = {
		rot(0, 0), rot(1, 0), rot(2, 0), 0.0f,
		rot(0, 1), rot(1, 1), rot(2, 1), 0.0f,
		rot(0, 2), rot(1, 2), rot(2, 2), 0.0f,
		pos[0],    pos[1],    pos[2],    1.0f};

#ifdef DEBUG
	std::cout << "[DEBUG] Print loaded matrix values:" << std::endl;
	std::cout << rot(0,0) << " " << rot(1,0) << " " << rot(2,0) << " " << "0" << std::endl;
	std::cout << rot(0,1) << " " << rot(1,1) << " " << rot(2,1) << " " << "0" << std::endl;
	std::cout << rot(0,2) << " " << rot(1,2) << " " << rot(2,2) << " " << "0" << std::endl;
	std::cout << pos[0]   << " " << pos[1]   << " " << pos[2]   << " " << "1" << std::endl;
	std::cout << std::endl;
#endif

#ifdef VCU
	// Mode 1
	out_pose = out_pose * matrix_TCtoI;

	// Mode 2
    // ORUtils::Matrix4<float> inv_CtoI;
	// matrix_TCtoI.inv(inv_CtoI);
	// out_pose = out_pose * inv_CtoI;
#endif
}


template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel, TIndex>::SkipFrame(void)
{
	// Only required for the gt tracker
	if (settings->useICP)
		return;

	if (!seq_pose.empty())
		seq_pose.pop();
}


template <typename TVoxel, typename TIndex>
ITMTrackingState::TrackingResult ITMBasicEngine<TVoxel,TIndex>::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
	std::cout << "============= Begin Basic Engine  ===============" << std::endl;
	// prepare image and turn it into a depth image
	if (imuMeasurement == NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

	if (!mainProcessingActive) return ITMTrackingState::TRACKING_FAILED;


	// tracking
	ORUtils::SE3Pose oldPose(*(trackingState->pose_d));

	if (settings->useICP)
	{
		if (trackingActive) trackingController->Track(trackingState, view);
		ORUtils::SE3Pose trackPose(*(trackingState->pose_d));

#ifdef DEBUG
		std::cout << "[DEBUG] Fetched pose from InfiniTAM: " << std::endl;
		printMatrix(trackPose);

		ORUtils::SE3Pose invPose = trackPose.GetInvM();
		std::cout << "[DEBUG] Inversed pose from InfinTAM: " << std::endl;
		printMatrix(invPose);

		std::cout << "[DEBUG] Double inversed pose for sanity check: " << std::endl;
		printMatrix(invPose.GetInvM());

		std::cout << "[DEBUG] Before applying extrinsic matrix: " << std::endl;
		printMatrix(trackPose);
#endif // End of DEBUG

		Matrix2Quaternion(trackPose);

#ifdef DEBUG
		// params is a private variable
		// std::cout << "before tx: " << trackingState->pose_d->params.each.tx << std::endl;
		// std::cout << "before ty: " << trackingState->pose_d->params.each.ty << std::endl;
		// std::cout << "before tz: " << trackingState->pose_d->params.each.tz << std::endl;
		// std::cout << "before rx: " << trackingState->pose_d->params.each.rx << std::endl;
		// std::cout << "before ry: " << trackingState->pose_d->params.each.ry << std::endl;
		// std::cout << "before rz: " << trackingState->pose_d->params.each.rz << std::endl;
		// std::cout << std::endl;
#endif // End of DEBUG
	}
	else
	{
		// Uncomment to use loaded pose from text file
		// input from text file
		// std::cout << "Current queue size: " << this->seq_pose.size() << std::endl;
		// std::vector<double> loaded_pose = this->seq_pose.front();
		// this->seq_pose.pop();

		// Use loaded_pose from ILLIXR
		// trackingState->pose_d->SetInvM(input_mat);

		if (input_pose.size() != 8) {
			std::cout << "Input pose from ILLIXR is invalid" << std::endl;
			exit(EXIT_FAILURE);
		}
		std::vector<double> loaded_pose = input_pose;
		input_pose.clear();

		ORUtils::Matrix4<float> assigned_M;
		Quaternion2Matrix(loaded_pose, assigned_M);

		trackingState->pose_d->SetInvM(assigned_M.m);

#ifdef DEBUG
		std::cout << "[DEBUG] Print assigned pose matrix: " << std::endl;
		ORUtils::SE3Pose sanityAssignedPose(*(trackingState->pose_d));
		printMatrix(sanityAssignedPose);

		// std::cout << "after tx: " << trackingState->pose_d->params.each.tx << std::endl;
		// std::cout << "after ty: " << trackingState->pose_d->params.each.ty << std::endl;
		// std::cout << "after tz: " << trackingState->pose_d->params.each.tz << std::endl;
		// std::cout << "after rx: " << trackingState->pose_d->params.each.rx << std::endl;
		// std::cout << "after ry: " << trackingState->pose_d->params.each.ry << std::endl;
		// std::cout << "after rz: " << trackingState->pose_d->params.each.rz << std::endl;
#endif // End of DEBUG
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
		denseMapper->ProcessFrame(view, trackingState, scene, renderState_live);
		didFusion = true;
		if (framesProcessed > 50) trackingInitialised = true;

		framesProcessed++;
	}

	// Only raycast in coupled mode. In decoupled mode, raycasting will be run separately
	if (!settings->useDecoupledRaycasting && (trackerResult == ITMTrackingState::TRACKING_GOOD || trackerResult == ITMTrackingState::TRACKING_POOR))
	{
		if (!didFusion) denseMapper->UpdateVisibleList(view, trackingState, scene, renderState_live);

		// raycast to renderState_live for tracking and free visualisation
		trackingController->Prepare(trackingState, scene, view, visualisationEngine, renderState_live);

		if (addKeyframeIdx >= 0)
		{
			ORUtils::MemoryBlock<Vector4u>::MemoryCopyDirection memoryCopyDirection =
				settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU;

			kfRaycast->SetFrom(renderState_live->raycastImage, memoryCopyDirection);
		}
	}
	else if (!settings->useDecoupledRaycasting)
	{
		*trackingState->pose_d = oldPose;
	}

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

	// Collect stats
	newBricks.push_back(scene->index.getNumAllocatedVoxelBlocks() - scene->localVBA.lastFreeBlockId - 1);
	visibleBricks.push_back(((ITMRenderState_VH *) renderState_live)->noVisibleEntries);

	std::cout << "============== End Basic Engine  ================" << std::endl;
    
	return trackerResult;
}


template <typename TVoxel, typename TIndex>
unsigned ITMBasicEngine<TVoxel, TIndex>::GetNumNewBricks(void) const
{
	const unsigned newBricksSize = newBricks.size();

	if (newBricksSize == 0)
	{
		return 0;
	}
	else if (newBricksSize == 1)
	{
		return newBricks[0];
	}
	else
	{
		return (newBricks[newBricksSize - 1] - newBricks[newBricksSize - 2]);
	}
}


template <typename TVoxel, typename TIndex>
unsigned ITMBasicEngine<TVoxel, TIndex>::GetFreqDivisor(void)
{
	const double maxFreq = ITMLibSettings::MAX_FREQ;

	switch (settings->freqMode) {
	case ITMLibSettings::FREQMODE_NONE: {
		frequency.push_back(maxFreq);
		return 1;
	}
	case ITMLibSettings::FREQMODE_CONSTANT: {
		double fusionFreq = settings->fusionFreq;
		frequency.push_back(fusionFreq);
		return static_cast<unsigned>(maxFreq / fusionFreq);
	}
	case ITMLibSettings::FREQMODE_CONTROLLER: {
		const unsigned newBricksSize = newBricks.size();
		unsigned numNewBricks;

		if (newBricksSize == 0)
		{
			// No data for first frame, so just run at full frequency
			frequency.push_back(maxFreq);
			return 1;
		}
		else if (newBricksSize == 1)
		{
			// This is a really ugly hack to make sure the controller doesn't see
			// the first brick allocation. This is because the first allocation
			// will be of thousands of bricks and will completely skew the
			// controller's set points.
			frequency.push_back(maxFreq);
			return 1;
		}
		else
		{
			numNewBricks = newBricks[newBricksSize - 1] - newBricks[newBricksSize - 2];
		}

		double divisor = pidController->Calculate(numNewBricks);
		frequency.push_back(maxFreq / divisor);
#ifdef DEBUG
		std::cout << "[DEBUG] PID controller output: " << maxFreq << "/" << divisor << " Hz\n";
#endif
		return static_cast<unsigned>(divisor);
	}
	default: {
		throw std::runtime_error("Illegal frequency mode!");
	}
	}
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
