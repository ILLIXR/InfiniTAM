// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMLibSettings.h"
using namespace ITMLib;

#include <climits>
#include <cmath>
#include <sstream>
#include <stdexcept>

/// Returns the value of 'var' from the environment on success, returns 'defaultVal' otherwise
static bool getBooleanVar(std::string var, bool defaultVal) {
	const char *val_c_str = std::getenv(var.c_str());
	if (!val_c_str) {
		return defaultVal;
	}

	std::istringstream is(std::string{val_c_str});
	bool val;
	is >> std::boolalpha >> val;
	return val;
}

/// Returns the value of 'var' from the environment on success, returns 'defaultVal' otherwise
static double getDoubleVar(std::string var, double defaultVal) {
	const char *freq_c_str = std::getenv(var.c_str());
	if (!freq_c_str) {
		return defaultVal;
	}

	return std::stod(std::string{freq_c_str});
}

/// Returns the fusion frequency mode set in the environment
static ITMLibSettings::FreqMode getFreqMode() {
	const char *mode_c_str = std::getenv("freqMode");
	if (!mode_c_str) {
		return ITMLibSettings::FREQMODE_NONE;
	}

	std::string mode = std::string{mode_c_str};
	if (mode == "none")
		return ITMLibSettings::FREQMODE_NONE;
	else if (mode == "constant")
		return ITMLibSettings::FREQMODE_CONSTANT;
	else if (mode == "controller")
		return ITMLibSettings::FREQMODE_CONTROLLER;
	else
		return ITMLibSettings::FREQMODE_NONE;
}

ITMLibSettings::ITMLibSettings(void)
	: sceneParams(0.02f, 100, 0.005f, 0.2f, 3.0f, false), // Default
//	: sceneParams(0.02f, 500, 0.002f, 0.2f, 3.0f, false), // Used most of the time
//	: sceneParams(0.02f, 100, 0.005f, 0.2f, 3.0f, true), // For VCU robustness test
//	: sceneParams(0.1f, 100, 0.01f, 0.4f, 4.0f, false), // P parameters
	surfelSceneParams(0.5f, 0.6f, static_cast<float>(20 * M_PI / 180), 0.01f, 0.004f, 3.5f, 25.0f, 4, 1.0f, 5.0f, 20, 10000000, true, true)
{
#ifndef COMPILE_WITHOUT_CUDA
	deviceType = DEVICE_CUDA;
#else
#ifdef COMPILE_WITH_METAL
	deviceType = DEVICE_METAL;
#else
	deviceType = DEVICE_CPU;
#endif
#endif

	// Enables or disables using ICP for tracking
	useICP = getBooleanVar("useICP", false);

	// Enables or disables approximate raycast
	useApproximateRaycast = false;

	// Enables or disables decoupled raycasting
	useDecoupledRaycasting = getBooleanVar("decoupleRaycasting", false);
	if (useICP && useDecoupledRaycasting)
		throw std::runtime_error("Cannot decouple raycasting when ICP is enabled!");

	// Enables or disables point-only projection during visibility check
	useApproximateDepthCheck = getBooleanVar("approxDepthCheck", false);

	// Enables or disables using previous frame's visibility list
	usePreviousVisibilityList = getBooleanVar("usePrevList", true);

	// Enable or disable bilateral depth filtering
	useBilateralFilter = true;

	// Skips every other point when using the colour renderer for creating a point cloud
	skipPoints = true;

	// Create all the things required for marching cubes and mesh extraction
	// - uses additional memory (lots!)
	createMeshingEngine = true;

	// What to do on tracker failure: ignore, relocalise or stop integration - not supported in loop closure version
	behaviourOnFailure = FAILUREMODE_IGNORE;

	// How swapping works: disabled, fully enabled (still with dragons) and delete what's not visible - not supported in loop closure version
	swappingMode = SWAPPINGMODE_DISABLED;

	// Switch between various library modes - basic, with loop closure, etc.
	libMode = LIBMODE_BASIC;

	// Choose between various camera frequency modes - none, constant frequency, online controller
	freqMode = getFreqMode();

	// Frequency for fusion in constant frequency mode
	fusionFreq = getDoubleVar("fusionFrequency", MAX_FREQ);

	// Frequency for raycasting in decoupled mode
	raycastingFreq = getDoubleVar("raycastingFrequency", MAX_FREQ);

	// Default ICP tracking
	//trackerConfig = "type=icp,levels=rrrbb,minstep=1e-3,"
	//				"outlierC=0.01,outlierF=0.002,"
	//				"numiterC=10,numiterF=2,failureDec=5.0"; // 5 for normal, 20 for loop closure

	// Depth-only extended tracker
	//trackerConfig = "type=extended,levels=rrbb,useDepth=1,minstep=1e-4,"
	//				  "outlierSpaceC=0.1,outlierSpaceF=0.004,"
	//				  "numiterC=20,numiterF=50,tukeyCutOff=8,"
	//				  // "framesToSkip=0,framesToWeight=50,failureDec=20.0";
	//				  "framesToSkip=10,framesToWeight=15,failureDec=20.0"; // P parameters

	// For hybrid intensity+depth tracking
	trackerConfig = "type=extended,levels=bbbb,useDepth=1,useColour=1,"
					"colourWeight=0.3,minstep=1e-4,"
					"outlierColourC=0.175,outlierColourF=0.005,"
					"outlierSpaceC=0.1,outlierSpaceF=0.004,"
					"numiterC=20,numiterF=50,tukeyCutOff=8,"
					"framesToSkip=10,framesToWeight=20,failureDec=20.0";

	// Colour only tracking, using rendered colours
	//trackerConfig = "type=rgb,levels=rrbb";

	//trackerConfig = "type=imuicp,levels=tb,minstep=1e-3,outlierC=0.01,outlierF=0.005,numiterC=4,numiterF=2";
	//trackerConfig = "type=extendedimu,levels=ttb,minstep=5e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=5,tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0";

	// Surfel tracking
	if(libMode == LIBMODE_BASIC_SURFELS)
	{
		trackerConfig = "extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=20,tukeyCutOff=8,framesToSkip=0,framesToWeight=1,failureDec=20.0";
	}
}

MemoryDeviceType ITMLibSettings::GetMemoryType() const
{
	return deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
}
