// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMSceneParams.h"
#include "ITMSurfelSceneParams.h"
#include "../../ORUtils/MemoryDeviceType.h"

namespace ITMLib
{
	class ITMLibSettings
	{
	public:
		/// The device used to run the DeviceAgnostic code
		typedef enum {
			DEVICE_CPU,
			DEVICE_CUDA,
			DEVICE_METAL
		} DeviceType;

		typedef enum
		{
			FAILUREMODE_RELOCALISE,
			FAILUREMODE_IGNORE,
			FAILUREMODE_STOP_INTEGRATION
		} FailureMode;

		typedef enum
		{
			SWAPPINGMODE_DISABLED,
			SWAPPINGMODE_ENABLED,
			SWAPPINGMODE_DELETE
		} SwappingMode;

		typedef enum
		{
			LIBMODE_BASIC,
			LIBMODE_BASIC_SURFELS,
			LIBMODE_LOOPCLOSURE
		} LibMode;

		/// Frequency mode for incoming images
		typedef enum
		{
			FREQMODE_NONE,
			FREQMODE_CONSTANT,
			FREQMODE_CONTROLLER
		} FreqMode;

		/// Select the type of device to use
		DeviceType deviceType;

		/// Use ICP for tracking, otherwise use file pose
		bool useICP;

		/// Do not raycast every frame
		bool useApproximateRaycast;

		/// Only project depth point (and not ray) during visibility check
		bool useApproximateDepthCheck;

		/// Use visibility list of previous frame during visibility check
		bool usePreviousVisibilityList;

		/// Apply bilateral filter to depth image before using it
		bool useBilateralFilter;

		/// For ITMColorTracker: skip every other point in energy function evaluation.
		bool skipPoints;

		bool createMeshingEngine;

		FailureMode behaviourOnFailure;
		SwappingMode swappingMode;
		LibMode libMode;

		/// Frequency mode
		FreqMode freqMode;

		/// Frequency to use in constant frequency mode
		double constFreq;

		/// Maximum camera frequency
		static constexpr double MAX_FREQ = 30.0f;

		const char *trackerConfig;

		/// Further, scene specific parameters such as voxel size
		ITMSceneParams sceneParams;
		ITMSurfelSceneParams surfelSceneParams;

		ITMLibSettings(void);
		virtual ~ITMLibSettings(void) {}

		// Suppress the default copy constructor and assignment operator
		ITMLibSettings(const ITMLibSettings&);
		ITMLibSettings& operator=(const ITMLibSettings&);

		MemoryDeviceType GetMemoryType() const;
	};
}
