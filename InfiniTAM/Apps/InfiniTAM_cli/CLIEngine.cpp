// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "CLIEngine.h"

#include <fstream>
#include <iostream>
#include <string.h>

#include "../../ORUtils/FileUtils.h"

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

CLIEngine* CLIEngine::instance;

void CLIEngine::Initialise(ImageSourceEngine *imageSource, IMUSourceEngine *imuSource, ITMMainEngine *mainEngine,
	ITMLibSettings::DeviceType deviceType)
{
	this->imageSource = imageSource;
	this->imuSource = imuSource;
	this->mainEngine = mainEngine;

	this->currentFrameNo = 0;

	bool allocateGPU = false;
	if (deviceType == ITMLibSettings::DEVICE_CUDA) allocateGPU = true;

	inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, allocateGPU);
	inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, allocateGPU);
	inputIMUMeasurement = new ITMIMUMeasurement();

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaThreadSynchronize());
#endif

	sdkCreateTimer(&timer_instant);
	sdkCreateTimer(&timer_average);

	sdkResetTimer(&timer_average);

	freqControl.freqDivisor = mainEngine->GetFreqDivisor();
	freqControl.framesSinceFreqChange = 0;
	freqControl.processed = new std::vector<unsigned>;
	freqControl.frequencies = new std::vector<unsigned>;
	freqControl.newBricks = new std::vector<unsigned>;

	printf("initialised.\n");
}


bool CLIEngine::ProcessFrame()
{
	std::cout << "\n============================ Begin Frame =============================\n";

	// Frequency control
	freqControl.frequencies->push_back(ITMLibSettings::MAX_FREQ / freqControl.freqDivisor);
	bool shouldSkip = (freqControl.framesSinceFreqChange % freqControl.freqDivisor) != 0;
	if (shouldSkip)
	{
		std::cout << "Skipping frame " << currentFrameNo << "\n";
		imageSource->skipImage();
		mainEngine->SkipFrame();
		freqControl.processed->push_back(0);
		freqControl.newBricks->push_back(0);
	}
	else
	{
		std::cout << "Running frame " << currentFrameNo << "\n";

		if (!imageSource->hasMoreImages()) return false;
		imageSource->getImages(inputRGBImage, inputRawDepthImage);

		if (imuSource != NULL) {
			if (!imuSource->hasMoreMeasurements()) return false;
			else imuSource->getMeasurement(inputIMUMeasurement);
		}

		sdkResetTimer(&timer_instant);
		sdkStartTimer(&timer_instant); sdkStartTimer(&timer_average);

		//actual processing on the mainEngine
		mainEngine->currentTimeStamp = imageSource->currentTimeStamp;
		if (imuSource != NULL) mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);
		else mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);

#ifndef COMPILE_WITHOUT_CUDA
		ORcudaSafeCall(cudaThreadSynchronize());
#endif
		sdkStopTimer(&timer_instant); sdkStopTimer(&timer_average);

		float processedTime_inst = sdkGetTimerValue(&timer_instant);
		float processedTime_avg = sdkGetAverageTimerValue(&timer_average);

		printf("frame %i: time %.2f, avg %.2f\n", currentFrameNo, processedTime_inst, processedTime_avg);

		freqControl.processed->push_back(1);
		freqControl.newBricks->push_back(mainEngine->GetNumNewBricks());
		unsigned newDivisor = mainEngine->GetFreqDivisor();
		if (newDivisor != freqControl.freqDivisor)
		{
			freqControl.freqDivisor = newDivisor;
			freqControl.framesSinceFreqChange = -1;
		}
	}

	currentFrameNo++;
	freqControl.framesSinceFreqChange++;
	std::cout << "============================= End Frame ==============================\n";
	return true;
}

void CLIEngine::Run()
{
	while (true) {
		if (!ProcessFrame()) break;
	}
}

void CLIEngine::Shutdown()
{
	sdkDeleteTimer(&timer_instant);
	sdkDeleteTimer(&timer_average);

	delete inputRGBImage;
	delete inputRawDepthImage;
	delete inputIMUMeasurement;

	delete instance;

	std::ofstream frameFile("execution_data.csv");
	frameFile << "#Frame,New Bricks,Frequency\n";
	for (unsigned idx = 0; idx < freqControl.processed->size(); idx++)
		frameFile << idx << "," << freqControl.newBricks->at(idx) << "," << freqControl.frequencies->at(idx) << "\n";
	frameFile.close();

	delete freqControl.processed;
	delete freqControl.frequencies;
	delete freqControl.newBricks;
}
