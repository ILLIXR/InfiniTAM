// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <cstdlib>
#include <iostream>

#include "CLIEngine.h"

#include "../../InputSource/OpenNIEngine.h"
#include "../../InputSource/Kinect2Engine.h"

#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Core/ITMBasicEngine.h"

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

std::vector<std::string> list2vec(std::string filename)
{
        std::ifstream file;
        file.open(filename);
        std::string line;
        std::vector<std::string> output;

        while(file.good() && (getline(file, line)))
        {
                std::cout << "line: " << line << std::endl;
                output.push_back(line);
        }
        return output;
}


int main(int argc, char** argv)
try
{
	const char *calibFile = "";
	const char *groundtruth = NULL;
	const char *imagesource_part1 = NULL;
	const char *imagesource_part2 = NULL;
	const char *imagesource_part3 = NULL;

	int arg = 1;
	do {
		if (argv[arg] != NULL) calibFile = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) groundtruth = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) imagesource_part1 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) imagesource_part2 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) imagesource_part3 = argv[arg]; else break;
	} while (false);

	if (arg == 1) {
		printf("usage: %s [<calibfile> [<imagesource>] ]\n"
		       "  <calibfile>   : path to a file containing intrinsic calibration parameters\n"
		       "  <imagesource> : either one argument to specify OpenNI device ID\n"
		       "                  or two arguments specifying rgb and depth file masks\n"
		       "\n"
		       "examples:\n"
		       "  %s ./Files/Teddy/calib.txt ./Files/Teddy/Frames/%%04i.ppm ./Files/Teddy/Frames/%%04i.pgm\n"
		       "  %s ./Files/Teddy/calib.txt\n\n", argv[0], argv[0], argv[0]);
	}

	printf("initialising ...\n");
	ITMLibSettings *internalSettings = new ITMLibSettings();

	ImageSourceEngine *imageSource;
	IMUSourceEngine *imuSource = NULL;
	printf("using calibration file: %s\n", calibFile);
	if (imagesource_part2 == NULL) 
	{
		printf("using OpenNI device: %s\n", (imagesource_part1==NULL)?"<OpenNI default device>":imagesource_part1);
		imageSource = new OpenNIEngine(calibFile, imagesource_part1);
		if (imageSource->getDepthImageSize().x == 0) {
			delete imageSource;
			printf("trying MS Kinect device\n");
			imageSource = new Kinect2Engine(calibFile);
		}
	} 
	else
	{
		if (imagesource_part3 == NULL)
		{
			printf("using rgb images: %s\nusing depth images: %s\n", imagesource_part1, imagesource_part2);

			bool useMask = false;

			if (useMask)
			{
				std::cout << "Using ImageMaskPathGenerator! " << std::endl;
				ImageMaskPathGenerator pathGenerator(imagesource_part1, imagesource_part2);
				imageSource = new ImageFileReader<ImageMaskPathGenerator>(calibFile, pathGenerator);
			}
			else
			{
				std::cout << "Using ImageListPathGenerator! " << std::endl;

				std::vector<std::string> color_list = list2vec(imagesource_part1);
				std::vector<std::string> depth_list = list2vec(imagesource_part2);

				std::cout << "Loaded list size:" << color_list.size() << " " << depth_list.size() << std::endl;
				ImageListPathGenerator pathGenerator(color_list, depth_list);
				imageSource = new ImageFileReader<ImageListPathGenerator>(calibFile, pathGenerator);
			}

		}
		else
		{
			printf("using rgb images: %s\nusing depth images: %s\nusing imu data: %s\n", imagesource_part1, imagesource_part2, imagesource_part3);
			imageSource = new RawFileReader(calibFile, imagesource_part1, imagesource_part2, Vector2i(320, 240), 0.5f);
			imuSource = new IMUSourceEngine(imagesource_part3);
		}
	}

	ITMMainEngine *mainEngine = new ITMBasicEngine<ITMVoxel,ITMVoxelIndex>(
		internalSettings, imageSource->getCalib(), imageSource->getRGBImageSize(), imageSource->getDepthImageSize(), groundtruth
	);

	CLIEngine::Instance()->Initialise(imageSource, imuSource, mainEngine, internalSettings->deviceType);
	CLIEngine::Instance()->Run();
	CLIEngine::Instance()->Shutdown();

	delete mainEngine;
	delete internalSettings;
	delete imageSource;
	return 0;
}
catch(std::exception& e)
{
	std::cerr << e.what() << '\n';
	return EXIT_FAILURE;
}
