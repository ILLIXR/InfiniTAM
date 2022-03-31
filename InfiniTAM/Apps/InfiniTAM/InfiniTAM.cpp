// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>

#include "UIEngine.h"

#include "../../InputSource/OpenNIEngine.h"
#include "../../InputSource/Kinect2Engine.h"
#include "../../InputSource/LibUVCEngine.h"
#include "../../InputSource/PicoFlexxEngine.h"
#include "../../InputSource/RealSenseEngine.h"
#include "../../InputSource/LibUVCEngine.h"
#include "../../InputSource/RealSense2Engine.h"
#include "../../InputSource/FFMPEGReader.h"
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Core/ITMBasicEngine.h"
#include "../../ITMLib/Core/ITMBasicSurfelEngine.h"
#include "../../ITMLib/Core/ITMMultiEngine.h"

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

void list2vec(std::string filename, std::vector<std::string> &time_list, std::vector<std::string> &color_image, std::vector<std::string> &depth_image)
{
		int frameNo = 0;
		double timestamp = 0.0;

        std::ifstream file;
        file.open(filename);
        std::string line;

        std::vector<std::string> output;

		// get the path to the input list file
        std::string path = filename.substr(0, filename.find_last_of("/")+1);

        while(file.good() && (getline(file, line)))
        {
#ifdef DEBUG
                std::cout << "[DEBUG] line: " << line << std::endl;
#endif
                output.clear();
                if (line.find("#") != std::string::npos)
                        continue;

                std::istringstream iss(line);
                std::string str;
                while (iss >> str)
                {
#ifdef DEBUG
                        std::cout << "[DEBUG] str: " << str << std::endl;
#endif
                        output.push_back(str);
                }
				frameNo++;
				timestamp = std::stod(output[0]);
				time_list.push_back(output[0]);

#ifdef DEBUG
				std::cout << "[DEBUG] frameNo: " << frameNo << std::endl;
				std::cout << "[DEBUG] timestamp: " << timestamp << std::endl;
#endif

				/*** 0207 partial test - lab-simple3 ***/

				// if (timestamp < 5815.00 || timestamp > 5825.00) // 5815 - 25
				// if (timestamp < 5800.00 || timestamp > 5810.00) // 5800 - 10

				/*** 0216 ICP on all ***/

				/* lab simple1 */
				// if (timestamp < 5643.577500690)
				// if (timestamp < 5645.179388000)
				// if (timestamp < 5650.179388000 || frameNo > 913)

				/* lab simple2 */
				// if (timestamp < 5724.120164271)
				// if (timestamp < 5725.992547000)
				// if (timestamp < 5743.992547000)

				/* lab simple3 */
				// if (timestamp < 5798.100389485)
				// if (timestamp < 5800.002870000 || frameNo > 1334)

				/* corridor4 */
				// if (frameNo > 400)

				/* hall1 */
				// if (frameNo > 450)

				/* lab-bumper2 */
				/* lab-bumper5 */

				/* lab-dynamic5 */
				// if (frameNo > 750)

				/* lab-light4 */
				// if (frameNo > 800)

				/* lab-manual3 */
				// if (frameNo > 3450)

				/* lab-motion3 */
				// if (frameNo > 700)

				/* office-kt0 */
				// if (frameNo > 900)

				/* living-kt0 */
				// if (frameNo > 1000)

					// continue;


				// This follows the particular list format specified

				/***************************************************
				// fully associated: ts, gt, ts/path for depth, ts/path for color
				***************************************************/
                // color_image.push_back(path + output[11]);
                // depth_image.push_back(path + output[9]);

				/***************************************************
				// rgb-d associated: ts/path for depth, ts/path for color
				***************************************************/
                // color_image.push_back(path + output[3]);
                // depth_image.push_back(path + output[1]);
				// handle the case that the file inside of 'associated'
                color_image.push_back(path + "../" + output[3]);
                depth_image.push_back(path + "../" + output[1]);
        }
        std::cout << "Filename: " << filename+"../"+color_image[1] << std::endl;
        std::cout << "Path: " << path << std::endl;
}

/** Create a default source of depth images from a list of command line
    arguments. Typically, @para arg1 would identify the calibration file to
    use, @para arg2 the colour images, @para arg3 the depth images and
    @para arg4 the IMU images. If images are omitted, some live sources will
    be tried.
*/
static void CreateDefaultImageSource(ImageSourceEngine* & imageSource, IMUSourceEngine* & imuSource, const char *arg1, const char *arg2, const char *arg3, const char *arg4)
{
	const char *calibFile = arg1;
	const char *filename1 = arg2;
	const char *filename2 = arg3;
	const char *filename_imu = arg4;

	if (strcmp(calibFile, "viewer") == 0)
	{
		imageSource = new BlankImageGenerator("", Vector2i(640, 480));
		printf("starting in viewer mode: make sure to press n first to initiliase the views ... \n");
		return;
	}

	printf("using calibration file: %s\n", calibFile);

	if ((imageSource == NULL) && (filename1 != NULL) && (filename2 == NULL))
	{
		std::cout << "Using ImageListPathGenerator! " << std::endl;

		std::vector<std::string> time_list;
		std::vector<std::string> color_list;
		std::vector<std::string> depth_list;

		list2vec(filename1, time_list, color_list, depth_list);

		std::cout << "List size:" << time_list.size() << " " << color_list.size() << " " << depth_list.size() << std::endl;
		ImageListPathGenerator pathGenerator(time_list, color_list, depth_list);
		imageSource = new ImageFileReader<ImageListPathGenerator>(calibFile, pathGenerator);
	}

	if ((imageSource == NULL) && (filename2 != NULL))
	{
		printf("using rgb images: %s\nusing depth images: %s\n", filename1, filename2);
		if (filename_imu == NULL)
		{
			std::cout << "Using ImageMaskPathGenerator! " << std::endl;

			ImageMaskPathGenerator pathGenerator(filename1, filename2);
			imageSource = new ImageFileReader<ImageMaskPathGenerator>(calibFile, pathGenerator);
		}
		else
		{
			printf("using imu data: %s\n", filename_imu);
			imageSource = new RawFileReader(calibFile, filename1, filename2, Vector2i(320, 240), 0.5f);
			imuSource = new IMUSourceEngine(filename_imu);
		}

		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			if (imuSource != NULL) delete imuSource;
			imuSource = NULL;
			imageSource = NULL;
		}
	}

	if ((imageSource == NULL) && (filename1 != NULL) && (filename_imu == NULL))
	{
		imageSource = new InputSource::FFMPEGReader(calibFile, filename1, filename2);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

	if (imageSource == NULL)
	{
		// If no calibration file specified, use the factory default calibration
		bool useInternalCalibration = !calibFile || strlen(calibFile) == 0;

		printf("trying OpenNI device: %s - calibration: %s\n",
				filename1 ? filename1 : "<OpenNI default device>",
				useInternalCalibration ? "internal" : "from file");
		imageSource = new OpenNIEngine(calibFile, filename1, useInternalCalibration);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

	if (imageSource == NULL)
	{
		printf("trying UVC device\n");
		imageSource = new LibUVCEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

	if (imageSource == NULL)
	{
		printf("trying RealSense device\n");
		imageSource = new RealSenseEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

    if (imageSource == NULL)
    {
        printf("trying RealSense device with SDK 2.X (librealsense2)\n");
        imageSource = new RealSense2Engine(calibFile);
        if (imageSource->getDepthImageSize().x == 0)
        {
            delete imageSource;
            imageSource = NULL;
        }
    }

    if (imageSource == NULL)
	{
		printf("trying MS Kinect 2 device\n");
		imageSource = new Kinect2Engine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

	if (imageSource == NULL)
	{
		printf("trying PMD PicoFlexx device\n");
		imageSource = new PicoFlexxEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}
}

int main(int argc, char** argv)
try
{
	const char *arg1 = "";
	const char *arg2 = NULL;
	const char *arg3 = NULL;
	const char *arg4 = NULL;

	int arg = 1;
	do {
		if (argv[arg] != NULL) arg1 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) arg2 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) arg3 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) arg4 = argv[arg]; else break;
	} while (false);

	if (arg == 1) {
		printf("usage: %s [<calibfile> [<imagesource>] ]\n"
		       "  <calibfile>   : path to a file containing intrinsic calibration parameters\n"
		       "  <imagesource> : either one argument to specify OpenNI device ID\n"
		       "                  or two arguments specifying rgb and depth file masks\n"
		       "\n"
		       "examples:\n"
		       "  %s ./Files/Teddy/calib.txt ./Files/Teddy/color/%%04i.ppm ./Files/Teddy/depth/%%04i.pgm\n"
		       "  %s ./Files/Teddy/calib.txt ./Files/Teddy/associated_rgbd.txt\n\n", argv[0], argv[0], argv[0]);
	}

	printf("initialising ...\n");
	ImageSourceEngine *imageSource = NULL;
	IMUSourceEngine *imuSource = NULL;

	CreateDefaultImageSource(imageSource, imuSource, arg1, arg2, arg3, arg4);
	if (imageSource==NULL)
	{
		std::cout << "failed to open any image stream" << std::endl;
		return -1;
	}

	ITMLibSettings *internalSettings = new ITMLibSettings();

	ITMMainEngine *mainEngine = NULL;
	switch (internalSettings->libMode)
	{
	case ITMLibSettings::LIBMODE_BASIC:
		mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, imageSource->getCalib(), imageSource->getRGBImageSize(), imageSource->getDepthImageSize());
		break;
	case ITMLibSettings::LIBMODE_BASIC_SURFELS:
		mainEngine = new ITMBasicSurfelEngine<ITMSurfelT>(internalSettings, imageSource->getCalib(), imageSource->getRGBImageSize(), imageSource->getDepthImageSize());
		break;
	case ITMLibSettings::LIBMODE_LOOPCLOSURE:
		mainEngine = new ITMMultiEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, imageSource->getCalib(), imageSource->getRGBImageSize(), imageSource->getDepthImageSize());
		break;
	default: 
		throw std::runtime_error("Unsupported library mode!");
		break;
	}

	UIEngine::Instance()->Initialise(argc, argv, imageSource, imuSource, mainEngine, "./Files/Out", internalSettings->deviceType);
	UIEngine::Instance()->Run();
	UIEngine::Instance()->Shutdown();

	delete mainEngine;
	delete internalSettings;
	delete imageSource;
	if (imuSource != NULL) delete imuSource;
	return 0;
}
catch(std::exception& e)
{
	std::cerr << e.what() << '\n';
	return EXIT_FAILURE;
}

