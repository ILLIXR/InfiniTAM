#include <map>
#include <fstream>
#include <string>
#include <optional>
#include <math.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>

#include "csv_iterator.hpp"

//TUM format
// timestamp tx ty tz qx qy qz qw

using namespace ILLIXR;

typedef tum_pose_type sensor_types;

static
std::map<ullong, sensor_types>
load_data() {
	const char* illixr_data_c_str = std::getenv("ILLIXR_DATA");
	if (!illixr_data_c_str) {
		std::cerr << "Please define ILLIXR_DATA" << std::endl;
		abort();
	}
	std::string illixr_data = std::string{illixr_data_c_str};

	std::map<ullong, sensor_types> data;

	std::ifstream gt_file {illixr_data + "/associate.txt"};

	if (!gt_file.good()) {
		std::cerr << "${ILLIXR_DATA}/groundtruth.txt (" << illixr_data <<  "/groundtruth.txt) is not a good path" << std::endl;
		abort();
	}

	for(CSVIterator row{gt_file, 1}; row != CSVIterator{}; ++row) {
	    std::string resize = row[0].c_str();
	    auto decimal_start = resize.find(".");
//        printf("reached here resize %s\n", row[0].c_str());
	    if(decimal_start != std::string::npos)
	    {
	        resize.erase(decimal_start,1);
	    }
//        printf("reached here resize %s\n", resize.c_str());
        
		ullong t = std::stoull(resize);
//		printf("timestamp :%llu\n", t);
		Eigen::Vector3f av {std::stof(row[1]), std::stof(row[2]), std::stof(row[3])};
		Eigen::Quaternionf la {std::stof(row[4]), std::stof(row[5]), std::stof(row[6]), std::stof(row[7])};
//        printf("depth timestamp %s, depth path %s\n", row[8].c_str(), row[9].c_str());
		std::string depth_timestamp = row[8].c_str();
		std::string depth_path = row[9].c_str();
		data[t] = {{}, av, la, depth_timestamp, depth_path};
	}

	return data;
}
