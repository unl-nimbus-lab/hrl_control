// This is the header file for the coordinate conversion to cpp

#ifndef JSON_READER_H
#define JSON_READER_H


#include <math.h>
#include <thread>
#include <string>
#include <iostream>
#include <math.h>
#include "json.hpp"
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Dense>

using json = nlohmann::json;
using namespace Eigen;


struct Sys {
	int num_clusters;
	MatrixXd p;
    MatrixXd T;
    MatrixXd TA;
    MatrixXd TB;
	MatrixXd ZD; 
	MatrixXd KL;
	MatrixXd Ka;
	MatrixXd DKL;
	MatrixXd DTA;
	MatrixXd DTB;
	MatrixXd state_idxs_C;
} ;
 
struct dSys {
	MatrixXd DTA;
	MatrixXd DTB;
	MatrixXd KD;
};
 

class json_reader{

	public:

		Sys readJSON(std::string path_to_json); // This retrns the full info on the json file
		std::vector<dSys> read_coreg_system(std::string path_to_json);
		Sys get_formation(std::string path_to_json); // This returns the agent groupings
		//double gps_to_meters(double lat1, double lon1, double lat2, double lon2) ; // This gives the distance beteen two gps coordiantes in meters
		//double meters_to_lat(double base_lat, double base_lon, double d ); // This gives the latitde from a base lat,lon coordinnates and a particualr distance with a bearing of 0 degrees

};



#endif
