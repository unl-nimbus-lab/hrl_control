/* This implements the functions of the json_reader for the swarm project.
*/

#include "custom_libs/json_reader.hpp"

Sys json_reader::readJSON(std::string path_to_json)
{
	// This function  reads the .json file located through the path_to_json and outputs structure with the following elements :
	//	1. A sys martix
	//	2. B sys matrix 


	std::ifstream i(path_to_json);
	json j;
	i >> j;

	//Read the json arrays as C arrays
	int num_clusters = j["num_clusters"]; //
	auto p_json = j["p"];
	auto T_json = j["T"];
	auto TA_json = j["TA"];
	auto TB_json = j["TB"];
	auto ZD_json = j["ZD"];
	auto KL_json = j["KL"];
	auto Ka_json = j["Ka"];
	auto DKL_json = j["DKL"];
	auto DTA_json = j["DTA"];
	auto DTB_json = j["DTB"];
	auto state_idxs_C_json = j["state_idxs_C"];

	//Get the dimensions of each of the json arrays
	int p_rows = p_json.size();
	int p_cols = p_json[0].size();
	int T_rows = T_json.size();
	int T_cols = T_json[0].size();
	int TA_rows = TA_json.size();
	int TA_cols = TA_json[0].size();
	int TB_rows = TB_json.size();
	int TB_cols = TB_json[0].size();
	int ZD_rows = ZD_json.size();
	int ZD_cols = ZD_json[0].size();
	int KL_rows = KL_json.size();
	int KL_cols = KL_json[0].size();
	int Ka_rows = Ka_json.size();
	int Ka_cols = Ka_json[0].size();
	int DKL_rows = DKL_json.size();
	int DKL_cols = DKL_json[0].size();
	int DTA_rows = DTA_json.size();
	int DTA_cols = DTA_json[0].size();
	int DTB_rows = DTB_json.size();
	int DTB_cols = DTB_json[0].size();
	int state_idxs_C_rows = state_idxs_C_json.size();
	int state_idxs_C_cols = state_idxs_C_json[0].size();

	//Declare the system matrcies
	MatrixXd p(p_rows,p_cols);
	MatrixXd T(T_rows,T_cols);
	MatrixXd TA(TA_rows,TA_cols);
	MatrixXd TB(TB_rows,TB_cols);
	MatrixXd ZD(ZD_rows,ZD_cols);
	MatrixXd KL(KL_rows,KL_cols);
	MatrixXd Ka(Ka_rows,Ka_cols);
	MatrixXd DKL(DKL_rows,DKL_cols);
	MatrixXd DTA(DTA_rows,DTA_cols);
	MatrixXd DTB(DTB_rows,DTB_cols);
	MatrixXd state_idxs_C(state_idxs_C_rows,state_idxs_C_cols);

	// assign the p matrix from the json file
	for (auto& el : p_json.items())
	{
			int i = 0 ;
			for (auto& subel : el.value()){
					p(stoi(el.key()),i) = subel ;
					i++ ;
			}
	}
	std::cout << "p Matrix" << std::endl;
	std::cout << "======" << std::endl;
	std::cout << p << std::endl;

	// assign the T matrix from the json file
	for (auto& el : T_json.items())
	{
			int i = 0 ;
			for (auto& subel : el.value()){
					T(stoi(el.key()),i) = subel ;
					i++ ;
			}
	}
	std::cout << "T Matrix" << std::endl;
	std::cout << "======" << std::endl;
	std::cout << T << std::endl;

	// assign the TA matrix from the json file
	for (auto& el : TA_json.items())
	{
			int i = 0 ;
			for (auto& subel : el.value()){
					TA(stoi(el.key()),i) = subel ;
					i++ ;
			}
	}
	std::cout << "TA Matrix" << std::endl;
	std::cout << "======" << std::endl;
	std::cout << TA << std::endl;

	// assign the TB matrix from the json file
	for (auto& el : TB_json.items())
	{
			int i = 0 ;
			for (auto& subel : el.value()){
					TB(stoi(el.key()),i) = subel ;
					i++ ;
			}
	}
	std::cout << "TB Matrix" << std::endl;
	std::cout << "======" << std::endl;
	std::cout << TB << std::endl;

	// assign the ZD matrix from the json file
	for (auto& el : ZD_json.items())
	{
			int i = 0 ;
			for (auto& subel : el.value()){
					ZD(stoi(el.key()),i) = subel ;
					i++ ;
			}
	}
	std::cout << "ZD Matrix" << std::endl;
	std::cout << "======" << std::endl;
	std::cout << ZD << std::endl;

	// assign the KL matrix from the json file
	for (auto& el : KL_json.items())
	{
			int i = 0 ;
			for (auto& subel : el.value()){
					KL(stoi(el.key()),i) = subel ;
					i++ ;
			}
	}
	std::cout << "KL Matrix" << std::endl;
	std::cout << "======" << std::endl;
	std::cout << KL << std::endl;

	// assign the Ka matrix from the json file
	for (auto& el : Ka_json.items())
	{
			int i = 0 ;
			for (auto& subel : el.value()){
					Ka(stoi(el.key()),i) = subel ;
					i++ ;
			}
	}
	std::cout << "Ka Matrix" << std::endl;
	std::cout << "======" << std::endl;
	std::cout << Ka << std::endl;

	// assign the DKL matrix from the json file
	for (auto& el : DKL_json.items())
	{
			int i = 0 ;
			for (auto& subel : el.value()){
					DKL(stoi(el.key()),i) = subel ;
					i++ ;
			}
	}
	std::cout << "DKL Matrix" << std::endl;
	std::cout << "======" << std::endl;
	std::cout << DKL << std::endl;

	// assign the DTA matrix from the json file
	for (auto& el : DTA_json.items())
	{
			int i = 0 ;
			for (auto& subel : el.value()){
					DTA(stoi(el.key()),i) = subel ;
					i++ ;
			}
	}
	std::cout << "DTA Matrix" << std::endl;
	std::cout << "======" << std::endl;
	std::cout << DTA << std::endl;

	// assign the TA matrix from the json file
	for (auto& el : DTB_json.items())
	{
			int i = 0 ;
			for (auto& subel : el.value()){
					DTB(stoi(el.key()),i) = subel ;
					i++ ;
			}
	}
	std::cout << "DTB Matrix" << std::endl;
	std::cout << "======" << std::endl;
	std::cout << DTB << std::endl;

	// assign the c indicies matrix from the json file
	for (auto& el : state_idxs_C_json.items())
	{
			int i = 0 ;
			for (auto& subel : el.value()){
					state_idxs_C(stoi(el.key()),i) = subel ;
					i++ ;
			}
	}
	std::cout << "state_idxs_C Matrix" << std::endl;
	std::cout << "======" << std::endl;
	std::cout << state_idxs_C << std::endl;


	struct Sys s = {num_clusters,p,T,TA,TB,ZD,KL,Ka,DKL,DTA,DTB,state_idxs_C} ;

	return s ;
}

std::vector<dSys> read_coreg_system(std::string path_to_json) {
//Read the json file containing the gain schedule with system comonents DTA, DTB, and KD
//Starting at 1 hz, outputs a vector of structures that are accessed like:
// gain_schedule[frequencey-1].DTA etc. 
    
    std::ifstream i(path_to_json);

	//Start some file error checking
    if (!i.is_open()) {
        std::cerr << "Error opening JSON file." << std::endl;
    }

	nlohmann::json jsonData;
	i >> jsonData;

    if (!jsonData.is_array()) {
        std::cerr << "JSON data is not an array." << std::endl;
    }

    size_t numberOfGains = jsonData.size();
    //std::cout << "Number of items in the JSON file: " << numberOfGains << std::endl;

	//Vector to be returned
    std::vector<dSys> gain_schedule(numberOfGains);

	//Iterate through the json items and assign the structures in to the gain schedule
    for (int i = 0; i < numberOfGains; i++) {
        auto TA_json = jsonData[i]["TA"];
        auto TB_json = jsonData[i]["TB"];
        auto K_json = jsonData[i]["K"];

        size_t TA_rows = TA_json.size();
        size_t TA_cols = TA_json[0].size();

        size_t TB_rows = TB_json.size();
        size_t TB_cols = TB_json[0].size();

        size_t K_rows = K_json.size();
        size_t K_cols = K_json[0].size();

        Eigen::MatrixXd TA(TA_rows,TA_cols);
        Eigen::MatrixXd TB(TB_rows,TB_cols);
        Eigen::MatrixXd K(K_rows,K_cols);


        for (auto& el : TA_json.items())
        {
                int i = 0 ;
                for (auto& subel : el.value()){
                        TA(stoi(el.key()),i) = subel ;
                        i++ ;
                }
        }
        // std::cout << "TA Matrix" << std::endl;
        // std::cout << "======" << std::endl;
        // std::cout << TA << std::endl;

        for (auto& el : TB_json.items())
        {
                int i = 0 ;
                for (auto& subel : el.value()){
                        TB(stoi(el.key()),i) = subel ;
                        i++ ;
                }
        }
        // std::cout << "TB Matrix" << std::endl;
        // std::cout << "======" << std::endl;
        // std::cout << TB << std::endl;

        for (auto& el : K_json.items())
        {
                int i = 0 ;
                for (auto& subel : el.value()){
                        K(stoi(el.key()),i) = subel ;
                        i++ ;
                }
        }
        // std::cout << "K Matrix" << std::endl;
        // std::cout << "======" << std::endl;
        // std::cout << K << std::endl;

        gain_schedule[i].DTA.resize(TA_rows,TA_cols);
        gain_schedule[i].DTB.resize(TB_rows,TB_cols);
        gain_schedule[i].KD.resize(K_rows,K_cols);

        gain_schedule[i].DTA = TA;
        gain_schedule[i].DTB = TB;
        gain_schedule[i].KD = K;

    }

    return gain_schedule;

}

Sys get_formation(std::string path_to_json){


        struct Sys s = {} ;

        return s ;

}
