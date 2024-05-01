#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<iostream>
#include <string>
#include <numeric>
//#include<io.h>
#include<cstdlib>
#include <getopt.h>
#include "Eva.h"
#include <unistd.h>
using namespace std;
string folderPath;
bool add_overlap;
bool low_inlieratio;
bool no_logs;

string threeDMatch[8] = {
	"7-scenes-redkitchen",
	"sun3d-home_at-home_at_scan1_2013_jan_1",
	"sun3d-home_md-home_md_scan9_2012_sep_30",
	"sun3d-hotel_uc-scan3",
	"sun3d-hotel_umd-maryland_hotel1",
	"sun3d-hotel_umd-maryland_hotel3",
	"sun3d-mit_76_studyroom-76-1studyroom2",
	"sun3d-mit_lab_hj-lab_hj_tea_nov_2_2012_scan1_erika",
};

string threeDlomatch[8] = {
	"7-scenes-redkitchen_3dlomatch",
	"sun3d-home_at-home_at_scan1_2013_jan_1_3dlomatch",
	"sun3d-home_md-home_md_scan9_2012_sep_30_3dlomatch",
	"sun3d-hotel_uc-scan3_3dlomatch",
	"sun3d-hotel_umd-maryland_hotel1_3dlomatch",
	"sun3d-hotel_umd-maryland_hotel3_3dlomatch",
	"sun3d-mit_76_studyroom-76-1studyroom2_3dlomatch",
	"sun3d-mit_lab_hj-lab_hj_tea_nov_2_2012_scan1_erika_3dlomatch",
};

string ETH[4] = {
	"gazebo_summer",
	"gazebo_winter",
	"wood_autmn",
	"wood_summer",
};

double RE, TE, success_estimate_rate, RMSE;
vector<int>scene_num;
vector<string> analyse(const string& name, const string& result_scene, const string& dataset_scene, const string& descriptor, ofstream& outfile, int iters, int data_index) {
	if (!no_logs && access(result_scene.c_str(), 0))
	{
		if (mkdir(result_scene.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)!=0) {
			cout << " Create scene folder failed! " << endl;
			exit(-1);
		}
	}
	vector<string>error_pair;

	string error_txt;
	//error_txt = result_scene + "/error_pair.txt";

	if (descriptor == "fpfh" || descriptor == "spinnet" || descriptor == "d3feat")
	{
		error_txt = dataset_scene + "/dataload.txt";
	}
	else if (descriptor == "fcgf")
	{
		error_txt = dataset_scene + "/dataload_fcgf.txt";
	}
	if (access(error_txt.c_str(), 0))
	{
		cout << " Could not find dataloader file! " << endl;
		exit(-1);
	}

	ifstream f1(error_txt);
	string line;
	while (getline(f1, line))
	{
		error_pair.push_back(line);
	}
	f1.close();
	scene_num.push_back(error_pair.size());
	vector<string>match_success_pair;
	int index = 1;
	RE = 0;
	TE = 0;
	RMSE = 0; 
	success_estimate_rate = 0;
	vector<double>time;
	for (const auto& pair : error_pair)
	{
		time.clear();
		cout << "Pair " << index << ", " << "total " << error_pair.size() << " pairs." << endl;
		index++;
		string result_folder = result_scene + "/" + pair;
		string::size_type i = pair.find("+") + 1;
		string src_filename = dataset_scene + "/" + pair.substr(0, i - 1) + ".ply";
		string des_filename = dataset_scene + "/" + pair.substr(i, pair.length() - i) + ".ply";
		//cout << src_filename << " " << des_filename << endl;
		string corr_path = dataset_scene + "/" + pair + (descriptor == "fcgf" ? "@corr_fcgf.txt" : "@corr.txt");
		string gt_label = dataset_scene + "/" + pair + (descriptor == "fcgf" ? "@label_fcgf.txt" : "@label.txt");
		string gt_mat_path = dataset_scene + "/" + pair + (descriptor == "fcgf" ? "@GTmat_fcgf.txt" : "@GTmat.txt");

		string ov_label = "NULL";
		double re, te, inlier_num, total_num, inlier_ratio, success_estimate, total_estimate, rmse;
		int corrected = registration(name, src_filename, des_filename, corr_path, gt_label, ov_label, gt_mat_path, result_folder, re, te, inlier_num, total_num, inlier_ratio, success_estimate, total_estimate, descriptor, time, rmse);
		int iter = iters;
		double est_rr = success_estimate / (total_estimate / 1.0);
		success_estimate_rate += est_rr;
		if (corrected)
		{
			cout << pair << " Success." << endl;
			RE += re;
			TE += te;
			RMSE += rmse;
			match_success_pair.push_back(pair);
		}
		else
		{
			cout << pair << " Fail." << endl;
		}
		outfile << pair << ',' << corrected << ',' << inlier_num << ',' << total_num << ',';
		outfile << setprecision(4) << inlier_ratio << ',' << est_rr << ',' << re << ',' << te << ',' << time[0] << ',' << time[1] << ',' << time[2] << ',' << time[3] << ',' << success_estimate << ',' << rmse << endl;
		cout << endl;
	}
	error_pair.clear();

	return match_success_pair;
}

void demo(){
    PointCloudPtr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudPtr des_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudPtr new_src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudPtr new_des_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    string src_path = "demo/src.pcd";
    string des_path = "demo/des.pcd";
    pcl::io::loadPCDFile(src_path, *src_cloud);
    pcl::io::loadPCDFile(des_path, *des_cloud);
    float src_resolution = MeshResolution_mr_compute(src_cloud);
    float des_resolution = MeshResolution_mr_compute(des_cloud);
    float resolution = (src_resolution + des_resolution) / 2;

    float downsample = 5 * resolution;
    Voxel_grid_downsample(src_cloud, new_src_cloud, downsample);
    Voxel_grid_downsample(des_cloud, new_des_cloud, downsample);
    vector<vector<float>> src_feature, des_feature;
    FPFH_descriptor(new_src_cloud, downsample*5, src_feature);
    FPFH_descriptor(new_des_cloud, downsample*5, des_feature);

    vector<Corre_3DMatch>correspondence;
    feature_matching(new_src_cloud, new_des_cloud, src_feature, des_feature, correspondence);

    vector<double>ov_lable;
    ov_lable.resize((int)correspondence.size());
    
    folderPath = "demo/result";
    cout << "Start registration." << endl;
    registration(src_cloud, des_cloud, correspondence, ov_lable, folderPath, resolution,0.99);
    //clear data
    src_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    des_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    new_src_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    new_des_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    src_feature.clear();
    src_feature.shrink_to_fit();
    des_feature.clear();
    des_feature.shrink_to_fit();
    correspondence.clear();
    correspondence.shrink_to_fit();
    ov_lable.clear();
    ov_lable.shrink_to_fit();
    exit(0);
}

void usage(){
    cout << "Usage:" << endl;
    cout << "\tHELP --help" <<endl;
    cout << "\tDEMO --demo" << endl;
    cout << "\tREQUIRED ARGS:" << endl;
    cout << "\t\t--output_path\toutput path for saving results." << endl;
    cout << "\t\t--input_path\tinput data path." << endl;
    cout << "\t\t--dataset_name\tdataset name. [3dmatch/3dlomatch/KITTI/ETH/U3M]" << endl;
    cout << "\t\t--descriptor\tdescriptor name. [fpfh/fcgf/spinnet/predator]" << endl;
    cout << "\t\t--start_index\tstart from given index. (begin from 0)" << endl;
    cout << "\tOPTIONAL ARGS:" << endl;
    cout << "\t\t--no_logs\tforbid generation of log files." << endl;
};

int main(int argc, char** argv) {
    //////////////////////////////////////////////////////////////////
    add_overlap = false;
    low_inlieratio = false;
    no_logs = false;
    int id = 0;
    string resultPath; 
    string datasetPath; 
    string datasetName; 
    string descriptor; 
    //////////////////////////////////////////////////////////////////
    int opt;
    int digit_opind = 0;
    int option_index = 0;
    static struct option long_options[] = {
            {"output_path", required_argument, NULL, 'o'},
            {"input_path", required_argument, NULL, 'i'},
            {"dataset_name", required_argument, NULL, 'n'},
            {"descriptor", required_argument, NULL, 'd'},
            {"start_index", required_argument, NULL, 's'},
            {"no_logs", optional_argument, NULL, 'g'},
            {"help", optional_argument, NULL, 'h'},
            {"demo", optional_argument, NULL, 'm'},
            {NULL, 0, 0, '\0'}
    };

    while((opt = getopt_long(argc, argv, "", long_options, &option_index)) != -1){
        switch (opt) {
            case 'h':
                usage();
                exit(0);
            case 'o':
                resultPath = optarg;
                break;
            case 'i':
                datasetPath = optarg;
                break;
            case 'n':
                datasetName = optarg;
                break;
            case 'd':
                descriptor = optarg;
                break;
            case 'g':
                no_logs = true;
                break;
            case 's':
                id = atoi(optarg);
                break;
            case 'm':
                demo();
                exit(0);
            case '?':
                printf("Unknown option: %c\n",(char)optopt);
                usage();
                exit(-1);
        }
    }
    if(argc  < 11){
        cout << 11 - argc <<" more args are required." << endl;
        usage();
        exit(-1);
    }

    cout << "Check your args setting:" << endl;
    cout << "\toutput_path: " << resultPath << endl;
    cout << "\tinput_path: " << datasetPath << endl;
    cout << "\tdataset_name: " << datasetName << endl;
    cout << "\tdescriptor: " << descriptor << endl;
    cout << "\tstart_index: " << id << endl;
    cout << "\tno_logs: " << no_logs << endl;

    sleep(5);

	int corrected = 0;
	int total_num = 0;
	double total_re = 0;
	double total_te = 0;
	double total_rmse = 0;
	vector<double>total_success_est_rate;
	vector<int> scene_correct_num;
	vector<double>scene_re_sum;
	vector<double>scene_te_sum;
	vector<double>scene_rmse_sum;
	if (access(resultPath.c_str(), 0))
	{
		if (mkdir(resultPath.c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
			cout << " Create save folder failed! " << endl;
			exit(-1);
		}
	}

	if (descriptor == "predator" && (datasetName == "3dmatch" || datasetName == "3dlomatch")) {
		vector<string>pairs;
		string loader = datasetPath + "/dataload.txt";
        cout << loader << endl;
		ifstream f1(loader);
		string line;
		while (getline(f1, line))
		{
			pairs.push_back(line);
		}
		f1.close();

		string analyse_csv = resultPath + "/" + datasetName + "_" + descriptor + ".csv";
		ofstream outFile;
		outFile.open(analyse_csv.c_str(), ios::out);
		outFile.setf(ios::fixed, ios::floatfield);
		outFile << "pair_name" << ',' << "corrected_or_no" << ',' << "inlier_num" << ',' << "total_num" << ',' << "inlier_ratio" << ',' << "RE" << ',' << "TE" << ',' << "RMSE"<< endl;
		vector<string>fail_pair;
		vector<double>time;
		for (int i = id; i < pairs.size(); i++)
		{
			time.clear();
			cout << "Pair " << i + 1 << "，total" << pairs.size()/*name_list.size()*/ << "，fail " << fail_pair.size() << endl;
			string filename = pairs[i];
			string corr_path = datasetPath + "/" + filename + "@corr.txt";
			string gt_mat_path = datasetPath + "/" + filename + "@GTmat.txt";
			string gt_label_path = datasetPath + "/" + filename + "@label.txt";
            string ov_label = "NULL";
			string folderPath = resultPath + "/" + filename;
			double re, te, rmse;
			double inlier_num, total_num;
			double inlier_ratio, success_estimate, total_estimate;

			int corrected = registration(datasetName, "NULL", "NULL", corr_path, gt_label_path, ov_label, gt_mat_path, folderPath, re, te, inlier_num, total_num, inlier_ratio, success_estimate, total_estimate, descriptor, time, rmse);
			if (corrected)
			{
				cout << filename << " Success." << endl;
				RE += re;
				TE += te;
				RMSE += rmse;
			}
			else
			{
				fail_pair.push_back(filename);
				cout << filename << " Fail." << endl;
			}
			outFile << filename << ',' << corrected << ',' << inlier_num << ',' << total_num << ',';
			outFile << setprecision(4) << inlier_ratio << ',' << re << ',' << te << ',' << rmse << ','<< time[0] << ',' << time[1] << ',' << time[2] << ',' << time[3] << endl;
			cout << endl;
		}
		outFile.close();
		double success_num = pairs.size() - fail_pair.size();
		cout << "total:" << endl;
		cout << "\tRR:" << pairs.size() - fail_pair.size() << "/" << pairs.size() << " " << success_num / (pairs.size() / 1.0) << endl;
		cout << "\tRE:" << RE / (success_num / 1.0) << endl;
		cout << "\tTE:" << TE / (success_num / 1.0) << endl;
		cout << "\tRMSE:" << RMSE / (success_num / 1.0) << endl;
		cout << "fail pairs:" << endl;
		/*for (size_t i = 0; i < fail_pair.size(); i++)
		{
			cout << "\t" << fail_pair[i] << endl;
		}*/
	}
	else if (datasetName == "3dmatch")
	{
		for (size_t i = id; i < 8; i++)
		{
			cout << i + 1 << ":" << threeDMatch[i] << endl;
			string analyse_csv = resultPath + "/" + threeDMatch[i] + "_" + descriptor + ".csv";
			ofstream outFile;
			outFile.open(analyse_csv.c_str(), ios::out);
			outFile.setf(ios::fixed, ios::floatfield);
			outFile << "pair_name" << ',' << "corrected_or_no" << ',' << "inlier_num" << ',' << "total_num" << ',' << "inlier_ratio" << ',' << "est_rr" << ',' << "RE" << ',' << "TE" << ',' << "RMSE"<< ',' << "construction" << ',' << "search" << ',' << "selection" << ',' << "estimation" << endl;
			vector<string>matched = analyse("3dmatch", resultPath + "/" + threeDMatch[i], datasetPath + "/" + threeDMatch[i], descriptor, outFile, id, i);
			scene_re_sum.push_back(RE);
			scene_te_sum.push_back(TE);
			scene_rmse_sum.push_back(RMSE);
			if (!matched.empty())
			{
				cout << endl;
				cout << threeDMatch[i] << ":" << endl;
				for (auto t : matched)
				{
					cout << "\t" << t << endl;
				}
				cout << endl;
				cout << threeDMatch[i] << ":" << matched.size() / (scene_num[i] / 1.0) << endl;
				cout << "success_est_rate:" << success_estimate_rate / (scene_num[i] / 1.0) << "RE:" << RE / matched.size() << "\tTE:" << TE / matched.size() << "\tRMSE:" << RMSE / matched.size() << endl;
				corrected += matched.size();
				total_success_est_rate.push_back(success_estimate_rate);
				scene_correct_num.push_back(matched.size());
			}
			outFile.close();
			matched.clear();
		}
		string detail_txt = resultPath + "/details.txt";
		ofstream outFile;
		outFile.open(detail_txt.c_str(), ios::out);
		outFile.setf(ios::fixed, ios::floatfield);
		for (size_t i = 0; i < 8; i++)
		{
			total_num += scene_num[i];
			total_re += scene_re_sum[i];
			total_te += scene_te_sum[i];
			total_rmse += scene_rmse_sum[i];
			cout << i + 1 << ":" << endl;
			outFile << i + 1 << ":" << endl;
			cout << "\tRR: " << scene_correct_num[i] << "/" << scene_num[i] << " " << scene_correct_num[i] / (scene_num[i] / 1.0) << endl;
			outFile << "\tRR: " << scene_correct_num[i] << "/" << scene_num[i] << " " << setprecision(4) << scene_correct_num[i] / (scene_num[i] / 1.0) << endl;
			cout << "\tSuccess_est_rate: " << total_success_est_rate[i] / (scene_num[i] / 1.0) << endl;
			cout << "\tRE: " << scene_re_sum[i] / (scene_correct_num[i] / 1.0) << endl;
			outFile << "\tRE: " << setprecision(4) << scene_re_sum[i] / (scene_correct_num[i] / 1.0) << endl;
			cout << "\tTE: " << scene_te_sum[i] / (scene_correct_num[i] / 1.0) << endl;
			outFile << "\tTE: " << setprecision(4) << scene_te_sum[i] / (scene_correct_num[i] / 1.0) << endl;
			cout << "\tRMSE: " << scene_rmse_sum[i] / (scene_correct_num[i] / 1.0) << endl;
			outFile << "\tRMSE: " << setprecision(4) << scene_rmse_sum[i] / (scene_correct_num[i] / 1.0) << endl;
		}
		cout << "total:" << endl;
		outFile << "total:" << endl;
		cout << "\tRR: " << corrected / (total_num / 1.0) << endl;
		outFile << "\tRR: " << setprecision(4) << corrected / (total_num / 1.0) << endl;
		cout << "\tSuccess_est_rate: " << accumulate(total_success_est_rate.begin(), total_success_est_rate.end(), 0.0) / (total_num / 1.0) << endl;
		cout << "\tRE: " << total_re / (corrected / 1.0) << endl;
		outFile << "\tRE: " << setprecision(4) << total_re / (corrected / 1.0) << endl;
		cout << "\tTE: " << total_te / (corrected / 1.0) << endl;
		outFile << "\tTE: " << setprecision(4) << total_te / (corrected / 1.0) << endl;
		cout << "\tRMSE: " << total_rmse / (corrected / 1.0) << endl;
		outFile << "\tRMSE: " << setprecision(4) << total_rmse / (corrected / 1.0) << endl;
		outFile.close();
	}
	else if (datasetName == "3dlomatch")
	{
		for (size_t i = id; i < 8; i++)
		{
			string analyse_csv = resultPath + "/" + threeDlomatch[i] + "_" + descriptor + ".csv";
			ofstream outFile;
			outFile.open(analyse_csv.c_str(), ios::out);
			outFile.setf(ios::fixed, ios::floatfield);
			outFile << "pair_name" << ',' << "corrected_or_no" << ',' << "inlier_num" << ',' << "total_num" << ',' << "inlier_ratio" << ',' << "est_rr" << ',' << "RE" << ',' << "TE" << ',' << "RMSE" << endl;
			vector<string>matched = analyse("3dlomatch", resultPath + "/" + threeDlomatch[i], datasetPath + "/" + threeDlomatch[i], descriptor, outFile, id, i);
			scene_re_sum.push_back(RE);
			scene_te_sum.push_back(TE);
			scene_rmse_sum.push_back(RMSE);
			if (!matched.empty())
			{
				cout << endl;
				cout << threeDlomatch[i] << ":" << endl;
				for (auto t : matched)
				{
					cout << "\t" << t << endl;
				}
				cout << endl;
				cout << threeDlomatch[i] << ":" << matched.size() / (scene_num[i] / 1.0) << endl;
				cout << "RE:" << RE / matched.size() << "\tTE:" << TE / matched.size() << "\tRMSE:" << RMSE / matched.size()<< endl;
				corrected += matched.size();
				total_success_est_rate.push_back(success_estimate_rate);
				scene_correct_num.push_back(matched.size());
			}
			outFile.close();
			matched.clear();
		}
		string detail_txt = resultPath + "/details.txt";
		ofstream outFile;
		outFile.open(detail_txt.c_str(), ios::out);
		outFile.setf(ios::fixed, ios::floatfield);
		for (size_t i = 0; i < 8; i++)
		{
			total_num += scene_num[i];
			total_re += scene_re_sum[i];
			total_te += scene_te_sum[i];
			total_rmse += scene_rmse_sum[i];
			cout << i + 1 << ":" << endl;
			outFile << i + 1 << ":" << endl;
			cout << "\tRR: " << scene_correct_num[i] << "/" << scene_num[i] << " " << scene_correct_num[i] / (scene_num[i] / 1.0) << endl;
			outFile << "\tRR: " << scene_correct_num[i] << "/" << scene_num[i] << " " << setprecision(4) << scene_correct_num[i] / (scene_num[i] / 1.0) << endl;
			cout << "\tSuccess_est_rate: " << total_success_est_rate[i] / (scene_num[i] / 1.0) << endl;
			cout << "\tRE: " << scene_re_sum[i] / (scene_correct_num[i] / 1.0) << endl;
			outFile << "\tRE: " << setprecision(4) << scene_re_sum[i] / (scene_correct_num[i] / 1.0) << endl;
			cout << "\tTE: " << scene_te_sum[i] / (scene_correct_num[i] / 1.0) << endl;
			outFile << "\tTE: " << setprecision(4) << scene_te_sum[i] / (scene_correct_num[i] / 1.0) << endl;
			cout << "\tRMSE: " << scene_rmse_sum[i] / (scene_correct_num[i] / 1.0) << endl;
			outFile << "\tRMSE: " << setprecision(4) << scene_rmse_sum[i] / (scene_correct_num[i] / 1.0) << endl;
		}
		cout << "total:" << endl;
		outFile << "total:" << endl;
		cout << "\tRR: " << corrected / (total_num / 1.0) << endl;
		outFile << "\tRR: " << setprecision(4) << corrected / (total_num / 1.0) << endl;
		cout << "\tSuccess_est_rate: " << accumulate(total_success_est_rate.begin(), total_success_est_rate.end(), 0.0) / (total_num / 1.0) << endl;
		cout << "\tRE: " << total_re / (corrected / 1.0) << endl;
		outFile << "\tRE: " << setprecision(4) << total_re / (corrected / 1.0) << endl;
		cout << "\tTE: " << total_te / (corrected / 1.0) << endl;
		outFile << "\tTE: " << setprecision(4) << total_te / (corrected / 1.0) << endl;
		cout << "\tRMSE: " << total_rmse / (corrected / 1.0) << endl;
		outFile << "\tRMSE: " << setprecision(4) << total_rmse / (corrected / 1.0) << endl;
		outFile.close();
	}
	else if (datasetName == "KITTI")
	{
		int pair_num = 555;
		//string txt_path = datasetPath + "/" + descriptor;
        const string& txt_path = datasetPath;
		string analyse_csv = resultPath + "/KITTI_" + descriptor + ".csv";
		ofstream outFile;
		outFile.open(analyse_csv.c_str(), ios::out);
		outFile.setf(ios::fixed, ios::floatfield);
		outFile << "pair_name" << ',' << "corrected_or_no" << ',' << "inlier_num" << ',' << "total_num" << ',' << "inlier_ratio" << ',' << "RE" << ',' << "TE" << ',' << "RMSE" << endl;
		vector<string>fail_pair;
		vector<double>time;
		for (int i = id; i < pair_num; i++)
		{
			time.clear();
			cout << "Pair " << i + 1 << "，total" << pair_num/*name_list.size()*/ << "，fail " << fail_pair.size() << endl;

            string filename = to_string(i);/*name_list[i]*/;
			string corr_path = txt_path + "/" + filename + '/' + descriptor + "@corr.txt";
			string gt_mat_path = txt_path + "/" + filename + '/' + descriptor + "@gtmat.txt";
			string gt_label_path = txt_path + "/" + filename + '/' + descriptor + "@gtlabel.txt";
            string ov_label = "NULL";
			string folderPath = resultPath + "/" + filename;
			double re, te, rmse;
			double inlier_num, total_num;
			double inlier_ratio, success_estimate, total_estimate;

			int corrected = registration("KITTI", "NULL", "NULL", corr_path, gt_label_path, ov_label, gt_mat_path, folderPath, re, te, inlier_num, total_num, inlier_ratio, success_estimate, total_estimate, descriptor, time, rmse);
			if (corrected)
			{
				cout << filename << " Success." << endl;
				RE += re;
				TE += te;
				RMSE += rmse;
			}
			else
			{
				fail_pair.push_back(filename);
				cout << filename << " Fail." << endl;
			}
			outFile << filename << ',' << corrected << ',' << inlier_num << ',' << total_num << ',';
			outFile << setprecision(4) << inlier_ratio << ',' << re << ',' << te << ',' << rmse << endl;
			cout << endl;
		}
		outFile.close();
		double success_num = pair_num - fail_pair.size();
		cout << "total:" << endl;
		cout << "\tRR:" << pair_num - fail_pair.size() << "/" << pair_num << " " << success_num / (pair_num / 1.0) << endl;
		cout << "\tRE:" << RE / (success_num / 1.0) << endl;
		cout << "\tTE:" << TE / (success_num / 1.0) << endl;
		cout << "\tRMSE:" << RMSE / (success_num / 1.0) << endl;
		cout << "fail pairs:" << endl;
		for (size_t i = 0; i < fail_pair.size(); i++)
		{
			cout << "\t" << fail_pair[i] << endl;
		}
	}
	else {
		exit(0);
	}
    return 0;
}