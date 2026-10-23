/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"
#include <filesystem>
#include <stdexcept>

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;
int USE_SUPERPOINT;
std::string SUPERPOINT_BACKEND;
std::string SUPERPOINT_ENGINE_PATH;
double SUPERPOINT_SCORE_THRESHOLD;
int SUPERPOINT_MAX_KEYPOINTS;
int SUPERPOINT_BORDER;
int FEATURE_DIAGNOSTICS;
std::string FEATURE_DIAGNOSTICS_PATH;


template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];

    USE_SUPERPOINT = fsSettings["use_superpoint"].empty() ? 0 : static_cast<int>(fsSettings["use_superpoint"]);
    SUPERPOINT_BACKEND = "tensorrt";
    if (!fsSettings["superpoint_backend"].empty())
        fsSettings["superpoint_backend"] >> SUPERPOINT_BACKEND;
    SUPERPOINT_ENGINE_PATH.clear();
    if (!fsSettings["superpoint_engine_path"].empty())
        fsSettings["superpoint_engine_path"] >> SUPERPOINT_ENGINE_PATH;
    ros::param::get("~superpoint_engine_path", SUPERPOINT_ENGINE_PATH);
    SUPERPOINT_SCORE_THRESHOLD = fsSettings["superpoint_score_threshold"].empty() ? 0.02 : static_cast<double>(fsSettings["superpoint_score_threshold"]);
    SUPERPOINT_MAX_KEYPOINTS = fsSettings["superpoint_max_keypoints"].empty() ? 1024 : static_cast<int>(fsSettings["superpoint_max_keypoints"]);
    SUPERPOINT_BORDER = fsSettings["superpoint_border"].empty() ? 10 : static_cast<int>(fsSettings["superpoint_border"]);
    FEATURE_DIAGNOSTICS = fsSettings["feature_diagnostics"].empty() ? 0 : static_cast<int>(fsSettings["feature_diagnostics"]);
    FEATURE_DIAGNOSTICS_PATH.clear();
    if (!fsSettings["feature_diagnostics_path"].empty())
        fsSettings["feature_diagnostics_path"] >> FEATURE_DIAGNOSTICS_PATH;

    // 正式运行 SuperPoint 时缺少 engine 应直接报错，不能静默换检测器污染对照。 (Ref: 2026-10-23_Init_RDSwarm_Framework.md)
    if (USE_SUPERPOINT && SUPERPOINT_ENGINE_PATH.empty())
        throw std::runtime_error("SuperPoint engine is required; set superpoint_engine_path launch argument");
    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    USE_IMU = fsSettings["imu"];
    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    ros::param::get("~output_path", OUTPUT_FOLDER);
    if (OUTPUT_FOLDER.empty()) throw std::runtime_error("output_path is empty");
    std::filesystem::create_directories(OUTPUT_FOLDER);
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    } 
    
    NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }


    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);

    if (!SUPERPOINT_ENGINE_PATH.empty() && SUPERPOINT_ENGINE_PATH[0] != '/')
        SUPERPOINT_ENGINE_PATH = configPath + "/" + SUPERPOINT_ENGINE_PATH;
    if (!FEATURE_DIAGNOSTICS_PATH.empty() && FEATURE_DIAGNOSTICS_PATH[0] != '/')
        FEATURE_DIAGNOSTICS_PATH = configPath + "/" + FEATURE_DIAGNOSTICS_PATH;
    if (FEATURE_DIAGNOSTICS && FEATURE_DIAGNOSTICS_PATH.empty())
        FEATURE_DIAGNOSTICS_PATH = OUTPUT_FOLDER + "/feature_tracker_diagnostics.csv";
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        //printf("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);
        
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }

    fsSettings.release();
}
