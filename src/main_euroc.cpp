#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "Logging.hpp"
#include "MotionEstimation.hpp"
#include "TrajectoryViz.hpp"
#include <thread>

using namespace std;
using namespace cv;

// dataset parameters
std::string base_path, path_calib, path_timestamps, path_imgs_grey_l, path_imgs_grey_r;

// keyhandling parameters
bool f_pause, f_verbose;

// other parameters
Logging Log;
enum { ESTIMATE_DENSE, ESTIMATE_SPARSE_NAIVE, ESTIMATE_SPARSE };
int estimator_mode;

// function prototypes
bool initAndParseCmdArgs(int argc, char **argv);
void printUsage();
void show_all_frames(Mat& _img_grey_l, Mat& _img_grey_r);
void LoadImages(const string &_str_path_left, const string &_str_path_right, const string &_str_path_times,\
                vector<string> &_vstr_image_left, vector<string> &_vstr_image_right, vector<double> &_vd_timestamps);

int main(int argc, char **argv)
{
    // local variables
    Mat img_grey_l, img_grey_r, img_grey_rect_l, img_grey_rect_r;
    double t_start, t_end, t_est_motion, t_viz_3d, t_viz_bv;
    char key;

    // initialize according to command line arguments
    if(!initAndParseCmdArgs(argc, argv))
        return -1;

    // load images
    std::vector<string> vstr_image_l, vstr_image_r;
    std::vector<double> vd_timestamps;
    LoadImages(path_imgs_grey_l, path_imgs_grey_r, path_timestamps, vstr_image_l, vstr_image_r, vd_timestamps);

    if(vstr_image_l.empty() || vstr_image_r.empty())
    {
        cerr << "ERROR: No images in provided path." << endl;
        return 1;
    }

    if(vstr_image_l.size()!=vstr_image_r.size())
    {
        cerr << "ERROR: Different number of left and right images." << endl;
        return 1;
    }

    // undistort and rectify images -> code taken from ORB-SLAM2 project
    cv::FileStorage fs_settings(path_calib, cv::FileStorage::READ);
    if(!fs_settings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fs_settings["LEFT.K"] >> K_l;
    fs_settings["RIGHT.K"] >> K_r;
    fs_settings["LEFT.P"] >> P_l;
    fs_settings["RIGHT.P"] >> P_r;
    fs_settings["LEFT.R"] >> R_l;
    fs_settings["RIGHT.R"] >> R_r;
    fs_settings["LEFT.D"] >> D_l;
    fs_settings["RIGHT.D"] >> D_r;
    int rows_l = fs_settings["LEFT.height"];
    int cols_l = fs_settings["LEFT.width"];
    int rows_r = fs_settings["RIGHT.height"];
    int cols_r = fs_settings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty()\
       || rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    // init motion estimation
    MotionEstimation motion_estimator;
    motion_estimator.initMotionEstimator(Log, path_calib, f_verbose, 1); // 1 == EuRoC

    // init 3D trajectory visualization
    TrajectoryVisualizer trajectoryViz;
    trajectoryViz.Init(motion_estimator.fx, motion_estimator.fy, motion_estimator.cx, motion_estimator.cy);

    // invoke threaded 3D visualization
    std::thread* pThreadViz = new thread(&TrajectoryVisualizer::Run, &trajectoryViz);

    // main processing loop, process as long as images are available
    Log("\n", "\033[0m\033[1;47;30m", "run main loop", "\033[0m", "\n\n");
    unsigned int nCount = 0;
    const int n_imgs = vstr_image_l.size();
    for(int i = 0; i < n_imgs; ++i)
    {
        // start clocking processing time
        t_start = (double) getTickCount();

        /******************************************************************************/
        /* Start Image Processing Code here                                           */
        /******************************************************************************/

        // load frames
        img_grey_l = cv::imread(vstr_image_l[i], CV_LOAD_IMAGE_UNCHANGED);
        img_grey_r = cv::imread(vstr_image_r[i], CV_LOAD_IMAGE_UNCHANGED);

        if(img_grey_l.empty())
        {
            cerr << endl << "Failed to load image at: " << string(vstr_image_l[i]) << endl;
            return 1;
        }

        if(img_grey_r.empty())
        {
            cerr << endl << "Failed to load image at: " << string(vstr_image_r[i]) << endl;
            return 1;
        }

        cv::remap(img_grey_l,img_grey_rect_l,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(img_grey_r,img_grey_rect_r,M1l,M2l,cv::INTER_LINEAR);

        // check for estimator mode
        if(estimator_mode == ESTIMATE_SPARSE_NAIVE)
            motion_estimator.estimateMotionSparse_naive(img_grey_rect_l, img_grey_rect_r);
        else if(estimator_mode == ESTIMATE_SPARSE)
        {
            // convert images to CV_8UC1
            double t_tmp = (double) getTickCount(); // start clocking total time for motion estimation
            motion_estimator.estimateMotionSparse(img_grey_rect_l, img_grey_rect_r);
            t_est_motion = (double(getTickCount()) - t_tmp)/getTickFrequency(); // end clocking total time for motion estimation

            // update data for 3D trajectory visualization
            t_tmp = (double) getTickCount(); // start clocking total time for 3D visualization
            trajectoryViz.UpdateTrajectory(motion_estimator.cameraFrames);
            t_viz_3d = (double(getTickCount()) - t_tmp)/getTickFrequency(); // end clocking total time for 3D visualization

            // visualize simple trajectory 2D from birdview
            t_tmp = (double) getTickCount(); // start clocking total time for bird view visualization
            motion_estimator.visualize_birdview();
            // motion_estimator.visualize_birdview_gt(nCount++);
            t_viz_bv = (double(getTickCount()) - t_tmp)/getTickFrequency(); // end clocking total time for bird view visualization
        }
        else if(estimator_mode == ESTIMATE_DENSE)
            motion_estimator.estimateMotionDense(img_grey_rect_l, img_grey_rect_r);
        // show_all_frames(img_grey_rect_l, img_grey_rect_r);

        /******************************************************************************/
        /* End Image Processing Code here                                             */
        /******************************************************************************/

        // stop clocking processing time
        t_end = (double(getTickCount()) - t_start)/getTickFrequency();
        Log("", "\e[34m", "total time for motion estimation: " + std::to_string(t_est_motion) + "s | fps: "+ std::to_string(1.0/t_est_motion), "\e[0m", "\n");
        Log("", "\e[34m", "total time for 3D visualization: " + std::to_string(t_viz_3d) + "s | fps: "+ std::to_string(1.0/t_viz_3d), "\e[0m", "\n");
        Log("", "\e[34m", "total time for bird view visualization: " + std::to_string(t_viz_bv) + "s | fps: "+ std::to_string(1.0/t_viz_bv), "\e[0m", "\n");
        Log("", "\e[34m", "total processing time: " + std::to_string(t_end) + "s | fps: "+ std::to_string(1.0/t_end), "\e[0m", "\n");
        if(!f_verbose)
            std::cout << "\033[s\033[34mprocessing time: " << t_end << "s | fps: " << std::to_string(1.0/t_end) << "\033[0m\033[u" << std::flush;

        // key handling
        key = waitKey(f_pause ? 0 : 1);
        if(key == 27)
        {
            // stop on ESC
            break;
        }
        else if(key == 32)
        {
            // pause on SPACE (equals stepwise mode)
            f_pause = !f_pause;
        }
    }

    // print resulted image
    // double cpuTime = double(cv::getTickCount())/cv::getTickFrequency();
    // std::string result_path = base_path+"result_"+std::to_string(cpuTime)+".png";
    // cv::imwrite(result_path, motion_estimator.img_vis);

    // exit programme on keyboard event
    Log("\n", "\033[0m\033[1;47;30m", "Reached End of Dataset successfully", "\033[0m", "\n");
    Log("Press any key to exit...");
    waitKey(0);

    // shutdown 3D visualization
    trajectoryViz.Shutdown();

    return 0;
}

void show_all_frames(Mat& _img_grey_l, Mat& _img_grey_r)
{
    imshow("Right Grayscale", _img_grey_r);
    imshow("Left Grayscale",  _img_grey_l);
}

bool initAndParseCmdArgs(int argc, char **argv)
{
    // set default flags
    bool f_logging = false;
    f_pause   = true;
    f_verbose = false;
    estimator_mode = ESTIMATE_SPARSE;

    // parse commandline arguments
    for (int i = 1; i < argc; ++i)
    {
        // print usage on demand
        if(!std::strcmp(argv[i], "-h") || !std::strcmp(argv[i], "--help"))
        {
            printUsage();
            return false;
        }
        // check for dataset
        if(!std::strcmp(argv[i], "-i") || !std::strcmp(argv[i], "--input"))
        {
            i++;
            if(i < argc)
            {
                if(!std::strcmp(argv[i], "0"))
                {
                    // dataset base paths for "drive_0047_sync"
                    base_path       = "/home/claudio/Datasets/EuRoC/MH01/mav0/";
                    path_calib      = "/home/claudio/Datasets/EuRoC/Calib.yaml";
                    path_timestamps = "/home/claudio/Datasets/EuRoC/EuRoC_TimeStamps/MH01.txt";
                }
            }
        }
        // check for estimator mode
        if(!std::strcmp(argv[i], "-m") || !std::strcmp(argv[i], "--mode"))
        {
            i++;
            if(i < argc)
            {
                if(!std::strcmp(argv[i], "0") || !std::strcmp(argv[i], "sparse_naive"))
                    estimator_mode = ESTIMATE_SPARSE_NAIVE;
                else if(!std::strcmp(argv[i], "1") || !std::strcmp(argv[i], "sparse"))
                    estimator_mode = ESTIMATE_SPARSE;
                else if(!std::strcmp(argv[i], "2") || !std::strcmp(argv[i], "dense"))
                    estimator_mode = ESTIMATE_DENSE;
            }
        }
        // check for verbose flag
        if(!std::strcmp(argv[i], "-v") || !std::strcmp(argv[i], "--verbose"))
        {
            f_verbose = true;
        }
        // check for logging flag
        if (!std::strcmp(argv[i], "-l") || !std::strcmp(argv[i], "--logging"))
        {
            f_logging = true;
        }
    }

    // set relative paths
    path_imgs_grey_l = base_path + "cam0/data";
    path_imgs_grey_r = base_path + "cam1/data";

    // init logging
    Log = Logging(f_verbose, f_logging, "../logs/");

    return true;
}

void printUsage()
{
    cout << "Usage: visodom [OPTION]...\n"
         << "By default visodom starts running on the KIIT_drive_0047_sync dataset.\n"
         << "\nOptions:\n"
         << "-h --help                                print usage\n"
         << "-i --input [BASE IMAGES] [BASE CALIB]    set basepath to KITTI dataset (*)\n"
         << "-v --verbose                             print all debug messages\n"
         << "-l --logging                             write output to logfile in log directory"
         << "-m --mode [0|1|2]                        switch estimator mode: 0 = sparse_naive, 1 = sparse, 2 = dense\n"
         << "\n(*) for the quicker use 3 standard KITTI datasets are predefined.\n"
         << "To use start visodom with \"-i {0-2}\" option."
         << endl;
}

// load EuRoC dataset -> code taken from ORB-SLAM2 project
void LoadImages(const string &_str_path_left, const string &_str_path_right, const string &_str_path_times,\
                vector<string> &_vstr_image_left, vector<string> &_vstr_image_right, vector<double> &_vd_timestamps)
{
    ifstream f_times;
    f_times.open(_str_path_times.c_str());
    _vd_timestamps.reserve(5000);
    _vstr_image_left.reserve(5000);
    _vstr_image_right.reserve(5000);
    while(!f_times.eof())
    {
        string s;
        getline(f_times,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            _vstr_image_left.push_back(_str_path_left + "/" + ss.str() + ".png");
            _vstr_image_right.push_back(_str_path_right + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            _vd_timestamps.push_back(t/1e9);
        }
    }
}
