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
std::string base_path, base_calib, path_imgs_grey_l, path_imgs_grey_r, \
    path_imgs_rgb_l, path_imgs_rgb_r;

// keyhandling parameters
bool f_pause, f_verbose;

// other parameters
Logging Log;
enum { ESTIMATE_DENSE, ESTIMATE_SPARSE_NAIVE, ESTIMATE_SPARSE };
int estimator_mode;

// function prototypes
bool initAndParseCmdArgs(int argc, char **argv);
void printUsage();
void show_all_frames(Mat& _img_grey_l, Mat& _img_grey_r, Mat& _img_rgb_l, Mat& _img_rgb_r);

int main(int argc, char **argv)
{
    // local variables
    VideoCapture cap_grey_l, cap_grey_r, cap_rgb_l, cap_rgb_r;
    Mat img_grey_l, img_grey_r, img_rgb_l, img_rgb_r;
    double t_start, t_end, t_est_motion, t_viz_3d, t_viz_bv;
    char key;

    // initialize according to command line arguments
    if(!initAndParseCmdArgs(argc, argv))
        return -1;

    // read from dataset (binocular) // NOTE USE THIS FOR RAW DATA
    cap_grey_l = VideoCapture(path_imgs_grey_l + "%10d.png");
    cap_grey_r = VideoCapture(path_imgs_grey_r + "%10d.png");
    cap_rgb_l  = VideoCapture(path_imgs_rgb_l  + "%10d.png");
    cap_rgb_r  = VideoCapture(path_imgs_rgb_r  + "%10d.png");
    // NOTE USE THIS FOR KITTI BENCHMARK
    // cap_grey_l = VideoCapture(path_imgs_grey_l + "%6d.png");
    // cap_grey_r = VideoCapture(path_imgs_grey_r + "%6d.png");
    // cap_rgb_l  = VideoCapture(path_imgs_rgb_l  + "%6d.png");
    // cap_rgb_r  = VideoCapture(path_imgs_rgb_r  + "%6d.png");

    if(!cap_grey_l.isOpened())
        return -1;

    // grab initial frame(s)
    cap_grey_l >> img_grey_l;
    cap_grey_r >> img_grey_r;
    cap_rgb_l  >> img_rgb_l;
    cap_rgb_r  >> img_rgb_r;

    // init motion estimation
    MotionEstimation motion_estimator;
    motion_estimator.initMotionEstimator(Log, base_calib, f_verbose, 0); // 0 == KITTI

    // init 3D trajectory visualization
    TrajectoryVisualizer trajectoryViz;
    trajectoryViz.Init(motion_estimator.fx, motion_estimator.fy, motion_estimator.cx, motion_estimator.cy);

    // invoke threaded 3D visualization
    std::thread* pThreadViz = new thread(&TrajectoryVisualizer::Run, &trajectoryViz);

    // main processing loop, process as long as images are available
    Log("\n", "\033[0m\033[1;47;30m", "run main loop", "\033[0m", "\n\n");
    unsigned int nCount = 0;
    while(!img_grey_l.empty())
    {
        // start clocking processing time
        t_start = (double) getTickCount();

        /******************************************************************************/
        /* Start Image Processing Code here                                           */
        /******************************************************************************/

        // check for estimator mode
        if(estimator_mode == ESTIMATE_SPARSE_NAIVE)
            motion_estimator.estimateMotionSparse_naive(img_grey_l, img_grey_r);
        else if(estimator_mode == ESTIMATE_SPARSE)
        {
            // convert images to CV_8UC1
            cvtColor(img_grey_l, img_grey_l, COLOR_RGB2GRAY);
            cvtColor(img_grey_r, img_grey_r, COLOR_RGB2GRAY);
            double t_tmp = (double) getTickCount(); // start clocking total time for motion estimation
            motion_estimator.estimateMotionSparse(img_grey_l, img_grey_r);
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
            motion_estimator.estimateMotionDense(img_grey_l, img_grey_r);

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

        // grab next frames
        cap_grey_l >> img_grey_l;
        cap_grey_r >> img_grey_r;
        cap_rgb_l  >> img_rgb_l;
        cap_rgb_r  >> img_rgb_r;

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

void show_all_frames(Mat& _img_grey_l, Mat& _img_grey_r, Mat& _img_rgb_l, Mat& _img_rgb_r)
{
    imshow("Right Grayscale", _img_grey_r);
    imshow("Left Grayscale",  _img_grey_l);
    imshow("Right RGB", _img_rgb_r);
    imshow("Left RGB",  _img_rgb_l);
}

bool initAndParseCmdArgs(int argc, char **argv)
{
    // set default flags
    bool f_logging = false;
    f_pause   = true;
    f_verbose = false;
    estimator_mode = ESTIMATE_SPARSE;
    // set default dataset ("drive_0047_sync")
    base_path  = "/home/claudio/Datasets/KITTI/RawData_2011_10_03_drive_0047_sync/2011_10_03_drive_0047_sync/";
    base_calib = "/home/claudio/Datasets/KITTI/RawData_2011_10_03_drive_0047_sync/2011_10_03_calib/";

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
                    base_path  = "/home/claudio/Datasets/KITTI/RawData_2011_10_03_drive_0047_sync/2011_10_03_drive_0047_sync/";
                    base_calib = "/home/claudio/Datasets/KITTI/RawData_2011_10_03_drive_0047_sync/2011_10_03_calib/";
                }
                else if(!std::strcmp(argv[i], "1"))
                {
                    // dataset base paths for "drive_0071_sync"
                    base_path  = "/home/claudio/Datasets/KITTI/RawData_2011_09_29_drive_0071_sync/2011_09_29_drive_0071_sync/";
                    base_calib = "/home/claudio/Datasets/KITTI/RawData_2011_09_29_drive_0071_sync/2011_09_29_calib/";
                }
                else if(!std::strcmp(argv[i], "2"))
                {
                    // dataset base paths for "drive_0034_sync"
                    base_path  = "/home/claudio/Datasets/KITTI/RawData_2011_09_30_drive_0034_sync/2011_09_30_drive_0034_sync/";
                    base_calib = "/home/claudio/Datasets/KITTI/RawData_2011_09_30_drive_0034_sync/2011_09_30_calib/";
                }
                else
                {
                    base_path  = argv[i++];
                    base_calib = argv[i];
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

    // set relative paths (which are consistent among KITTI datasets) /// NOTE USE THIS FOR RAW DATA
    path_imgs_grey_l = base_path + "image_00/data/";
    path_imgs_grey_r = base_path + "image_01/data/";
    path_imgs_rgb_l  = base_path + "image_02/data/";
    path_imgs_rgb_r  = base_path + "image_03/data/";

    // NOTE USE THIS FOR KITTI BENCHMARK
    // path_imgs_grey_l = base_path + "image_0/";
    // path_imgs_grey_r = base_path + "image_1/";
    // path_imgs_rgb_l  = base_path + "image_0/";
    // path_imgs_rgb_r  = base_path + "image_1/";

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
