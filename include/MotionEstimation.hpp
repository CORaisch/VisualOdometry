#ifndef _MOTIONESTIMATION_HPP
#define _MOTIONESTIMATION_HPP

#include "Logging.hpp"
#include "util.hpp"
#include "ORBextractor.h"
#include "Frame.hpp"
#include <string>
#include <thread>
#include <opencv2/opencv.hpp>


using namespace cv;

class MotionEstimation
{
public:
    /* constructors */
    MotionEstimation() {};

    /* destructor */
    ~MotionEstimation()
    {
        delete orb_custom_l;
        delete orb_custom_r;
    }

    /* initializers */
    void initMotionEstimator(Logging& _log, const std::string& _path_calib, const bool bDbg, const int iDataset);

    /* public sparse methodes naive */
    void estimateMotionSparse_naive(const Mat& _img_l, const Mat& _img_r);

    /* public sparse methodes */
    void estimateMotionSparse(const Mat& _img_l, const Mat& _img_r);

    /* public dense methodes */
    void estimateMotionDense(const Mat& _img_l, const Mat& _img_r);

    /* visualizes camera poses via 2D projection (bird view) */
    void visualize_birdview();

    /* visualizes camera poses via 2D projection (bird view). Also show ground truth. */
    void visualize_birdview_gt(unsigned int nIter);

    // easy visualization image
    Mat img_vis;
    // list of all estimated poses
    std::vector<Frame> cameraFrames;
    // rectified camera intrinsics
    double fx, fy, cx, cy, bf;

private:
    /* private sparse methodes naive */
    void extractStereoCorrORB_naive(const Mat& _in_img_l, const Mat& _in_img_r, std::vector<Point2f>& _points_l, std::vector<Point2f>& _points_r, std::vector<size_t>& _indices_l, std::vector<size_t>& _indices_r);
    void reconstruct3DFromStereoORB_naive(const std::vector<Point2f>& _in_points_l, const std::vector<Point2f>& _in_points_r, std::vector<Point3f>& _out_points3d);

    /* private dense methodes */
    void reconstruct3DFromDisparityMap(const Mat& _in_img_l, const Mat& _in_img_r, std::vector<Point3f>& _out_points3D, std::vector<Point2f>& _out_points2D);

    /* private member variables */
    Logging Log;
    Ptr<ORB> orb_cv;
    ORBextractor *orb_custom_l, *orb_custom_r;
    Ptr<StereoBM> stereoMatcher;
    BFMatcher matcher;
    Mat Prect00, Prect01, S00, S01, Srect00, Srect01, T01, Rrect00, Rrect01, F, K;

    // constants
    const unsigned int N_ORB_PTS = 1000;

    // flags
    bool isInitialized, isDebugMode;

    // stuff for motion estimation
    Frame frame_t0, frame_t1;

    /* private member methodes */
    void readCalibFromFile_KITTI(const std::string& _path_calib);
    void readCalibFromFile_EuRoC(const std::string& _path_calib);
    void drawEpipolarLines(const Mat& _F, const Mat& _img1, const Mat& _img2, const std::vector<Point2f> _points1, const std::vector<Point2f> _points2);
    void visualizeBackprojection(const std::vector<Point3f>& corr3D, const std::vector<Point2f>& corr2D, const std::vector<int>& inliers);
    void visualizeOverlap(const std::vector<Point2f>& corr2D, const std::vector<Point2f>& corr3D, const std::vector<int>& inliers);

};
#endif //_MOTIONESTIMATION_HPP
