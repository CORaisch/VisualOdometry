#ifndef _FRAME_HPP
#define _FRAME_HPP

#include "Logging.hpp"
#include "MapPoint.hpp"
#include "ORBextractor.h"
#include <string>
#include <opencv2/opencv.hpp>

using namespace cv;

class Frame
{
public:
    Frame();

    Frame(Logging& _log, const Mat& _img_l, const Mat& _img_r, const double _fx, const double _fy, const double _cx,\
          const double _cy, const double _bfx, const Mat& _F, ORBextractor* _orb_l, ORBextractor* _orb_r,\
          const bool bDebugMode);

    // Frame(const Frame& f);

    ~Frame() {}

    /* assigns estimated pose to frame */
    void setPose(const Mat& pose);
    void setPose(const Mat& rvec, const Mat& tvec);

    /* transforms 3D points from camera to worldspace */
    void transform3D();

    /* members for feature extraction */
    ORBextractor *orb_extractor_l, *orb_extractor_r;

    /* point data */
    Mat descriptors_l, descriptors_r;
    std::vector<KeyPoint> keypoints_l, keypoints_r;
    std::vector<float> depth;
    std::vector<DMatch> matches;
    // NOTE triangulate(points2D_l[i], points2D_r[i]) = points3D[i]
    std::vector<MapPoint> points3D;

    /* left, right images */
    Mat img_l, img_r;

    /* pose data */
    Mat cTw, cRw, ctw;
    Mat wTc, wRc, wtc;

    /* rectified camera intrinsics */
    double fx, fy, cx, cy, bfx;

    /* fundamental matrix */
    Mat F;

    /* others */
    Logging Log;

private:
    /* methodes for feature extaction */
    void extractORB(int flag);
    void extractStereoCorrespondences(const bool bDebugMode);

    /* methode for feature matching by considering epipolar constraint */
    void matchByEpiConstraint();

    /* methode for reconstruction of 3D points */
    void triangulate3D();

    /* draw methodes mainly for debugging */
    void drawEpipolarLines();
};

#endif
