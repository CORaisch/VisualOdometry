#include "MotionEstimation.hpp"

#include "util.hpp"
#include "ORBextractor.h"
#include "Logging.hpp"

/***************************************************************************************/
/*                               initialization                                        */
/***************************************************************************************/

void MotionEstimation::initMotionEstimator(Logging& _log, const std::string& _path_calib, const bool bDbg, const int iDataset)
{
    // assign logging object
    Log = _log;
    // set flags
    isInitialized = false;
    isDebugMode = bDbg;
    // reserve space for camera poses and set initial pose
    cameraFrames.reserve(100);
    // initialize image for easy visualization
    img_vis = Mat(1000, 1000, CV_8UC3, cv::Scalar(255,255,255));
    // load calibration data from dataset file
    if(iDataset == 0)
    {
        // load KITTI calibration
        readCalibFromFile_KITTI(_path_calib);
    }
    else if(iDataset == 1)
    {
        // load EuRoc calibration
        readCalibFromFile_EuRoC(_path_calib);
    }
    // init ORB feature extractor
    orb_cv = ORB::create(N_ORB_PTS, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31);
    // init custom ORB extractor
    int nFeatures = 1000, nLeves = 8, nIniThFast = 20, nMinThFast = 7;
    float fScaleFactor = 1.2f;
    orb_custom_l = new ORBextractor(nFeatures, fScaleFactor, nLeves, nIniThFast, nMinThFast);
    orb_custom_r = new ORBextractor(nFeatures, fScaleFactor, nLeves, nIniThFast, nMinThFast);
    // init stereo block matcher
    stereoMatcher = StereoBM::create(16*5, 21);
}

/*******************************************************************************/
/*                  sparse motion estimation optimized                         */
/*******************************************************************************/

/* estimateMotionSparse */
/* estimates camera pose via SFM using only stereo images for input */
void MotionEstimation::estimateMotionSparse(const Mat& _img_l, const Mat& _img_r)
{
    /* implements simple structure from motion procedure without mapping over longer baselines */
    if(!isInitialized)
    {
        // create initial frame
        frame_t0 = Frame(Log, _img_l, _img_r, fx, fy, cx, cy, bf, F, orb_custom_l, orb_custom_r, isDebugMode);
        // assign initial pose
        frame_t0.setPose(Mat::eye(4,4,CV_32F));
        // transform 3D points in worldspace
        frame_t0.transform3D();
        // store initial frame
        cameraFrames.push_back(frame_t0);
        // set estimator to be initialized
        isInitialized = true;
    }
    else
    {
        // declare timing variables
        double t_tmp, t_extract_features, t_bf_matching, t_ransac_pnp;
        /* detect ORB features */
        // create frame t_i
        t_tmp = (double) getTickCount(); // start clocking time for feature extraction
        frame_t1 = Frame(Log, _img_l, _img_r, fx, fy, cx, cy, bf, F, orb_custom_l, orb_custom_r, isDebugMode);
        t_extract_features = (double(getTickCount()) - t_tmp)/getTickFrequency(); // stop clocking time for feature extraction

        /* collect 3D-2D-correspondences */
        t_tmp = (double) getTickCount(); // start clocking time for BF matching
        // collect all left image keypoints of frame t0 which have 3D points
        std::vector<MapPoint> assoc3D;
        assoc3D.reserve(frame_t0.keypoints_l.size());
        Mat desc3D;
        for(size_t i = 0; i < frame_t0.points3D.size(); ++i)
        {
            MapPoint pt3D = frame_t0.points3D[i];
            if(pt3D.isValid())
            {
                // these are the valid keypoints which have an 3D point in frame t0
                desc3D.push_back(frame_t0.descriptors_l.row(i));
                assoc3D.push_back(frame_t0.points3D[i]);
            }
        }
        // brute force match collected points from t0 with left image keypoints from frame t1
        std::vector<DMatch> matches;
        matcher.match(desc3D, frame_t1.descriptors_l, matches);
        t_bf_matching = (double(getTickCount()) - t_tmp)/getTickFrequency(); // stop clocking time for BF matching

        /*******************************************************************/
        /* [invariant] 3D-2D correspondences:                              */
        /* 3D point frame t0: assoc3D[matches[i].queryIdx].pt_ws           */
        /* 2D point frame t1: frame_t1.keypoints_l[matches[i].trainIdx].pt */
        /*******************************************************************/

        /* reshape correspondences for passing to opencvs' solvePnP function */
        t_tmp = (double) getTickCount(); // start clocking time for RANSAC PnP
        std::vector<Point3f> correspondences3D;
        std::vector<Point2f> correspondences2D, correspondences3D_in2D;
        correspondences3D.reserve(matches.size());
        correspondences2D.reserve(matches.size());
        for(size_t i = 0; i < matches.size(); ++i)
        {
            correspondences3D.push_back(assoc3D[matches[i].queryIdx].pt_ws);
            correspondences2D.push_back(frame_t1.keypoints_l[matches[i].trainIdx].pt);
            correspondences3D_in2D.push_back(assoc3D[matches[i].queryIdx].key.pt);
        }

        /* estimate new pose for frame t1 from 3D-2D-correspondences */
        // solve PnP in RANSAC scheme to be robust against outlier
        Mat rvec, tvec;
        std::vector<int> inliers;
        solvePnPRansac(correspondences3D, correspondences2D, K, noArray(), rvec, tvec, false, 2000, 8.0, 0.99, inliers, SOLVEPNP_P3P);
        t_ransac_pnp = (double(getTickCount()) - t_tmp)/getTickFrequency(); // stop clocking time for RANSAC PnP

        /* assign new pose to frame t1 */
        frame_t1.setPose(rvec, tvec);

        /* log estimated data */
        Log("\n\n", "\e[1;4m", "estimated data:", "\e[0m", "\n");
        Log("R:\n" + mat2str(frame_t1.cRw) + "\n");
        Log("t:\n" + mat2str(frame_t1.ctw) + "\n");
        Log("" + std::to_string(inliers.size()) + "/" + std::to_string(correspondences3D.size())\
            + " (" + std::to_string((float(inliers.size())/float(correspondences3D.size()))*100)\
            + "%) inliers\n");
        Log("", "\e[34m", "total time for feature extraction: " + std::to_string(t_extract_features) + "s | fps: "+ std::to_string(1.0/t_extract_features), "\e[0m", "\n");
        Log("", "\e[34m", "total time for BF matching: " + std::to_string(t_bf_matching) + "s | fps: "+ std::to_string(1.0/t_bf_matching), "\e[0m", "\n");
        Log("", "\e[34m", "total time for RANSAC PnP: " + std::to_string(t_ransac_pnp) + "s | fps: "+ std::to_string(1.0/t_ransac_pnp), "\e[0m", "\n");

        /* triangulate new 3D points for frame t1 */
        // transform 3D points of frame t1 in worldspace according to new pose
        frame_t1.transform3D();

        /* store all frames for visualization */
        cameraFrames.push_back(frame_t1);

        /* DEBUG visualize backprojection of 3D-2D correspondences */
        if(isDebugMode)
        {
            // visualizeBackprojection(correspondences3D, correspondences2D, inliers);
            // visualizeOverlap(correspondences2D, correspondences3D_in2D, inliers);
        }

        /* set frame t0 to t1 */
        frame_t0 = frame_t1;
    }
}

/* visualize_birdview */
/* visualizes camera poses in a simple manner */
/* by drawing it in the bird view perspective */
void MotionEstimation::visualize_birdview()
{
    // get inverted camera pose
    Mat currentT = cameraFrames.back().wtc;
    // transform it into bird view perspective
    Point2f center(currentT.at<float>(0,0), -currentT.at<float>(2,0));
    // shift the projected point
    center += Point2f(500,500);
    // draw point and show visualization
    circle(img_vis, center, 2, Scalar(0,0,255), -1, LINE_8, 0);
    imshow("pose graph visualization (bird view)", img_vis);
}

/* visualize_birdview_gt */
/* visualizes camera poses in a simple manner  */
/* by drawing it in the bird view perspective  */
/* Also visualize the KITTI ground truth poses */
// FIXME check used pose data and validate transform
void MotionEstimation::visualize_birdview_gt(unsigned int nIter)
{
    // get inverted camera pose
    Mat currentT = cameraFrames.back().wtc;
    // transform it into bird view perspective
    Point2f center(currentT.at<float>(0,0), -currentT.at<float>(2,0));
    // shift the projected point to lower right corner
    center += Point2f(300,550);
    // draw point and show visualization
    circle(img_vis, center, 3, Scalar(0,0,255), -1, LINE_8, 0);

    // load ground truth pose from file
    std::string gt_path = "/home/claudio/Documents/Datasets/KITTI/Ground_Truth/poses/06_converted.txt";
    std::string sel = "C" + std::to_string(nIter);
    std::cout << sel << std::endl;
    FileStorage fs(gt_path, FileStorage::READ);
    Log("", "\033[1;47;30m", "read ground truth data from \"" + gt_path + "\" @ " + sel + "...", "", "");
    cv::Mat C_gt;
    fs[sel] >> C_gt;
    fs.release();
    // get gt translation
    cv::Mat ctw_gt = C_gt.col(3);
    cv::Mat cRw_gt = C_gt.rowRange(0,3).colRange(0,3);
    cv::Mat wtc_gt = -cRw_gt.t()*ctw_gt;
    // transform it into bird view perspective
    Point2f center_gt(wtc_gt.at<float>(0,0), -wtc_gt.at<float>(2,0));
    // shift the projected point to lower right corner
    center_gt += Point2f(300,550);
    // draw point and show visualization
    circle(img_vis, center_gt, 2, Scalar(0,255,0), -1, LINE_8, 0);

    // draw image
    imshow("pose graph visualization (bird view)", img_vis);
}

void MotionEstimation::visualizeOverlap(const std::vector<Point2f>& corr2D,\
                                        const std::vector<Point2f>& corr3D,\
                                        const std::vector<int>& inliers)
{
    /* get consecutive images */
    Mat img_t1, img_t0;
    std::vector<Frame>::iterator iter = cameraFrames.end();
    cvtColor((iter-2)->img_l, img_t0, cv::COLOR_GRAY2BGR);
    cvtColor((iter-1)->img_l, img_t1, cv::COLOR_GRAY2BGR);
    RNG rng(0);

    /* draw correspondences in images t0 and t1 */
    Scalar color;
    unsigned int inlierCount = 0;
    for(size_t i = 0; i < corr3D.size(); ++i)
    {
        // draw only inliers
        if(inliers[inlierCount] == i)
        {
            // draw features
            color = Scalar(rng(256),rng(256),rng(256));
            circle(img_t0, corr3D[i], 5, color, 1, cv::LINE_AA);
            circle(img_t1, corr2D[i], 5, color, 1, cv::LINE_AA);

            // draw line to connect features
            line(img_t0, corr3D[i], corr2D[i], color, 3, cv::LINE_AA);

            inlierCount++;
        }
    }

    /* warp images of last and current frame */
    Mat img_warped;
    addWeighted(img_t0, 0.5, img_t1, 0.5, 0, img_warped);
    imshow("3D-2D correspondences overlayed", img_warped);
}

void MotionEstimation::visualizeBackprojection(const std::vector<Point3f>& corr3D,\
                                               const std::vector<Point2f>& corr2D,\
                                               const std::vector<int>& inliers)
{
    // prepare output image
    Mat img_backproj, img_outliers, img_selected;
    cvtColor(cameraFrames.back().img_l, img_backproj, cv::COLOR_GRAY2BGR);
    img_outliers = Mat::zeros(img_backproj.rows, img_backproj.cols, img_backproj.type());
    // img_outliers = Mat(img_backproj.rows, img_backproj.cols, img_backproj.type());
    // img_outliers = Scalar(255,255,255);
    RNG rng(0);
    Scalar color;
    int radius, circlethickness, linethickness;
    float alpha = 0.2;
    // prepare backprojection
    Mat R, t;
    cameraFrames.back().cRw.copyTo(R);
    cameraFrames.back().ctw.copyTo(t);
    // iterate for each 3D-2D correspondence to draw in image
    unsigned int inlierCount = 0;
    for(size_t i = 0; i < corr2D.size(); ++i)
    {
        // check for inliers
        if(inliers[inlierCount] == i) // match i is inlier
        {
            // roll random color
            color = Scalar(rng(256),rng(256),rng(256));
            radius = 5;
            circlethickness = 1;
            linethickness = 3;
            inlierCount++;
            img_selected = img_backproj;
        }
        else // match i is outlier
        {
            // light gray for outliers
            color = Scalar::all(200);
            radius = 2;
            circlethickness = 1;
            linethickness = 1;
            img_selected = img_outliers;
        }

        /* draw 2D correspondences */
        circle(img_selected, corr2D[i], radius, color, circlethickness, cv::LINE_AA);

        /* draw 3D correspondences */
        // backproject 3D correspondences
        Mat pt_ws(corr3D[i]);
        Mat pt_cs = R*pt_ws + t;
        float tmp_x = (pt_cs.at<float>(0,0)*fx/pt_cs.at<float>(2,0)) + cx;
        float tmp_y = (pt_cs.at<float>(1,0)*fy/pt_cs.at<float>(2,0)) + cy;
        Point2f corr3D_backproj(tmp_x, tmp_y);
        circle(img_selected, corr3D_backproj, radius, color, circlethickness, cv::LINE_AA);

        /* draw line to connect correspondences */
        line(img_selected, corr2D[i], corr3D_backproj, color, linethickness, cv::LINE_AA);
    }

    // // add key in upper right corner of final image
    // String txt0 = "Key:";
    // String txt1 = "2D point & projected 3D correspondence";
    // String txt2 = "outlierd 3D-2D matches";
    // Scalar txtColor(100,100,100);
    // int baseline = 0;
    // int fontface =  FONT_HERSHEY_SIMPLEX;
    // double fontscale = 0.6;
    // int thickness = 1;
    // int currentHeight = 0;
    // Size textsize = getTextSize(txt0, fontface, fontscale, thickness, &baseline);
    // currentHeight += textsize.height;
    // textsize = getTextSize(txt1, fontface, fontscale, thickness, &baseline);
    // putText(img_backproj, txt0, Point2f(0,textsize.height), fontface, fontscale, Scalar(255,255,255), thickness, 8);
    // circle(img_backproj, Point2f(7,10+currentHeight), 5, Scalar(128, 248, 13), 1, cv::LINE_AA);
    // circle(img_backproj, Point2f(12,10+currentHeight), 5, Scalar(129,0,210), 1, cv::LINE_AA);
    // putText(img_backproj, txt1, Point2f(20,currentHeight+textsize.height+2), fontface, fontscale, Scalar(255,255,255), thickness, 8);
    // currentHeight += textsize.height;
    // textsize = getTextSize(txt2, fontface, fontscale, thickness, &baseline);
    // line(img_outliers, Point2f(5,10+currentHeight), Point2f(15,10+currentHeight), Scalar::all(200), 1, cv::LINE_AA);
    // putText(img_backproj, txt2, Point2f(20,currentHeight+textsize.height+2), fontface, fontscale, Scalar(255,255,255), thickness, 8);

    /* alpha blend outliers and add key to the final image */
    // alpha blend the outliers over the final image
    addWeighted(img_outliers, alpha, img_backproj, 1.0 - alpha, 0, img_backproj);
    imshow("backprojection error of 3D-2D matches", img_backproj);
}

/*************************************************************************************/
/*                      sparse motion estimation naive                               */
/*************************************************************************************/

void MotionEstimation::estimateMotionSparse_naive(const Mat& _img_l, const Mat& _img_r)
{
    /* compute stereo correspondences */
    std::vector<Point2f> pts_l, pts_r;
    std::vector<size_t>  indices_l, indices_r;
    extractStereoCorrORB_naive(_img_l, _img_r, pts_l, pts_r, indices_l, indices_r);

    /* reconstruct 3D points from correspondences */
    std::vector<Point3f> csPts3D;
    reconstruct3DFromStereoORB_naive(pts_l, pts_r, csPts3D);

    // TODO visualize points
}

void MotionEstimation::extractStereoCorrORB_naive(const Mat& _in_img_l, const Mat& _in_img_r, std::vector<Point2f>& _points_l, std::vector<Point2f>& _points_r, std::vector<size_t>& _indices_l, std::vector<size_t>& _indices_r)
{
    /* naivly detect ORB features */
    Mat out_img, descriptors_l, descriptors_r;
    std::vector<KeyPoint> keypoints_l, keypoints_r;
    orb_cv->detectAndCompute(_in_img_l, Mat(), keypoints_l, descriptors_l, false);
    orb_cv->detectAndCompute(_in_img_r, Mat(), keypoints_r, descriptors_r, false);

    /* naivly match ORB features */
    matcher = BFMatcher(NORM_HAMMING, false);
    std::vector<DMatch> matches_raw, matches_filtered;
    matcher.match(descriptors_l, descriptors_r, matches_raw);

    /* filter ORB matches using rectified epipolar line constraint */
    filterByRectEpiConstrained(matches_raw, keypoints_l, keypoints_r, N_ORB_PTS, matches_filtered);

    /* extract 2D points from matches */
    // NOTE _points_{l,r}[i] <-> keypoints_{l,r}[indices_{l,r}[i]]
    // NOTE _points_{l,r}[i] <-> descriptors_{l,r}[indices_{l,r}[i]]
    getPointsFromMatches(matches_filtered, keypoints_l, keypoints_r, _points_l, _points_r, _indices_l, _indices_r);

    /* DEBUG draw epipolar lines for detected features */
    drawEpipolarLines(F, _in_img_l, _in_img_r, _points_l, _points_r);
    // drawEpipolarLines(fund_mat, _in_img_l, _in_img_r, _points_l, _points_r);

    /* DEBUG draw matches */
    Mat matches_img;
    drawMatches(_in_img_l, keypoints_l, _in_img_r, keypoints_r, matches_filtered, matches_img);
    imshow("matches", matches_img);
}

void MotionEstimation::reconstruct3DFromStereoORB_naive(const std::vector<Point2f>& _in_points_l, const std::vector<Point2f>& _in_points_r, std::vector<Point3f>& _out_points3d)
{
    /* initialize 3D points array */
    _out_points3d.clear();
    _out_points3d.reserve(_in_points_l.size());

    /* correct matches */
    std::vector<Point2f> pts_l, pts_r;
    correctMatches(F, _in_points_l, _in_points_r, pts_l, pts_r);

    /* triangulate 3D points from 2D correspondences */
    Mat points3D_hom;
    triangulatePoints(Prect00, Prect01, pts_l, pts_r, points3D_hom);

    /* convert points from homogenous to cartesian */
    std::vector<Vec<float,4>> points3D_hom_vec;
    mat4N2vec(points3D_hom, points3D_hom_vec);
    convertPointsFromHomogeneous(points3D_hom_vec, _out_points3d);
}

/******************************************************************************/
/*                          dense motion estimation                           */
/******************************************************************************/

void MotionEstimation::estimateMotionDense(const Mat& _img_l, const Mat& _img_r)
{
    /* reconstruct 3D points from disparity */
    std::vector<Point3f> csPts3D;
    // NOTE csPts3D[i] <-> _img_l.at<float>(indices[i].x, indices[i].y)
    std::vector<Point2f> pts2D;
    reconstruct3DFromDisparityMap(_img_l, _img_r, csPts3D, pts2D);

    // TODO visualize points
}

void MotionEstimation::reconstruct3DFromDisparityMap(const Mat& _in_img_l, const Mat& _in_img_r, std::vector<Point3f>& _out_points3D, std::vector<Point2f>& _out_points2D)
{
    /* compute disparity map */
    Mat imgDisparity16S = Mat(_in_img_l.rows, _in_img_l.cols, CV_16S);
    int ndisp = int(fx/40.0);
    stereoMatcher->compute(_in_img_l, _in_img_r, imgDisparity16S);

    /* show disparity map */
    double minVal, maxVal;
    minMaxLoc(imgDisparity16S, &minVal, &maxVal);
    Mat imgDisparity8U  = Mat(_in_img_l.rows, _in_img_l.cols, CV_8UC1);
    // normalize (alpha = X*255/(maxVal-minVal)) and convert disparity map for presentation
    imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255/(maxVal-minVal));
    imshow("disparity map", imgDisparity8U);

    /* triangulate 3D points from disparity */
    _out_points3D.clear(); _out_points2D.clear();
    _out_points3D.reserve(imgDisparity16S.cols*imgDisparity16S.rows);
    _out_points2D.reserve(imgDisparity16S.cols*imgDisparity16S.rows);
    int idxCount = 0;
    // NOTE max depth to reconstruct: 40*B as suggested in "Large-scale 6-DOF SLAM with stereo-in-hand"
    const short thMaxDepth = 40.0*bf/fx;
    for(size_t n = 0; n < imgDisparity16S.rows; ++n)
    {
        for(size_t m = 0; m < imgDisparity16S.cols; ++m)
        {
            // NOTE shift disparity value 4 times since it is represented in fixed point with 4 fractional bits
            float disparity = float(imgDisparity16S.at<short>(n,m))/16.0;
            // check if disparity exists
            if(disparity >= 0)
            {
                float depth = bf/disparity;
                // only triangulate points with depth <= 40*b
                if(depth <= thMaxDepth)
                {
                    double x = (m-cx)*depth/fx;
                    double y = (n-cy)*depth/fy;
                    _out_points3D.push_back(Point3f(x, y, depth));
                    _out_points2D.push_back(Point2f(m,n));
                }
            }
        }
    }
}

void MotionEstimation::drawEpipolarLines(const Mat& _F, const Mat& _img1, const Mat& _img2, const std::vector<Point2f> _points1, const std::vector<Point2f> _points2)
{
    Mat outImg(_img1.rows, _img1.cols*2, CV_8UC3);
    Rect rect1(0,0, _img1.cols, _img1.rows);
    Rect rect2(_img1.cols, 0, _img1.cols, _img1.rows);
    // allow color drawing
    if (_img1.type() == CV_8U)
    {
        cv::cvtColor(_img1, outImg(rect1), cv::COLOR_GRAY2BGR);
        cv::cvtColor(_img2, outImg(rect2), cv::COLOR_GRAY2BGR);
    }
    else
    {
        _img1.copyTo(outImg(rect1));
        _img2.copyTo(outImg(rect2));
    }

    std::vector<Vec<float,3>> epilines1, epilines2;
    computeCorrespondEpilines(_points1, 1, _F, epilines1);
    computeCorrespondEpilines(_points2, 2, _F, epilines2);

    /* epipolar lines of the 1st point set are drawn in the 2nd image and vice-versa */
    cv::RNG rng(0);
    for(size_t i=0; i < _points1.size(); ++i)
    {
        cv::Scalar color(rng(256),rng(256),rng(256));
        line(outImg(rect2),
             Point(0,-epilines1[i][2]/epilines1[i][1]),
             Point(_img1.cols,-(epilines1[i][2]+epilines1[i][0]*_img1.cols)/epilines1[i][1]),
             color);
        circle(outImg(rect1), _points1[i], 3, Scalar(256,0,0), 1, cv::LINE_AA);

        // line(outImg(rect1),
        //       Point(0,-epilines2[i][2]/epilines2[i][1]),
        //       Point(_img2.cols,-(epilines2[i][2]+epilines2[i][0]*_img2.cols)/epilines2[i][1]),
        //       color);
        circle(outImg(rect2), _points2[i], 3, Scalar(256,0,0), -1, cv::LINE_AA);
    }
    // show epipolar lines
    imshow("epipolar lines", outImg);
}

void MotionEstimation::readCalibFromFile_KITTI(const std::string& _path_calib)
{
    // open file storage
    std::string calib_file = _path_calib + "calib_cam_to_cam_converted.txt";
    FileStorage fs(calib_file, FileStorage::READ);
    Log("", "\033[1;47;30m", "read calibration data from \"" + calib_file + "\"...", "", "");
    fs["P_rect_00"] >> Prect00;
    fs["P_rect_01"] >> Prect01;
    fs["S_00"] >> S00;
    fs["S_01"] >> S01;
    fs["S_rect_00"] >> Srect00;
    fs["S_rect_01"] >> Srect01;
    fs["T_01"] >> T01;
    fs["R_rect_00"] >> Rrect00;
    fs["R_rect_01"] >> Rrect01;
    fs.release();

    // extract camera data from rectified data
    fx =  Prect01.at<double>(0,0);
    fy =  Prect01.at<double>(1,1);
    cx =  Prect01.at<double>(0,2);
    cy =  Prect01.at<double>(1,2);
    bf = -Prect01.at<double>(0,3);

    // set precomputed fundamental matrix
    // precomputed F
    F = (Mat_<double>(3,3) << -5.093170329928398e-11, 0.0002084672451019287, -0.04377365112304688, -0.0002077221870422363, 4.472583532333374e-05, 1461585517467.438, 0.04368209838867188, -1461585517467.452, 1.0);

    // compose camera calibration matrix
    K = Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    // write calibration data to log
    Log("", "", "\tdone!", "\033[0m", "");
    Log("\n\n", "\033[1;47;30m", "calibration data", "\033[0m", "\n\n");
    Log("Prect00:\n" + mat2str(Prect00) + "\n");
    Log("Prect01:\n" + mat2str(Prect01) + "\n");
    Log("S00: " + mat2str(S00) + "\n");
    Log("S01: " + mat2str(S01) + "\n");
    Log("Srect00: " + mat2str(Srect00) + "\n");
    Log("Srect01: " + mat2str(Srect01) + "\n");
    Log("T01:\n" + mat2str(T01) + "\n");
    Log("Rrect00:\n" + mat2str(Rrect00) + "\n");
    Log("Rrect01:\n" + mat2str(Rrect01) + "\n");
    Log("K:\n" + mat2str(K) + "\n");
    Log("\n", "\033[1;47;30m", "camera intrinsics after rectification", "\033[0m", "\n\n");
    Log("fx: " + std::to_string(fx) + "\n");
    Log("fy: " + std::to_string(fy) + "\n");
    Log("cx: " + std::to_string(cx) + "\n");
    Log("cy: " + std::to_string(cy) + "\n");
    Log("bf: " + std::to_string(bf) + "\n");
}

void MotionEstimation::readCalibFromFile_EuRoC(const std::string& _path_calib)
{
    std::cout << "calib path: " << _path_calib << std::endl;
    // open file storage
    FileStorage fs(_path_calib, FileStorage::READ);
    Log("", "\033[1;47;30m", "read calibration data from \"" + _path_calib + "\"...", "", "");

    // FIXME check if Prect00 and Prect01 are set correctly
    fs["LEFT.P"] >> Prect00;
    fs["RIGHT.P"] >> Prect01;
    fs["LEFT.R"] >> Rrect00; // TODO Rrect00 = essential matrix ?
    fs["RIGHT.R"] >> Rrect01;

    fs.release();

    // extract camera data from rectified data
    fx =  Prect01.at<double>(0,0);
    fy =  Prect01.at<double>(1,1);
    cx =  Prect01.at<double>(0,2);
    cy =  Prect01.at<double>(1,2);
    bf = -Prect01.at<double>(0,3);

    // set precomputed fundamental matrix
    // precomputed F
    F = (Mat_<double>(3,3) << -5.093170329928398e-11, 0.0002084672451019287, -0.04377365112304688, -0.0002077221870422363, 4.472583532333374e-05, 1461585517467.438, 0.04368209838867188, -1461585517467.452, 1.0);

    // compose camera calibration matrix
    K = Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    // write calibration data to log
    Log("", "", "\tdone!", "\033[0m", "");
    Log("\n\n", "\033[1;47;30m", "calibration data", "\033[0m", "\n\n");
    Log("Prect00:\n" + mat2str(Prect00) + "\n");
    Log("Prect01:\n" + mat2str(Prect01) + "\n");
    Log("Rrect00:\n" + mat2str(Rrect00) + "\n");
    Log("Rrect01:\n" + mat2str(Rrect01) + "\n");
    Log("K:\n" + mat2str(K) + "\n");
    Log("\n", "\033[1;47;30m", "camera intrinsics after rectification", "\033[0m", "\n\n");
    Log("fx: " + std::to_string(fx) + "\n");
    Log("fy: " + std::to_string(fy) + "\n");
    Log("cx: " + std::to_string(cx) + "\n");
    Log("cy: " + std::to_string(cy) + "\n");
    Log("bf: " + std::to_string(bf) + "\n");
}
