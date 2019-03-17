#include "Frame.hpp"

/* default constructor -> should not be used except implicitly (then must overide before use) */
Frame::Frame() {}

/* copy constructor */
// Frame::Frame(const Frame& f) : Log(f.Log), img_l(f.img_l.clone()), img_r(f.img_r.clone()),\
//              fx(f.fx), fy(f.fy), cx(f.cx), cy(f.cy), bfx(f.bfx), F(f.F.clone()),\
//              orb_extractor_l(f.orb_extractor_l), orb_extractor_r(f.orb_extractor_r),\
//              descriptors_l(f.descriptors_l.clone()), descriptors_r(f.descriptors_r.clone()),\
//              keypoints_l(f.keypoints_l), keypoints_r(f.keypoints_r), matches(f.matches),\
//              depth(f.depth), points3D(f.points3D)
// {}

/* constructor */
/* extracts stereo correspondences and triangulate all points that are certain enough */
Frame::Frame(Logging& _log, const Mat& _img_l, const Mat& _img_r, const double _fx, const double _fy,\
             const double _cx, const double _cy, const double _bfx, const Mat& _F, ORBextractor* _orb_l,\
             ORBextractor* _orb_r, const bool bDebugMode) :
             Log(_log), img_l(_img_l), img_r(_img_r), fx(_fx), fy(_fy), cx(_cx), cy(_cy), bfx(_bfx),\
             F(_F), orb_extractor_l(_orb_l), orb_extractor_r(_orb_r)
{
    /* compute stereo correspondences */
    extractStereoCorrespondences(bDebugMode);

    /* reconstruct 3D points from correspondences */
    triangulate3D();
}

/* extractStereoCorrespondences */
/* extracts ORB features on both images in parallel and computes*/
/* stereo matches by considering the epioplar constraint        */
void Frame::extractStereoCorrespondences(const bool bDebugMode)
{
    /* detect ORB features with custom extractor */
    Mat out_img;
    std::thread thread_l(&Frame::extractORB, this, 0);
    std::thread thread_r(&Frame::extractORB, this, 1);
    thread_l.join();
    thread_r.join();

    /* match ORB featuers considering epipolar constraint */
    matchByEpiConstraint();

    /* DEBUG draw matches and keypoints */
    if(bDebugMode)
    {
        Mat matches_img;
        drawMatches(img_l, keypoints_l, img_r, keypoints_r, matches, matches_img);
        imshow("stereo matches", matches_img);

        /* DEBUG draw epipolar lines for detected features */
        drawEpipolarLines();
    }
}

/* triangulate3D */
/* triangulates 3D points by unprojecting correspondences using */
/* disparity and baseline for reconstructing depth and scale    */
void Frame::triangulate3D()
{
    float u,v,z;
    double x,y;
    // NOTE max depth to reconstruct with accepted uncertainty is recommended to be 40*baseline as suggested in:
    // Paz, Lina M., et al. "Large-scale 6-DOF SLAM with stereo-in-hand." IEEE transactions on robotics 24.5 (2008): 946-957.
    float maxDepth = 40 * bfx/fx;
    points3D = std::vector<MapPoint>(keypoints_l.size(), MapPoint());
    for(size_t i = 0; i < keypoints_l.size(); ++i)
    {
        z = depth[i];
        // skip correspondences with invalid reconstruction
        if((z < 0) || (z > maxDepth))
            continue;
        // unproject points with valid reconstruction
        u = keypoints_l[i].pt.x;
        v = keypoints_l[i].pt.y;
        x = (u-cx)*z/fx;
        y = (v-cy)*z/fy;
        points3D[i] = MapPoint(Point3f(x,y,z));
        points3D[i].key = keypoints_l[i];
    }
}

/* matchByEpiConstraint */
/* find ORB matches considering the epipolar constraint. Assumging */
/* images to be rectified. Returning depth as bfx/disparity        */
void Frame::matchByEpiConstraint()
{
    depth = std::vector<float>(keypoints_l.size(), -1.0f);
    matches.reserve(keypoints_l.size());

    const int TH_HIGH = 100;
    const int TH_LOW  = 50;
    const int thOrbDist = (TH_HIGH+TH_LOW)/2;

    const int nRows = orb_extractor_l->mvImagePyramid[0].rows;

    // assign keypoints to row table
    std::vector<std::vector<size_t> > vRowIndices(nRows,std::vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = keypoints_r.size();

    for(int iR = 0; iR < Nr; iR++)
    {
        const cv::KeyPoint &kp = keypoints_r[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*orb_extractor_l->GetScaleFactors()[keypoints_r[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi = minr; yi <= maxr; yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = bfx/fx; // b*fx/fx = b
    const float minD = 0;
    const float maxD = bfx/minZ;

    // For each left keypoint search a match in the right image
    std::vector<std::pair<int, int> > vDistIdx;
    vDistIdx.reserve(keypoints_l.size());

    for(int iL = 0; iL < keypoints_l.size(); iL++)
    {
        const cv::KeyPoint &kpL = keypoints_l[iL];
        const int &levelL = kpL.octave;
        const float &vL   = kpL.pt.y;
        const float &uL   = kpL.pt.x;

        const std::vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int    bestDist = TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = descriptors_l.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC = 0; iC < vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = keypoints_r[iR];

            if((kpR.octave < (levelL-1)) || (kpR.octave > (levelL+1)))
                continue;

            const float &uR = kpR.pt.x;

            if((uR >= minU) && (uR <= maxU))
            {
                const cv::Mat &dR = descriptors_r.row(iR);
                const int dist = descriptorDistance(dL,dR);

                if(dist < bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist < thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = keypoints_r[bestIdxR].pt.x;
            const float scaleFactor = orb_extractor_l->GetInverseScaleFactors()[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = orb_extractor_l->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) * cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            std::vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= orb_extractor_r->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR = -L; incR <= +L; incR++)
            {
                cv::Mat IR = orb_extractor_r->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) * cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist < bestDist)
                {
                    bestDist = dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if((bestincR == -L) || (bestincR == L))
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = orb_extractor_l->GetScaleFactors()[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if((disparity >= minD) && (disparity < maxD))
            {
                if(disparity <= 0)
                {
                    disparity = 0.01;
                    bestuR = uL-0.01;
                }
                depth[iL] = bfx/disparity;
                vDistIdx.push_back(std::pair<int,int>(bestDist,iL));
                matches.push_back(DMatch(iL, bestIdxR, -1));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i = vDistIdx.size()-1; i >= 0; i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            depth[vDistIdx[i].second] = -1;
            matches.erase(matches.begin()+vDistIdx[i].second);
        }
    }
}

/* extractORB */
/* extracts ORB features in left and right images depending */
/* on flag using the ORBExtractor class from ORB SLAM       */
void Frame::extractORB(int flag)
{
    if(flag==0)
        (*orb_extractor_l)(img_l, Mat(), keypoints_l, descriptors_l);
    else
        (*orb_extractor_r)(img_r, Mat(), keypoints_r, descriptors_r);
}

/* setPose (1) */
/* assigns frames pose and extracts useful pose information */
void Frame::setPose(const Mat& pose)
{
    // allocate 3x4 matrix for poses
    cTw = Mat::eye(4,4,pose.type());
    wTc = Mat::eye(4,4,pose.type());

    // extract R and t from world to camera
    cRw = pose.rowRange(0,3).colRange(0,3);
    ctw = pose.rowRange(0,3).col(3);
    // compose R and t to new pose form world to camera
    cRw.copyTo(cTw.rowRange(0,3).colRange(0,3));
    ctw.copyTo(cTw.rowRange(0,3).col(3));

    // compute R and t from camera to world (fast inversion)
    wRc = cRw.t();
    wtc = -wRc*ctw;
    // compose inverse R and t to pose from camera to world
    wRc.copyTo(wTc.rowRange(0,3).colRange(0,3));
    wtc.copyTo(wTc.rowRange(0,3).col(3));
}

/* setPose (2) */
/* assigns frames pose and extracts useful pose information */
void Frame::setPose(const Mat& rvec, const Mat& tvec)
{
    // allocate 3x4 matrix for poses
    cTw = Mat::eye(4,4,rvec.type());
    wTc = Mat::eye(4,4,rvec.type());

    // extract R and t from world to camera
    rvec.convertTo(const_cast<Mat&>(rvec), CV_32F);
    tvec.convertTo(const_cast<Mat&>(tvec), CV_32F);
    Rodrigues(rvec, cRw);
    tvec.copyTo(ctw);

    // compose R and t to new pose form world to camera
    cRw.copyTo(cTw.rowRange(0,3).colRange(0,3));
    ctw.copyTo(cTw.rowRange(0,3).col(3));

    // compute R and t from camera to world (fast inversion)
    wRc = cRw.t();
    wtc = -wRc*ctw;
    // compose inverse R and t to pose from camera to world
    wRc.copyTo(wTc.rowRange(0,3).colRange(0,3));
    wtc.copyTo(wTc.rowRange(0,3).col(3));
}

/* transform3D */
/* transforms 3D points from camera to worldspace. This must be   */
/* called from outside since pose is not known from the beginning */
void Frame::transform3D()
{
    // iterate over each 3D point
    for(size_t i = 0; i < points3D.size(); ++i)
    {
        if(points3D[i].isValid())
        {
            // transform 3D point to worldspace
            Mat pt_cs(points3D[i].pt_cs);
            Mat pt_ws = wRc*pt_cs + wtc;
            points3D[i].pt_ws = Point3f(pt_ws.at<float>(0,0),\
                                        pt_ws.at<float>(1,0),\
                                        pt_ws.at<float>(2,0));
        }
    }
}

/* drawEpipolarLines */
/* draws matched features in the left image and */
/* corresponding epipolar lines in right image  */
void Frame::drawEpipolarLines()
{
    Mat outImg(img_l.rows, img_l.cols*2, CV_8UC3);
    Rect rect1(0,0, img_l.cols, img_l.rows);
    Rect rect2(img_l.cols, 0, img_l.cols, img_l.rows);
    // allow color drawing
    if (img_l.type() == CV_8U)
    {
        cv::cvtColor(img_l, outImg(rect1), CV_GRAY2BGR);
        cv::cvtColor(img_r, outImg(rect2), CV_GRAY2BGR);
    }
    else
    {
        img_l.copyTo(outImg(rect1));
        img_r.copyTo(outImg(rect2));
    }

    std::vector<Vec<float,3>> epilines1, epilines2;
    std::vector<size_t> indices1, indices2;
    std::vector<Point2f> points2D_l, points2D_r;
    getPointsFromMatches(matches, keypoints_l, keypoints_r, points2D_l, points2D_r, indices1, indices2);
    computeCorrespondEpilines(points2D_l, 1, F, epilines1);
    computeCorrespondEpilines(points2D_r, 2, F, epilines2);

    /* epipolar lines of the 1st point set are drawn in the 2nd image and vice-versa */
    cv::RNG rng(0);
    for(size_t i = 0; i < points2D_l.size(); ++i)
    {
        cv::Scalar color(rng(256),rng(256),rng(256));
        line(outImg(rect2),
              Point(0,-epilines1[i][2]/epilines1[i][1]),
              Point(img_l.cols,-(epilines1[i][2]+epilines1[i][0]*img_l.cols)/epilines1[i][1]),
              color);
        circle(outImg(rect1), points2D_l[i], 3, Scalar(256,0,0), 1, CV_AA);

        // line(outImg(rect1),
        //       Point(0,-epilines2[i][2]/epilines2[i][1]),
        //       Point(img_r.cols,-(epilines2[i][2]+epilines2[i][0]*img_r.cols)/epilines2[i][1]),
        //       color);
        circle(outImg(rect2), points2D_r[i], 3, Scalar(256,0,0), -1, CV_AA);
    }
    // show epipolar lines
    imshow("selected features + corresponding epipolar lines", outImg);
}
