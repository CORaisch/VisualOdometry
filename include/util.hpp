#ifndef _UTIL_HPP
#define _UTIL_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <math.h>

inline
std::string mat2str(const cv::Mat& mat)
{
    std::string sTmp = "";
    cv::Ptr<cv::Formatted> fmtd = cv::Formatter::get()->format(mat);
    fmtd->reset();
    for(const char* str = fmtd->next(); str; str = fmtd->next())
        sTmp += str;
    return sTmp;
}

inline
void keypoints2points(const std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& points)
{
    for(size_t i = 0; i < keypoints.size(); ++i)
            points.push_back(keypoints[i].pt);
}

inline
void idx2DMatch(std::vector<cv::KeyPoint>& keypoints_l, std::vector<int>& idxR, std::vector<cv::DMatch>& matches)
{
    matches.reserve(keypoints_l.size());
    for(size_t i = 0; i < keypoints_l.size(); ++i)
    {
        if(idxR[i] >= 0)
        {
            cv::DMatch match(i, idxR[i], -1);
            matches.push_back(match);
        }
    }
}

inline
void getPointsFromMatches(const std::vector<cv::DMatch>& matches, const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2, std::vector<size_t>& indices1, std::vector<size_t>& indices2)
{
    indices1.reserve(matches.size());
    indices2.reserve(matches.size());
    for(size_t i = 0; i < matches.size(); ++i)
    {
        points1.push_back(keypoints1[matches[i].queryIdx].pt);
        points2.push_back(keypoints2[matches[i].trainIdx].pt);
        indices1.push_back(matches[i].queryIdx);
        indices2.push_back(matches[i].trainIdx);
    }
}

inline
void mat4N2vec(const cv::Mat& points3D_mat, std::vector<cv::Vec<float,4>>& points3D_vec)
{
    points3D_vec.clear();
    points3D_vec.reserve(points3D_mat.cols);
    for(size_t i = 0; i < points3D_mat.cols; ++i)
        points3D_vec.push_back(points3D_mat.col(i));
}

inline
void filterByRectEpiConstrained(const std::vector<cv::DMatch>& matches_raw, const std::vector<cv::KeyPoint>& keypoints_l, const std::vector<cv::KeyPoint>& keypoints_r, const unsigned int nPts, std::vector<cv::DMatch>& matches_filtered)
{
    cv::Point2f pl, pr;
    double absdiff;
    matches_filtered.clear();
    matches_filtered.reserve(nPts);
    for(size_t i = 0; i < matches_raw.size(); ++i)
    {
        // get point correspondence
        pl = keypoints_l[matches_raw[i].queryIdx].pt;
        pr = keypoints_r[matches_raw[i].trainIdx].pt;
        // since points are rectified check if they are approximatley on the same horizontal line
        absdiff = fabsf(pl.y-pr.y);
        if(absdiff > 5.0) continue;
        matches_filtered.push_back(matches_raw[i]);
    }
}

inline
int descriptorDistance(const cv::Mat& dL, const cv::Mat& dR)
{
    const int *pa = dL.ptr<int32_t>();
    const int *pb = dR.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

#endif //_UTIL_HPP