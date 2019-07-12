#ifndef _MAPPOINT_HPP
#define _MAPPOINT_HPP

#include <opencv2/opencv.hpp>

using namespace cv;

class MapPoint
{
public:
    MapPoint();

    MapPoint(Point3f pt);

    bool isValid();

    Point3f pt_cs;
    Point3f pt_ws;

    KeyPoint key;

private:
    bool valid;
};

#endif
