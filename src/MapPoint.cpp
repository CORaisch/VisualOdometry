#include "MapPoint.hpp"

MapPoint::MapPoint() : valid(false) {};

MapPoint::MapPoint(Point3f pt) : pt_cs(pt), valid(true) {};

bool MapPoint::isValid()
{
    return valid;
}