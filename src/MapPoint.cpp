#include "MapPoint.hpp"

#include <string>

MapPoint::MapPoint() : valid(false) {};

MapPoint::MapPoint(Point3f pt) : pt_cs(pt), valid(true) {};

bool MapPoint::isValid()
{
    return valid;
}
