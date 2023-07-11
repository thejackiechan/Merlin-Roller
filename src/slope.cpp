#include "../include/slope.h"

Slope::Slope(int id, const Point2D &start, const Point2D &end) : start_{start}, end_{end}
{
    setID(id);
    setObjType(Utils::ObjectType::Slope);
    slope_ = Vector2D(end, start);
    normal_ = Vector2D{-slope_.y(), slope_.x()};
    length_ = Utils::computeMagnitude(slope_.x(), slope_.y());
}