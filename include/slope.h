#pragma once

#include "utils.h"
#include "linalg.h"

using LinearAlgebra::Vector2D;
using Utils::Point2D;
using Utils::WorldObject;

class Slope : public virtual WorldObject
{
public:
    Slope(int id, const Point2D &start, const Point2D &end);

    inline const float getLength() const
    {
        return length_;
    }

    inline const Point2D getStart() const
    {
        return start_;
    }

    inline const Point2D getEnd() const
    {
        return end_;
    }

    inline const Vector2D getSlope() const
    {
        return slope_;
    }

    inline const Vector2D getNormal() const
    {
        return normal_;
    }

    void printState() const override;

private:
    float length_;    // m
    Point2D start_;   // [m, m]
    Point2D end_;     // [m, m]
    Vector2D slope_;  // [m, m]
    Vector2D normal_; // [m, m]
};
