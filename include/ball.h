#pragma once

#include <iostream>
#include "utils.h"
#include "linalg.h"

using LinearAlgebra::Vector2D;
using Utils::Point2D;
using Utils::WorldObject;

class Ball : public virtual WorldObject
{
public:
    Ball(int id, const Point2D &center, float radius, float mass, float xVel = 0.0, float yVel = 0.0);

    inline const Point2D getCenter() const
    {
        return center_;
    }

    inline const float getRadius() const
    {
        return radius_;
    }

    inline const float getMass() const
    {
        return mass_;
    }

    const float getLinearVelocity()
    {
        linVel_ = Utils::computeMagnitude(xVel_, yVel_);
        return linVel_;
    }

    const Vector2D getVelVec() const {
        return Vector2D{xVel_, yVel_};
    }

    const Vector2D getAccVec() const {
        return Vector2D{xAcc_, yAcc_};
    }

    inline const float getAngVel() const
    {
        return angVel_;
    }

    inline const float getAngAcc() const
    {
        return angAcc_;
    }

    inline const float getInertia() const
    {
        return inertia_;
    }

    void printState() const override
    {
        std::cout << "Ball " << getID() << " (x = " << center_.x
                  << ", y = " << center_.y << ") (vx = " << xVel_
                  << ", vy = " << yVel_ << ") (wz = " << angVel_ << ") \n";
    }

    // setters

private:
    Point2D center_; // [m, m]
    float radius_;   // m
    float mass_;     // kg
    float linVel_;   // m/s
    float xVel_;     // m/s
    float yVel_;     // m/s
    float xAcc_;     // m/s/s
    float yAcc_;     // m/s/s
    float angVel_;   // rad/s
    float angAcc_;   // rad/s/s
    float inertia_;  // kg * m^2
};