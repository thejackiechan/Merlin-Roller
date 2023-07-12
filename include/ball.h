#pragma once

#include "utils.h"
#include "linalg.h"

using LinearAlgebra::Vector2D;
using Utils::Point2D;
using Utils::WorldObject;

class Ball : public virtual WorldObject
{
public:
    Ball(int id, const Point2D &center, float radius, float mass,
         float xVel = 0.0, float yVel = 0.0);

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

    inline const float getLinVelMag()
    {
        linVelMag_ = Utils::computeMagnitude(xVel_, yVel_);
        return linVelMag_;
    }

    inline const Vector2D getVelVec() const
    {
        return Vector2D{xVel_, yVel_};
    }

    inline const Vector2D getAccVec() const
    {
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

    inline void setCenter(const Vector2D &posVec)
    {
        center_.x = posVec.x();
        center_.y = posVec.y();
    }

    inline void setLinVels(const Vector2D &velVec)
    {
        xVel_ = velVec.x();
        yVel_ = velVec.y();
    }

    inline void setLinAccs(const Vector2D &accVec)
    {
        xAcc_ = accVec.x();
        yAcc_ = accVec.y();
    }

    inline void setAngVel(const float angVel)
    {
        angVel_ = angVel;
    }

    inline void setAngAcc(const float angAcc)
    {
        angAcc_ = angAcc;
    }

    void printState() const override;

private:
    Point2D center_;    // [m, m]
    float radius_;      // m
    float mass_;        // kg
    float linVelMag_;   // m/s
    float xVel_;        // m/s
    float yVel_;        // m/s
    float xAcc_{0.f};   // m/s/s
    float yAcc_{0.f};   // m/s/s
    float angVel_{0.f}; // rad/s
    float angAcc_{0.f}; // rad/s/s
    float inertia_;     // kg * m^2
};