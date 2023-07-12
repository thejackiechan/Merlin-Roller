#pragma once

#include "utils.h"
#include "ball.h"
#include "slope.h"
#include "linalg.h"

using LinearAlgebra::Vector2D;
using Utils::Point2D;

namespace Physics
{
    const float kEpsilon{0.001};
    const float kGravAccel{9.8};        // m/s/s
    const float kFrictionStatic{0.74};  // steel on steel
    const float kFrictionDynamic{0.42}; // steel on steel
    const float kRestoring{1.0};        // kg/s/s
    const Vector2D kGravForce{0, -kGravAccel};

    inline Vector2D computeAccelVec(const Vector2D &forceSum, float mass)
    {
        return forceSum / mass;
    }

    inline Vector2D updateVecParam(const Vector2D &param, const Vector2D &deriv, float dt)
    {
        return (deriv * dt) + param;
    }

    inline float computeAngAcc(float torqueSum, float inertia)
    {
        return torqueSum / inertia;
    }

    inline float updateScalarParam(float param, float deriv, float dt)
    {
        return (deriv * dt) + param;
    }

    inline float computeSurfaceVel(float angVel, float radius)
    {
        return radius * angVel;
    }

    inline float computeMaxStatFric(float normalForce)
    {
        return kFrictionStatic * normalForce;
    }

    inline float computeMaxDynFric(float normalForce)
    {
        return kFrictionDynamic * normalForce;
    }

    inline float computeFricCeaseSliding(float inertia, float velDiff, float radius, float dt)
    {
        return inertia * velDiff / (dt * radius * radius);
    }

    float computeFricTorque(const Ball &ball, const Point2D &contactPoint, const Vector2D &fricForce);

    Vector2D getForceVec(float forceMag, const Vector2D &direction);

    float computeVelDiff(float linVel, float angVel, float radius);

    float computeGravAlongSlope(const Vector2D &slopeVec);

    float computeStoppingForce(float mass, const Vector2D &normalVec, const Vector2D &linVelVec, float dt);

    float computeGravResistForce(float mass, const Vector2D &normalVec);

    float computeRestoringForce(float heightAboveSurface);

    Vector2D getFricDirVec(const Vector2D &velVec, const Vector2D &slopeVec);

    Point2D getClosestPtOnSlope(const Ball &b, const Slope &slope);

    bool isBallInBounds(const Ball &ball, const Slope &slope, Point2D &closestPtOnSlope);

    Vector2D computeRadiusLine(const Ball &ball, const Point2D &closestPtOnSlope);

    float computeHeightAboveSurface(const Ball &ball, const Point2D &closestPtOnSlope);

    float computeFrictionMag(Ball &ball, const Vector2D &slopeVec, float normalMag, float dt);

    float computeStaticFrictionMag(const Vector2D &slopeVec, float normalMag);

    float computeDynamicFrictionMag(float normalMag, float velDiff, float inertia, float ballRadius, float dt);

    Vector2D computeNormalForce(float mass, const Vector2D &velVec, const Vector2D &slopeNormal,
                                float height, float dt, float &normalMag);

    Vector2D computeFrictionForce(Ball &ball, const Vector2D &velVec, const Vector2D &slopeVec, float normalMag, float dt, float &frictionMag);
} // namespace Physics