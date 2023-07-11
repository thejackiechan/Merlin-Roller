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

    Vector2D computeAccelVec(const Vector2D &forceSum, float mass);

    Vector2D updateVecParam(const Vector2D &param, const Vector2D &deriv, float dt);

    float computeAngAcc(float torqueSum, float inertia);

    float computeFricTorque(const Ball &ball, const Point2D &contactPoint, const Vector2D &fricForce);

    float updateScalarParam(float param, float deriv, float dt);

    float computeNormalForceMag(const Vector2D &normalVec);

    Vector2D getNormalForceVec(float normalForceMag, const Vector2D &normalVec);

    float computeSurfaceVel(float angVel, float radius);

    float computeVelDiff(float linVel, float angVel, float radius);

    float computeMaxStatFric(float normalForce);

    float computeGravAlongSlope(const Vector2D &slopeVec);

    float computeMaxDynFric(float normalForce);

    float computeFricCeaseSliding(float inertia, float velDiff, float radius, float dt);

    float computeStoppingForce(float mass, const Vector2D &normalVec, const Vector2D &linVelVec, float dt);

    float computeGravResistForce(float mass, const Vector2D &normalVec);

    float computeRestoringForce(float heightAboveSurface);

    Point2D getClosestPtOnSlope(const Ball &b, const Slope &slope);

    bool isBallInBounds(const Ball &ball, const Slope &slope);

    Vector2D computeRadiusLine(const Ball &ball, const Point2D &closestPtOnSlope);

    float computeHeightAboveSurface(const Ball &ball, const Point2D &closestPtOnSlope);

} // namespace Physics