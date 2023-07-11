#include "../include/physics.h"

Vector2D Physics::computeAccelVec(const Vector2D &forceSum, float mass)
{
    return forceSum / mass;
}

Vector2D Physics::updateVecParam(const Vector2D &param, const Vector2D &deriv, float dt)
{
    return (deriv * dt) + param;
}

float Physics::computeAngAcc(float torqueSum, float inertia)
{
    return torqueSum / inertia;
}

float Physics::computeFricTorque(const Ball &ball, const Point2D &contactPoint, const Vector2D &fricForce)
{
    Vector2D radiusLine = computeRadiusLine(ball, contactPoint);
    return radiusLine.cross(fricForce);
}

float Physics::updateScalarParam(float param, float deriv, float dt)
{
    return (deriv * dt) + param;
}

float Physics::computeNormalForceMag(const Vector2D &normalVec)
{
    return -normalVec.dot(kGravForce) / normalVec.getMagnitude();
}

Vector2D Physics::getNormalForceVec(float normalForceMag, const Vector2D &normalVec) {
    Vector2D v{normalVec};
    return v.getUnitVec() * normalForceMag;
}

float Physics::computeSurfaceVel(float angVel, float radius)
{
    return radius * angVel;
}

float Physics::computeVelDiff(float linVel, float angVel, float radius) {
    return linVel - computeSurfaceVel(angVel, radius);
}

float Physics::computeMaxStatFric(float normalForce)
{
    return kFrictionStatic * normalForce;
}

float Physics::computeGravAlongSlope(const Vector2D &slopeVec)
{
    return slopeVec.dot(kGravForce) / slopeVec.getMagnitude();
}

float Physics::computeMaxDynFric(float normalForce)
{
    return kFrictionDynamic * normalForce;
}

float Physics::computeFricCeaseSliding(float inertia, float velDiff, float radius, float dt)
{
    return inertia * velDiff / (dt * radius * radius);
}

float Physics::computeStoppingForce(float mass, const Vector2D &normalVec, const Vector2D &linVelVec, float dt)
{
    return -mass * (normalVec.dot(linVelVec) / normalVec.getMagnitude()) / dt;
}

float Physics::computeGravResistForce(float mass, const Vector2D &normalVec)
{
    return -mass * (normalVec.dot(kGravForce)) / normalVec.getMagnitude();
}

float Physics::computeRestoringForce(float heightAboveSurface)
{
    return kRestoring * std::min(static_cast<float>(0), heightAboveSurface);
}

Point2D Physics::getClosestPtOnSlope(const Ball &b, const Slope &slope)
{
    Point2D slopeStart = slope.getStart();
    Vector2D projection = LinearAlgebra::projAOntoB(Vector2D{b.getCenter(), slopeStart}, slope.getSlope());
    return slopeStart + Point2D(projection.x(), projection.y());
}

bool Physics::isBallInBounds(const Ball &ball, const Slope &slope)
{
    Point2D closestPt = getClosestPtOnSlope(ball, slope);
    Point2D slopeStart = slope.getStart();

    // check horizontal bounds
    if (closestPt.x < slopeStart.x || closestPt.x > slope.getEnd().x)
    {
        return false;
    }
    // check vertical with y = mx + b
    Vector2D slopeVec = slope.getSlope();
    float slopeYVal = (slopeVec.y() / slopeVec.x()) * closestPt.x + slopeStart.y;
    return closestPt.y > slopeYVal;
}

Vector2D Physics::computeRadiusLine(const Ball &ball, const Point2D &closestPtOnSlope)
{
    Vector2D line{closestPtOnSlope, ball.getCenter()};
    return line * ball.getRadius() / line.getMagnitude();
}

float Physics::computeHeightAboveSurface(const Ball &ball, const Point2D &closestPtOnSlope)
{
    Vector2D line{ball.getCenter(), closestPtOnSlope};
    return line.getMagnitude() - ball.getRadius();
}