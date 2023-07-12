#include "../include/physics.h"

#include <cmath>
#include <algorithm>

float Physics::computeFricTorque(const Ball &ball, const Point2D &contactPoint,
                                 const Vector2D &fricForce)
{
    Vector2D radiusLine = computeRadiusLine(ball, contactPoint);
    return radiusLine.cross(fricForce);
}

Vector2D Physics::getForceVec(float forceMag, const Vector2D &direction)
{
    return direction.getUnitVec() * forceMag;
}

float Physics::computeVelDiff(float linVel, float angVel, float radius)
{
    return linVel - std::abs(computeSurfaceVel(angVel, radius));
}

float Physics::computeGravAlongSlope(const Vector2D &slopeVec)
{
    return slopeVec.dot(kGravForce) / slopeVec.getMagnitude();
}

float Physics::computeStoppingForce(float mass, const Vector2D &normalVec,
                                    const Vector2D &linVelVec, float dt)
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

Vector2D Physics::getFricDirVec(const Vector2D &velVec, const Vector2D &slopeVec)
{
    return LinearAlgebra::projAOntoB(velVec, slopeVec) * -1;
}

Point2D Physics::getClosestPtOnSlope(const Ball &b, const Slope &slope)
{
    Point2D slopeStart = slope.getStart();
    Vector2D projection = LinearAlgebra::projAOntoB(Vector2D{b.getCenter(), slopeStart}, slope.getSlope());
    return slopeStart + Point2D(projection.x(), projection.y());
}

// checks if ball is located within x and y bounds of the slope
bool Physics::isBallInBounds(const Ball &ball, const Slope &slope, Point2D &closestPtOnSlope)
{
    closestPtOnSlope = getClosestPtOnSlope(ball, slope);
    Point2D slopeStart = slope.getStart();

    // check horizontal bounds
    if (closestPtOnSlope.x < slopeStart.x || closestPtOnSlope.x > slope.getEnd().x)
    {
        return false;
    }
    // check vertical with y = mx + b
    Vector2D slopeVec = slope.getSlope();
    float slopeYVal = (slopeVec.y() / slopeVec.x()) * closestPtOnSlope.x + slopeStart.y;
    return ball.getCenter().y > slopeYVal;
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

float Physics::computeFrictionMag(Ball &ball, const Vector2D &slopeVec, float normalMag, float dt)
{
    float ballRadius = ball.getRadius();
    float velDiff = Physics::computeVelDiff(ball.getLinVelMag(), ball.getAngVel(), ballRadius);
    float frictionMag{0.f};

    if (std::abs(velDiff) < kEpsilon)
    {
        frictionMag = computeStaticFrictionMag(slopeVec, normalMag);
    }
    else
    {
        frictionMag = computeDynamicFrictionMag(normalMag, velDiff, ball.getInertia(), ballRadius, dt);
    }
    return frictionMag;
}

float Physics::computeStaticFrictionMag(const Vector2D &slopeVec, float normalMag)
{
    float gravAlongSlopeMag = Physics::computeGravAlongSlope(slopeVec);
    float statFricMag = std::min(Physics::computeMaxStatFric(normalMag), gravAlongSlopeMag);

    if (statFricMag > gravAlongSlopeMag)
        return statFricMag;
    return 0.f;
}

float Physics::computeDynamicFrictionMag(float normalMag, float velDiff, float inertia,
                                         float ballRadius, float dt)
{
    float maxDynFric = Physics::computeMaxDynFric(normalMag);
    float fricCeaseSliding = Physics::computeFricCeaseSliding(inertia, velDiff, ballRadius, dt);
    return std::min(maxDynFric, fricCeaseSliding);
}

Vector2D Physics::computeNormalForce(float mass, const Vector2D &velVec, const Vector2D &slopeNormal,
                                     float height, float dt, float &normalMag)
{
    normalMag = computeStoppingForce(mass, slopeNormal, velVec, dt) +
                computeGravResistForce(mass, slopeNormal) +
                computeRestoringForce(height);
    return getForceVec(normalMag, slopeNormal);
}

Vector2D Physics::computeFrictionForce(Ball &ball, const Vector2D &velVec, const Vector2D &slopeVec,
                                       float normalMag, float dt, float &frictionMag)
{
    frictionMag = Physics::computeFrictionMag(ball, slopeVec, normalMag, dt);
    Vector2D fricDirVec = Physics::getFricDirVec(velVec, slopeVec);
    return Physics::getForceVec(frictionMag, fricDirVec);
}