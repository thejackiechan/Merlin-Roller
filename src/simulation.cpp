#include "../include/simulation.h"

#include <cmath>
#include "physics.h"

Simulation::Simulation(SharedPtrVec<Utils::WorldObject> &objVec, float dt, float duration)
    : dt_{dt}, endTime_{duration}
{
    processWorldObjects(objVec);
}

void Simulation::processWorldObjects(SharedPtrVec<Utils::WorldObject> &objVec)
{
    for (auto &obj : objVec)
    {
        auto objType = obj->getObjType();
        if (objType == ObjectType::Slope)
        {
            auto slopePtr = std::dynamic_pointer_cast<Slope>(obj);
            if (slopePtr != nullptr)
                slopeObjects_.emplace_back(slopePtr);
        }
        else if (objType == ObjectType::Ball)
        {
            int count = 0;
            auto ballPtr = std::dynamic_pointer_cast<Ball>(obj);
            if (ballPtr != nullptr)
            {
                BallID id = ballPtr->getID();
                ballObjects_.emplace_back(ballPtr);
                ballBallAsscMap_.insert(std::make_pair(id, count));
                ballSlopeAsscMap_.insert(std::make_pair(id, kResetIdx));
                hasImpactedMap_.insert(std::make_pair(id, false));
                count++;
            }
        }
    }
}

// void Simulation::printSlopeImpactTimes(const Ball &ball) const
// {
//     // one ball can have many slopes
//     const int ballID = ball.getID();
//     auto impactFound = ballSlopeAsscMap_.find(ballID);
//     if (impactFound != ballSlopeAsscMap_.end())
//     {
//         // auto &impacts = impactFound->second;
//         // for (const auto &impact : impacts)
//         // {
//         //     std::cout << "Ball " << ballID << " Slope " << impact.slopeID << " (first = "
//         //               << impact.firstTime << ", last = " << impact.lastTime << ") \n";
//         // }
//     }
// }

void Simulation::Tick()
{
    // Reset heights above surface
    std::unordered_map<BallID, float> heightsAboveSurf;
    for (int i = 0; i < ballObjects_.size(); ++i)
    {
        BallID id = ballObjects_.at(i)->getID();
        heightsAboveSurf.insert(std::make_pair(id, std::numeric_limits<float>::max()));
    }

    // For each ball-slope pair in objAsscMap, see if ball is no longer within bounds
    for (auto &[ballID, slopeIdx] : ballSlopeAsscMap_)
    {
        if (slopeIdx != kResetIdx)
        {
            auto ballPtr = ballObjects_.at(ballBallAsscMap_.at(ballID));
            auto slopePtr = slopeObjects_.at(slopeIdx);
            if (!Physics::isBallInBounds(*ballPtr, *slopePtr))
            {
                slopeIdx = kResetIdx;
            }
        }
    }

    // For each ball, loop over all slopes to see which it is associated with
    for (auto &ballPtr : ballObjects_)
    {
        BallID ballID = ballPtr->getID();
        auto slopeIdx = ballSlopeAsscMap_.at(ballID);
        // If ball is not associated with a slope, loop through slopes and check if there is (new) one
        if (slopeIdx == kResetIdx)
        {
            for (int i = 0; i < slopeObjects_.size(); ++i)
            {
                auto slopePtr = slopeObjects_.at(i);
                // If in bounds, get the slope closest to the ball
                if (Physics::isBallInBounds(*ballPtr, *slopePtr))
                {
                    Point2D closestPtOnSlope = Physics::getClosestPtOnSlope(*ballPtr, *slopePtr);
                    float heightAboveSurface = Physics::computeHeightAboveSurface(*ballPtr, closestPtOnSlope);

                    auto itr = heightsAboveSurf.find(ballID);
                    if (heightAboveSurface < itr->second)
                    {
                        ballSlopeAsscMap_.at(ballID) = i;
                        itr->second = heightAboveSurface;
                    }
                }
            }
        }
        else
        { // ball is associated, we just update its heightAboveSurface
            auto slopePtr = slopeObjects_.at(slopeIdx);
            Point2D closestPtOnSlope = Physics::getClosestPtOnSlope(*ballPtr, *slopePtr);
            heightsAboveSurf.at(ballID) = Physics::computeHeightAboveSurface(*ballPtr, closestPtOnSlope);
        }
    }

    // Loop through objects
    for (auto &[ballID, height] : heightsAboveSurf)
    {
        auto ballPtr = ballObjects_.at(ballBallAsscMap_.at(ballID));
        Vector2D ballVelVec = ballPtr->getVelVec();
        Vector2D forceSum{Physics::kGravForce};

        // impact, so we have additional forces
        if (height < kEpsilon)
        {
            auto slopePtr = slopeObjects_.at(ballSlopeAsscMap_.at(ballID));
            auto slopeNormal = slopePtr->getNormal();
            float normalMag{0.f};
            auto hasImpactedItr = hasImpactedMap_.find(ballID);

            // compute normal force magnitude
            if (hasImpactedItr->second)
            {
                normalMag = Physics::computeNormalForceMag(slopeNormal);
            }
            else
            {
                float mass = ballPtr->getMass();
                normalMag = Physics::computeStoppingForce(mass, slopeNormal, ballVelVec, dt_) +
                            Physics::computeGravResistForce(mass, slopeNormal) + Physics::computeRestoringForce(height);
                hasImpactedItr->second = true;
            }
            // turn normal force into vector
            Vector2D normalForce = Physics::getNormalForceVec(normalMag, slopeNormal);

            // now compute friction
            float frictionMag{0.f};
            float ballRadius = ballPtr->getRadius();
            float velDiff = Physics::computeVelDiff(ballPtr->getLinearVelocity(), ballPtr->getAngVel(), ballRadius);

            if (std::abs(velDiff) < kEpsilon)
            {
                // maybe static friction applies
                float gravAlongSlopeMag = Physics::computeGravAlongSlope(slopePtr->getSlope());
                float statFricMag = std::min(Physics::computeMaxStatFric(normalMag), gravAlongSlopeMag);

                if (statFricMag > gravAlongSlopeMag)
                    frictionMag = statFricMag;
            }
            else
            {
                // dynamic friction applies
                frictionMag = std::min(Physics::computeMaxDynFric(normalMag),
                                       Physics::computeFricCeaseSliding(ballPtr->getInertia(), velDiff, ballRadius, dt_));
            }
            // turn friction force into vector
            
        }
    }
}