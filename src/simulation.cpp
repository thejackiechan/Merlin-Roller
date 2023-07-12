#include "../include/simulation.h"
#include "../include/physics.h"

#include <iostream>
#include <limits>

Simulation::Simulation(PtrVec<Utils::WorldObject> &objVec, float dt, float duration)
    : dt_{dt}, endTime_{duration}
{
    processWorldObjects(objVec);
    runSimulation();
}

void Simulation::processWorldObjects(PtrVec<Utils::WorldObject> &objVec)
{
    int ballIdx = 0;
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
            processBallObject(obj, ballIdx);
        }
    }
}

void Simulation::processBallObject(std::shared_ptr<Utils::WorldObject> &obj, int &objIdx)
{
    auto ballPtr = std::dynamic_pointer_cast<Ball>(obj);
    if (ballPtr != nullptr)
    {
        BallID id = ballPtr->getID();
        ballObjects_.emplace_back(ballPtr);
        ballBallAsscMap_.insert(std::make_pair(id, objIdx));
        ballSlopeAsscMap_.insert(std::make_pair(id, kResetIdx));
        objIdx++;
    }
}

void Simulation::printSlopeImpactTimes(const Ball &ball) const
{
    // one ball can have many slopes
    const int ballID = ball.getID();
    auto impactFound = slopeImpactsMap_.find(ballID);
    if (impactFound != slopeImpactsMap_.end())
    {
        auto impacts = impactFound->second;
        for (const auto &impact : impacts)
        {
            std::cout << "Ball " << ballID << " Slope " << impact.slopeID << " (first = "
                      << impact.firstTime << ", last = " << impact.lastTime << ") \n";
        }
    }
}

void Simulation::runSimulation()
{
    while (currTime_ <= endTime_ + kEpsilon)
    {
        tick();
        currTime_ += dt_;
    }
}

void Simulation::initializeHeightsMap(BallIDMap<float> &heightsMap)
{
    for (int i = 0; i < ballObjects_.size(); ++i)
    {
        BallID id = ballObjects_.at(i)->getID();
        heightsMap.insert(std::make_pair(id, std::numeric_limits<float>::max()));
    }
}

void Simulation::resetInvalidPairsInAsscMap()
{
    // For each ball-slope pair in objAsscMap, see if ball is no longer within bounds
    for (auto &[ballID, slopeIdx] : ballSlopeAsscMap_)
    {
        if (slopeIdx != kResetIdx)
        {
            auto ballPtr = ballObjects_.at(ballBallAsscMap_.at(ballID));
            auto slopePtr = slopeObjects_.at(slopeIdx);
            Point2D closestPtOnSlope;
            if (!Physics::isBallInBounds(*ballPtr, *slopePtr, closestPtOnSlope))
            {
                slopeIdx = kResetIdx;
            }
        }
    }
}

void Simulation::updateHeightsMapWithNewPair(BallID ballID, std::shared_ptr<Ball> ballPtr,
                                             BallIDMap<float> &heightsMap)
{
    for (int i = 0; i < slopeObjects_.size(); ++i)
    {
        auto slopePtr = slopeObjects_.at(i);
        Point2D closestPtOnSlope;

        // If in bounds, get the slope closest to the ball
        if (Physics::isBallInBounds(*ballPtr, *slopePtr, closestPtOnSlope))
        {
            float heightAboveSurface = Physics::computeHeightAboveSurface(*ballPtr, closestPtOnSlope);
            auto itr = heightsMap.find(ballID);
            if (heightAboveSurface < itr->second)
            {
                ballSlopeAsscMap_.at(ballID) = i;
                itr->second = heightAboveSurface;
            }
        }
    }
}

void Simulation::updateHeightsMapWithAsscMap(BallIDMap<float> &heightsMap)
{
    // For each ball-slope pair in objAsscMap, see if ball is no longer within bounds
    resetInvalidPairsInAsscMap();

    // For each ball, loop over all slopes to see which it is associated with
    for (auto &ballPtr : ballObjects_)
    {
        auto ballID = ballPtr->getID();
        auto slopeIdx = ballSlopeAsscMap_.at(ballID);

        // If ball is not associated with a slope, loop through slopes and check if there is one
        if (slopeIdx == kResetIdx)
        {
            updateHeightsMapWithNewPair(ballID, ballPtr, heightsMap);
        }
        else
        { // ball is associated with a slope, we just update its height
            updateHeightsMapWithExistingPair(ballID, ballPtr, slopeIdx, heightsMap);
        }
    }
}

void Simulation::updateHeightsMapWithExistingPair(BallID ballID, std::shared_ptr<Ball> ballPtr,
                                                  ObjIdx slopeIdx, BallIDMap<float> &heightsMap)
{
    auto slopePtr = slopeObjects_.at(slopeIdx);
    Point2D closestPtOnSlope = Physics::getClosestPtOnSlope(*ballPtr, *slopePtr);
    heightsMap.at(ballID) = Physics::computeHeightAboveSurface(*ballPtr, closestPtOnSlope);
}

// Records time when a slope is first impacted and last impacted by a ball
void Simulation::updateImpactData(BallID ballID, SlopeID slopeID)
{
    auto impactsItr = slopeImpactsMap_.find(ballID);

    if (impactsItr == slopeImpactsMap_.end())
    {
        // Insert SlopeImpact
        auto slopeImpact = Utils::SlopeImpact{slopeID, currTime_, currTime_};
        auto slopeImpactVec = {slopeImpact};
        slopeImpactsMap_.insert(std::make_pair(ballID, slopeImpactVec));
    }
    else
    {
        // Update SlopeImpact
        bool containsSlopeID{false};
        for (auto &slopeImpact : impactsItr->second)
        {
            if (slopeImpact.slopeID == slopeID)
            {
                containsSlopeID = true;
                slopeImpact.lastTime = currTime_;
            }
        }
        if (!containsSlopeID)
        {
            // New slope so let's add a new SlopeImpact
            auto slopeImpact = Utils::SlopeImpact{slopeID, currTime_, currTime_};
            impactsItr->second.push_back(slopeImpact);
        }
    }
}

void Simulation::tick()
{
    // Reset heights above surface
    BallIDMap<float> heightsAboveSurf;
    initializeHeightsMap(heightsAboveSurf);
    updateHeightsMapWithAsscMap(heightsAboveSurf);

    for (auto &[ballID, height] : heightsAboveSurf)
    {
        auto ballPtr = ballObjects_.at(ballBallAsscMap_.at(ballID));
        Vector2D velVec = ballPtr->getVelVec();
        float mass = ballPtr->getMass();

        printBallState(ballPtr);

        // initialize force and torque for free fall case (no slope contact)
        Vector2D forceSum{Physics::kGravForce * mass};
        float torqueSum{0.f};

        // impact, so we have additional forces
        if (height < kEpsilon)
        {
            auto slopePtr = slopeObjects_.at(ballSlopeAsscMap_.at(ballID));
            SlopeID slopeID = slopePtr->getID();

            updateImpactData(ballID, slopeID);

            // compute force sum
            float normalMag{0.f};
            Vector2D normalForce = Physics::computeNormalForce(mass, velVec, slopePtr->getNormal(),
                                                               height, dt_, normalMag);

            float frictionMag{0.f};
            Vector2D frictionForce = Physics::computeFrictionForce(*ballPtr, velVec, slopePtr->getSlope(),
                                                                   normalMag, dt_, frictionMag);

            forceSum = forceSum + normalForce + frictionForce;

            // compute torque sum
            if (frictionMag != 0.f)
            {
                Point2D contactPoint = Physics::getClosestPtOnSlope(*ballPtr, *slopePtr);
                torqueSum = Physics::computeFricTorque(*ballPtr, contactPoint, frictionForce);
            }
        }
        // update kinematic parameters
        updateKinematics(ballPtr, forceSum, torqueSum);
    }
}

void Simulation::updateKinematics(std::shared_ptr<Ball> ballPtr, const Vector2D &forceSum,
                                  float torqueSum)
{
    Vector2D velVec = ballPtr->getVelVec();

    // update linear params
    Vector2D accelVec = Physics::computeAccelVec(forceSum, ballPtr->getMass());
    velVec = Physics::updateVecParam(velVec, accelVec, dt_);
    Vector2D posVec = Physics::updateVecParam(Vector2D{ballPtr->getCenter()}, velVec, dt_);

    ballPtr->setLinAccs(accelVec);
    ballPtr->setLinVels(velVec);
    ballPtr->setCenter(posVec);

    // update angular params
    float angAcc = Physics::computeAngAcc(torqueSum, ballPtr->getInertia());
    float angVel = Physics::updateScalarParam(ballPtr->getAngVel(), angAcc, dt_);

    ballPtr->setAngAcc(angAcc);
    ballPtr->setAngVel(angVel);
}

void Simulation::printBallState(std::shared_ptr<Ball> ballPtr)
{
    std::cout << "t = " << currTime_ << " : ";
    ballPtr->printState();
}