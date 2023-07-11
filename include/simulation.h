#pragma once

#include <iostream>
#include <limits>
#include <memory>
#include <vector>
#include <unordered_map>

#include "utils.h"
#include "slope.h"
#include "ball.h"

template <typename T>
using SharedPtrVec = std::vector<std::shared_ptr<T>>;

using ObjectType = Utils::ObjectType;
using BallID = int;
using Timestamp = float;
using ObjIdx = int;

// simulation should end once all balls exit the screen (i.e. roll off slope and fall below y = 0)
class Simulation
{
public:
    static constexpr int kBottomBound{0}; // [m]
    static constexpr int kResetIdx{-1};    // reset itr to -1
    const float kEpsilon{0.001};

    Simulation(SharedPtrVec<Utils::WorldObject> &objVec, float dt = 0.02, float duration = 10.f);
    
    void printSlopeImpactTimes(const Ball &ball) const;
    void runSimulation();
    
private:
    void processWorldObjects(SharedPtrVec<Utils::WorldObject> &objVec);
    void tick(int &numBallsInBounds);

    SharedPtrVec<Slope> slopeObjects_;
    SharedPtrVec<Ball> ballObjects_;
    float dt_;           // s
    Timestamp currTime_; // s
    Timestamp endTime_;  // s
    std::unordered_map<BallID, ObjIdx> ballBallAsscMap_;
    std::unordered_map<BallID, ObjIdx> ballSlopeAsscMap_;
    std::unordered_map<BallID, bool> hasImpactedMap_;
    std::unordered_map<BallID, std::vector<Utils::SlopeImpact>> slopeImpacts_;
};
