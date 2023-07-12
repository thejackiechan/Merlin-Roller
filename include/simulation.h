#pragma once

#include "utils.h"
#include "slope.h"
#include "ball.h"

#include <memory>
#include <vector>
#include <unordered_map>

template <typename T>
using PtrVec = std::vector<std::unique_ptr<T>>;

using BallID = int;
using ObjectType = Utils::ObjectType;
using SlopeID = int;
using Timestamp = float;
using ObjIdx = int;

template <typename T>
using BallIDMap = std::unordered_map<BallID, T>;

class Simulation
{
public:
    static constexpr int kResetIdx{-1}; // reset itr to -1
    const float kEpsilon{0.001};

    Simulation(PtrVec<Utils::WorldObject> &objVec, float dt = 0.02, float duration = 10.f);

    void printSlopeImpactTimes(const Ball &ball) const;
    void runSimulation();

private:
    void processWorldObjects(PtrVec<Utils::WorldObject> &objVec);
    void processSlopeObject(std::unique_ptr<Utils::WorldObject> &objPtr);
    void processBallObject(std::unique_ptr<Utils::WorldObject> &objPtr, int &objIdx);
    void initializeHeightsMap(BallIDMap<float> &heightsMap);
    void resetInvalidPairsInAsscMap();
    void updateHeightsMapWithNewPair(BallID ballID, const std::unique_ptr<Ball> &ballPtr,
                                     BallIDMap<float> &heightsMap);
    void updateHeightsMapWithExistingPair(BallID ballID, const std::unique_ptr<Ball> &ballPtr,
                                          ObjIdx slopeIdx, BallIDMap<float> &heightsMap);
    void updateHeightsMapWithAsscMap(BallIDMap<float> &heightsMap);
    void updateImpactData(BallID ballID, SlopeID slopeID);
    void updateKinematics(const std::unique_ptr<Ball> &ballPtr, const Vector2D &forceSum,
                          float torqueSum);
    void printBallState(const std::unique_ptr<Ball> &ballPtr);
    void tick();

    PtrVec<Slope> slopeObjects_;
    PtrVec<Ball> ballObjects_;
    float dt_;                // s
    Timestamp currTime_{0.f}; // s
    Timestamp endTime_;       // s
    BallIDMap<ObjIdx> ballBallAsscMap_;
    BallIDMap<ObjIdx> ballSlopeAsscMap_;
    BallIDMap<std::vector<Utils::SlopeImpact>> slopeImpactsMap_;
};
