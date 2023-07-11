#include <iostream>
#include <memory>
#include <vector>
#include "../include/utils.h"
#include "../include/ball.h"
#include "../include/slope.h"
#include "../include/linalg.h"
#include "../include/simulation.h"

using Vector2D = LinearAlgebra::Vector2D;

int main(void)
{
    // Ball b1(1, Point2D{10.f, 50.9961}, 1.f, 2.f);
    // std::shared_ptr<Ball> b1Ptr = std::make_shared<Ball>(b1);

    Ball b1(1, Point2D{5.f, 11.f}, 2.f, 10.f, 2.f);
    std::shared_ptr<Ball> b1Ptr = std::make_shared<Ball>(b1);

    Slope s1(1, Point2D{0.f, 10.f}, Point2D{100.f, 10.f});
    std::shared_ptr<Slope> s1Ptr = std::make_shared<Slope>(s1);

    // Slope s1(1, Point2D{0.f, 50.f}, Point2D{100.f, 0.f});
    // std::shared_ptr<Slope> s1Ptr = std::make_shared<Slope>(s1);

    // Slope s2(2, Point2D{0.f, 0.f}, Point2D{100.f, 50.f});
    // std::shared_ptr<Slope> s2Ptr = std::make_shared<Slope>(s2);

    std::vector<std::shared_ptr<Utils::WorldObject>> objVec;
    objVec.push_back(b1Ptr);
    objVec.push_back(s1Ptr);
    // objVec.push_back(s2Ptr);
    Simulation sim(objVec);
    sim.runSimulation();
    sim.printSlopeImpactTimes(b1);

    return 0;
}
