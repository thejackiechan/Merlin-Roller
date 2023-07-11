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
    Vector2D v1{1, 2};
    Vector2D v2{0, 0};
    // Slope s(1, Point2D{1, 1}, Point2D{2,2});

    Vector2D v3 = LinearAlgebra::projAOntoB(v1, v2);

    // std::cout << v3 << "\n";
    // float res = LinearAlgebra::computeProjMagOfAOnB(v1, v2);
    std::cout << "x is: " << v3.x() << ", y is: " << v3.y() << "\n";

    Ball b(1, Point2D{5, 5}, 2, 1, 1, 0);
    std::shared_ptr<Ball> bPtr = std::make_shared<Ball>(b);
    std::cout << bPtr->getRadius() << "\n";
    std::cout << bPtr->getID() << "\n";
    std::cout << bPtr->getCenter().x << "\n";
    std::cout << bPtr->getCenter().y << "\n";
    std::cout << bPtr->getLinearVelocity() << "\n";
    std::cout << bPtr->getInertia() << "\n";
    bPtr->printState();

    std::vector<std::shared_ptr<Utils::WorldObject>> objVec;
    objVec.push_back(bPtr);
    Simulation sim(objVec);

    return 0;
}
