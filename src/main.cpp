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
    // Ball above a ramp with negative slope. Ball rolls down to
    // the right and hops onto the next slope.
    // Ball 1 Parameters
    int b1_id{1};
    float b1_r{2.f};
    float b1_m{2.f};
    Point2D b1_c{10.f, 50.9961};

    // Slope 1 Parameters
    int s1_id = b1_id;
    Point2D s1_start{0.f, 50.f};
    Point2D s1_end{50.f, 25.f};

    Ball b1(b1_id, b1_c, b1_r, b1_m);
    std::shared_ptr<Ball> b1Ptr = std::make_shared<Ball>(b1);

    Slope s1(s1_id, s1_start, s1_end);
    std::shared_ptr<Slope> s1Ptr = std::make_shared<Slope>(s1);

    // Ball rolling to the right on a flat surface.
    // Slope 2 Parameters
    int s2_id{2};
    Point2D s2_start{50.f, 10.f};
    Point2D s2_end{100.f, s2_start.y};

    // Ball 2 Parameters
    int b2_id = s2_id;
    float b2_x{5.f};
    float b2_r{1.f};
    float b2_m{10.f};
    float x2_xVel{3.f};
    Point2D b2_c{s2_start.x + b2_x, s2_start.y + b2_r};

    Ball b2(b2_id, b2_c, b2_r, b2_m, x2_xVel);
    std::shared_ptr<Ball> b2Ptr = std::make_shared<Ball>(b2);

    Slope s2(s2_id, s2_start, s2_end);
    std::shared_ptr<Slope> s2Ptr = std::make_shared<Slope>(s2);

    std::vector<std::shared_ptr<Utils::WorldObject>> objVec;
    objVec.push_back(b1Ptr);
    objVec.push_back(s1Ptr);
    objVec.push_back(b2Ptr);
    objVec.push_back(s2Ptr);

    // Simulation Parameters
    float dt = 1.0 / 50;
    float duration{10.0};

    Simulation sim(objVec, dt, duration);
    std::cout << "\n";
    std::cout << "Simulation complete! Printing first and last slope impact times for each ball. \n";
    sim.printSlopeImpactTimes(b1);
    sim.printSlopeImpactTimes(b2);
    return 0;
}
