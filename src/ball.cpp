#include "../include/ball.h"

Ball::Ball(int id, const Point2D &center, float radius, float mass, float xVel, float yVel)
    : center_{center}, radius_{radius}, mass_{mass}, xVel_{xVel}, yVel_{yVel}
{
    setID(id);
    setObjType(Utils::ObjectType::Ball);
    inertia_ = 2 * mass * radius * radius / 5;
}