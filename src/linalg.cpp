#include "../include/linalg.h"
#include <limits>

using Vector2D = LinearAlgebra::Vector2D;

float LinearAlgebra::computeProjMagOfAOnB(const Vector2D &A, const Vector2D &B)
{
    float bMag = B.getMagnitude();
    if (bMag > 0)
    {
        return A.dot(B) / bMag;
    }
    return std::numeric_limits<float>::quiet_NaN();
}

Vector2D LinearAlgebra::projAOntoB(const Vector2D &A, const Vector2D &B)
{
    float projMagOfAOnB = computeProjMagOfAOnB(A, B);
    if (!std::isnan(projMagOfAOnB))
    {
        return B * (projMagOfAOnB / B.getMagnitude());
    }
    return Vector2D{projMagOfAOnB, projMagOfAOnB};
}