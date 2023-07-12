#pragma once

#include "utils.h"

using Utils::Point2D;

namespace LinearAlgebra
{
    class Vector2D
    {
    public:
        Vector2D() {}

        Vector2D(float x, float y) : p_{Point2D{x, y}} {}

        Vector2D(const Point2D &p2, const Point2D &p1 = Point2D{0.0, 0.0})
        {
            p_ = p2 - p1;
        }

        Vector2D(const Vector2D &v)
        {
            p_.x = v.x();
            p_.y = v.y();
        }

        Vector2D operator+(const Vector2D &rhs) const
        {
            return Vector2D{p_.x + rhs.x(), p_.y + rhs.y()};
        }

        Vector2D operator-(const Vector2D &rhs) const
        {
            return Vector2D{p_.x - rhs.x(), p_.y - rhs.y()};
        }

        Vector2D operator*(const float val) const
        {
            return Vector2D{p_.x * val, p_.y * val};
        }

        Vector2D operator/(const float val) const
        {
            return Vector2D{p_.x / val, p_.y / val};
        }

        inline const float dot(const Vector2D &rhs) const
        {
            return p_.x * rhs.x() + p_.y * rhs.y();
        }

        inline const float cross(const Vector2D &rhs) const
        {
            return p_.x * rhs.y() - p_.y * rhs.x();
        }

        inline const float getMagnitude() const
        {
            return Utils::computeMagnitude(p_.x, p_.y);
        }

        inline const float x() const
        {
            return p_.x;
        }

        inline const float y() const
        {
            return p_.y;
        }

        const Vector2D getUnitVec() const
        {
            if (getMagnitude() == 0)
                return Vector2D{0.f, 0.f};
            Vector2D v{p_};
            return v / getMagnitude();
        }

    private:
        Point2D p_;
    };
    // Linear Algebra Functions
    float computeProjMagOfAOnB(const Vector2D &A, const Vector2D &B);

    Vector2D projAOntoB(const Vector2D &A, const Vector2D &B);

} // namespace LinearAlgebra