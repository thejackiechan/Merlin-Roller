#pragma once

#include <cmath>

namespace Utils
{

    enum class ObjectType
    {
        Slope,
        Ball
    };

    struct Point2D
    {
        float x; // m
        float y; // m

        Point2D() {}
        Point2D(float inX, float inY) : x{inX}, y{inY} {}

        Point2D operator+(const Point2D &rhs) const
        {
            return Point2D{x + rhs.x, y + rhs.y};
        }

        Point2D operator-(const Point2D &rhs) const
        {
            return Point2D{x - rhs.x, y - rhs.y};
        }
    };

    struct SlopeImpact
    {
        int slopeID;
        float firstTime; // s
        float lastTime;  // s

        SlopeImpact(int inSlopeID, float inFirstTime, float inLastTime) : slopeID{inSlopeID}, firstTime{inFirstTime}, lastTime{inLastTime} {}
    };

    class WorldObject
    {
    public:
        inline void setID(const int &id)
        {
            id_ = id;
        }

        inline const int getID() const
        {
            return id_;
        }

        inline const ObjectType getObjType() const
        {
            return type_;
        }

        void setObjType(ObjectType type)
        {
            type_ = type;
        }

        virtual void printState() const = 0;

    private:
        int id_;
        ObjectType type_;
    };

    // Utility functions
    inline float computeMagnitude(float x, float y)
    {
        return std::hypotf(x, y);
    }

} // namespace Utils
