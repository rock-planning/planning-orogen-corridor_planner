#ifndef NAV_TYPES_H
#define NAV_TYPES_H

#include <base/wrappers/eigen.h>
#include <base/wrappers/geometry/NURBSCurve3D.h>

namespace nav
{
    class VoronoiPoint;
    class Corridor;
    class Plan;
};

namespace corridor_planner
{
    typedef wrappers::geometry::NURBSCurve3D Curve;
    typedef std::vector<wrappers::Vector3> PointVector;

    struct VoronoiPoint
    {
        double            width;
        wrappers::Vector3 center;
        PointVector       borders[2];
    };

    struct Corridor
    {
        std::vector<VoronoiPoint> voronoi;
        PointVector boundaries[2];

        Curve median_curve;
        Curve boundary_curves[2];
    };

    enum CORRIDOR_SIDE
    { FRONT_SIDE, BACK_SIDE };

    struct CorridorConnection
    {
        int from_idx;
        CORRIDOR_SIDE from_side;
        int to_idx;
        CORRIDOR_SIDE to_side;
    };

    struct Plan
    {
        std::vector<Corridor> corridors;
        std::vector<CorridorConnection> connections;
    };
}

#endif

