#ifndef NAV_TYPES_H
#define NAV_TYPES_H

#include <base/wrappers/eigen.h>
#include <base/wrappers/geometry/spline.h>

namespace corridors
{
    typedef wrappers::geometry::Spline Curve;

    struct Corridor
    {
        double min_width, max_width;
        Curve width_curve;
        Curve median_curve;
        Curve boundary_curves[2];
    };

    /** Values used in CorridorConnection to indicate from which side is the
     * connection attached to the corresponding corridor
     */
    enum CORRIDOR_SIDE
    {
        FRONT_SIDE, //! the corridor is attached to the beginning of the corridor
        BACK_SIDE   //! the corridor is attached to the end of the corridor
    };

    struct CorridorConnection
    {
        int from_idx;
        CORRIDOR_SIDE from_side;
        int to_idx;
        CORRIDOR_SIDE to_side;
    };

    struct Plan
    {
        /** the size, in meters, of a cell in the map used to compute the
         * corridors. This is for reference only, as the data is provided in
         * meters already
         */
        double cell_size;

        /** The index of the start corridor */
        int start_corridor;
        /** The index of the end corridor */
        int end_corridor;
        /** The set of corridors */
        std::vector<Corridor> corridors;
        /** The data structures that represent the connections between
         * the corridors */
        std::vector<CorridorConnection> connections;
    };
}

#endif

