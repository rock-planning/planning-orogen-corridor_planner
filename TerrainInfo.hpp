#ifndef OROGEN_CORRIDOR_PLANNER_TERRAIN_INFO_HPP
#define OROGEN_CORRIDOR_PLANNER_TERRAIN_INFO_HPP

#include <vector>
#include <string>

namespace corridor_planner
{
    struct TerrainType
    {
        /** The terrain name (used only for debugging / human-readable things)
         */
        std::string name;

        /** The R, G and B values that encode this terrain on the MLS */
        int rgb[3];

        /** The maximum shred force for this terrain type */
        double max_force;
    };

    typedef std::vector<TerrainType> TerrainInfo;
}

#endif

