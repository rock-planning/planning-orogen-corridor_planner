#ifndef OROGEN_CORRIDOR_PLANNER_FILTER_CONFIG_TYPES_HPP
#define OROGEN_CORRIDOR_PLANNER_FILTER_CONFIG_TYPES_HPP

#include <boost/cstdint.hpp>
#include <string>

namespace corridor_planner {
    struct StrongEdgeFilterConfig {
        std::string env_path;
        std::string map_id;
        std::string band_name;
        double threshold;
    };

    struct NarrowWideFilterConfig {
        double narrow_threshold;
        double wide_threshold;
    };

    struct KnownUnknownFilterConfig {
        boost::uint8_t unknown_class;
    };
}

#endif

