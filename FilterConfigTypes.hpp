#ifndef OROGEN_CORRIDOR_PLANNER_FILTER_CONFIG_TYPES_HPP
#define OROGEN_CORRIDOR_PLANNER_FILTER_CONFIG_TYPES_HPP

#include <boost/cstdint.hpp>

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

    enum MapSizeMode {
	/** use map size of input map */
	INPUT = 0,
	/** use map size parameters provided */
	PARAMS = 1
    };
}

#endif

