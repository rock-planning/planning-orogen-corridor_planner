#include "Task.hpp"
#include <corridor_planner/corridor_planner.hh>

#include <boost/lexical_cast.hpp>

#include <Eigen/LU>

using namespace corridor_planner;

Task::Task(std::string const& name)
    : TaskBase(name)
    , planner(0) { }

bool Task::configureHook()
{
    delete planner;
    planner  = new corridor_planner::CorridorPlanner();
    planner->init(_terrain_classes.get(), _map.get(), _min_width.get());
    return true;
}

bool Task::startHook()
{
    Eigen::Vector3d p0 = _start_point.get();
    Eigen::Vector3d p1 = _target_point.get();
    planner->setMarginFactor(_margin.get());
    planner->setWorldPositions(
            Eigen::Vector2d(p0.x(), p0.y()), 
            Eigen::Vector2d(p1.x(), p1.y()));

    return true;
}

template<typename SrcContainer, typename DstContainer>
static void wrapContainer(DstContainer& dest, SrcContainer& src,
        double scale, Eigen::Transform3d const& raster_to_world)
{
    dest.resize(src.size());

    size_t point_idx;
    typename SrcContainer::iterator src_it = src.begin();
    typename SrcContainer::iterator const src_end = src.end();
    for (point_idx = 0, src_it = src.begin(); src_it != src.end(); ++point_idx, ++src_it)
    {
        toWrapper(dest[point_idx], *src_it, scale, raster_to_world);
    }
}

template<typename SrcContainer>
static void wrapContainer(std::vector< wrappers::Vector3 >& dest, SrcContainer const& src,
        double scale, Eigen::Transform3d const& raster_to_world)
{
    dest.reserve(src.size());

    size_t point_idx;
    typename SrcContainer::const_iterator src_it = src.begin();
    typename SrcContainer::const_iterator const src_end = src.end();
    for (point_idx = 0, src_it = src.begin(); src_it != src.end(); ++point_idx, ++src_it)
    {
        Eigen::Vector3d p(raster_to_world * src_it->toEigen());
        dest.push_back(p);
    }
}


template<int DIM, typename Transform>
static void toWrapper(corridors::Curve& dest, base::geometry::Spline<DIM> const& src,
        Transform const& raster_to_world)
{
    base::geometry::Spline<DIM> world_curve(src);
    world_curve.transform(raster_to_world);
    dest = world_curve;
}

static void toWrapper(corridors::Corridor& dest, corridor_planner::Corridor& src,
        double scale, Eigen::Transform3d const& raster_to_world)
{
    src.updateCurves();
    toWrapper(dest.median_curve, src.median_curve, raster_to_world);
    toWrapper(dest.boundary_curves[0], src.boundary_curves[0], raster_to_world);
    toWrapper(dest.boundary_curves[1], src.boundary_curves[1], raster_to_world);

    dest.min_width = src.min_width;
    dest.max_width = src.max_width;
    toWrapper(dest.width_curve, src.width_curve, scale);
}

static void toWrapper(corridors::Plan& dest, corridor_planner::Plan& src,
        double scale, Eigen::Transform3d const& raster_to_world)
{
    wrapContainer(dest.corridors, src.corridors, scale, raster_to_world);

    for (unsigned int corridor_idx = 0; corridor_idx < src.corridors.size(); ++corridor_idx)
    {
        corridor_planner::Corridor const& corridor = src.corridors[corridor_idx];
        corridor_planner::Corridor::const_connection_iterator
            conn_it = corridor.connections.begin(),
            conn_end = corridor.connections.end();

        for (; conn_it != conn_end; ++conn_it)
        {
            corridors::CorridorConnection conn = 
                { corridor_idx, conn_it->this_side ? corridors::BACK_SIDE : corridors::FRONT_SIDE,
                  conn_it->target_idx, conn_it->target_side ? corridors::BACK_SIDE : corridors::FRONT_SIDE };

            dest.connections.push_back(conn);
        }
    }

    dest.cell_size      = scale;
    dest.start_corridor = src.findStartCorridor();
    dest.end_corridor   = src.findEndCorridor();
}

void Task::updateHook()
{
    if (state() == RUNNING)
        state(DSTAR);

    if (state() == DSTAR)
    {
        planner->computeDStar();
        state(SKELETON);
    }
    else if (state() == SKELETON)
    {
        planner->extractSkeleton();
        state(PLAN);
    }
    else if (state() == PLAN)
    {
        planner->computePlan();
        state(PLAN_SIMPLIFICATION);
    }
    else if (state() == PLAN_SIMPLIFICATION)
    {
        planner->simplifyPlan();

        // Finished, send to the output ports and stop the task
        corridors::Plan result;
        Eigen::Transform3d raster_to_world(planner->map->getLocalToWorld());
        toWrapper(result, planner->plan, planner->map->getScale(), raster_to_world);
        _plan.write(result);
        stop();
        return;
    }

    getActivity()->trigger();
}

// void Task::errorHook()
// {
// }

// void Task::stopHook()
// {
//     delete map;
//     map = 0;
// }

// void Task::cleanupHook()
// {
// }
