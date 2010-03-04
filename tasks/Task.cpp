#include "Task.hpp"
#include <nav/corridor_planner.hh>

#include <boost/lexical_cast.hpp>

using namespace corridor_planner;

Task::Task(std::string const& name)
    : TaskBase(name)
    , planner(0) { }

bool Task::configureHook()
{
    delete planner;
    planner  = new nav::CorridorPlanner();
    planner->init(_terrain_classes.get(), _map.get());

    Eigen::Vector3d p0 = _start_point.get();
    Eigen::Vector3d p1 = _target_point.get();
    planner->setMarginFactor(_margin.get());
    planner->setPositions(
            Eigen::Vector2d(p0.x(), p0.y()), 
            Eigen::Vector2d(p1.x(), p1.y()));

    return true;
}

// bool Task::startHook()
// {
//     return true;
// }

template<typename SrcContainer, typename DstContainer>
static void wrapContainer(DstContainer& dest, SrcContainer const& src,
        double scale, Eigen::Transform3d const& world_to_local)
{
    dest.resize(src.size());

    size_t point_idx;
    typename SrcContainer::const_iterator src_it = src.begin();
    typename SrcContainer::const_iterator const src_end = src.end();
    for (point_idx = 0, src_it = src.begin(); src_it != src.end(); ++point_idx, ++src_it)
    {
        toWrapper(dest[point_idx], *src_it, scale, world_to_local);
    }
}

template<typename SrcContainer>
static void wrapContainer(std::vector< wrappers::Vector3 >& dest, SrcContainer const& src,
        double scale, Eigen::Transform3d const& world_to_local)
{
    dest.reserve(src.size());

    size_t point_idx;
    typename SrcContainer::const_iterator src_it = src.begin();
    typename SrcContainer::const_iterator const src_end = src.end();
    for (point_idx = 0, src_it = src.begin(); src_it != src.end(); ++point_idx, ++src_it)
    {
        Eigen::Vector3d p(world_to_local * src_it->toEigen());
        dest.push_back(p);
    }
}


static void toWrapper(Curve& dest, base::geometry::NURBSCurve3D const& src,
        double scale, Eigen::Transform3d const& world_to_local)
{
    dest.geometric_resolution = src.getGeometricResolution();
    dest.curve_order = src.getCurveOrder();
    std::vector<Eigen::Vector3d> const& points = src.getPoints();
    for (int i = 0; i < points.size(); ++i)
    {
        Eigen::Vector3d p = world_to_local * points[i];
        dest.points.push_back(p);
    }
}

static void toWrapper(VoronoiPoint& dest, nav::VoronoiPoint const& src, 
        double scale, Eigen::Transform3d const& world_to_local)
{
    dest.center = world_to_local * src.center.toEigen();
    dest.width  = scale * src.width;

    if (src.borders.size() != 2)
        throw std::logic_error("got point with " + boost::lexical_cast<std::string>(src.borders.size()) + " borders");

    wrapContainer(dest.borders[0], src.borders.front(), scale, world_to_local);
    wrapContainer(dest.borders[1], src.borders.back(), scale, world_to_local);
}

static void toWrapper(Corridor& dest, nav::Corridor const& src,
        double scale, Eigen::Transform3d const& world_to_local)
{
    wrapContainer(dest.voronoi, src.voronoi, scale, world_to_local);

    dest.voronoi.resize(src.voronoi.size());
    int i;
    std::list< nav::VoronoiPoint>::const_iterator voronoi_it;
    for (i = 0, voronoi_it = src.voronoi.begin(); voronoi_it != src.voronoi.end(); ++i, ++voronoi_it)
        toWrapper(dest.voronoi[i], *voronoi_it, scale, world_to_local);

    wrapContainer(dest.boundaries[0], src.boundaries[0], scale, world_to_local);
    wrapContainer(dest.boundaries[1], src.boundaries[1], scale, world_to_local);

    toWrapper(dest.median_curve, src.median_curve, scale, world_to_local);
    toWrapper(dest.boundary_curves[0], src.boundary_curves[0], scale, world_to_local);
    toWrapper(dest.boundary_curves[1], src.boundary_curves[1], scale, world_to_local);
}

static void toWrapper(Plan& dest, nav::Plan const& src,
        double scale, Eigen::Transform3d const& world_to_local)
{
    wrapContainer(dest.corridors, src.corridors, scale, world_to_local);

    for (int corridor_idx = 0; corridor_idx < src.corridors.size(); ++corridor_idx)
    {
        nav::Corridor const& corridor = src.corridors[corridor_idx];
        nav::Corridor::const_connection_iterator
            conn_it = corridor.connections.begin(),
            conn_end = corridor.connections.end();

        for (; conn_it != conn_end; ++conn_it)
        {
            CorridorConnection conn = 
                { corridor_idx, conn_it->this_side ? BACK_SIDE : FRONT_SIDE,
                  conn_it->target_idx, conn_it->target_side ? BACK_SIDE : FRONT_SIDE };

            dest.connections.push_back(conn);
        }
    }
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
        Plan result;
        toWrapper(result, planner->plan, planner->map->getScale(),
                planner->map->getLocalToWorld());
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
