#include "Task.hpp"
#include <nav/corridor_planner.hh>

#include <boost/lexical_cast.hpp>

#include <Eigen/LU>

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
    planner->setWorldPositions(
            Eigen::Vector2d(p0.x(), p0.y()), 
            Eigen::Vector2d(p1.x(), p1.y()));

    return true;
}

// bool Task::startHook()
// {
//     return true;
// }

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


static void toWrapper(Curve& dest, base::geometry::Spline<3> const& src,
        double scale, Eigen::Transform3d const& raster_to_world)
{
    base::geometry::Spline<3> world_curve(src);
    world_curve.transform(raster_to_world);
    dest = world_curve;
}

static void toWrapper(Corridor& dest, nav::Corridor& src,
        double scale, Eigen::Transform3d const& raster_to_world)
{
    src.updateCurves();
    toWrapper(dest.median_curve, src.median_curve, scale, raster_to_world);
    toWrapper(dest.boundary_curves[0], src.boundary_curves[0], scale, raster_to_world);
    toWrapper(dest.boundary_curves[1], src.boundary_curves[1], scale, raster_to_world);

    // Now compute an estimate of the "width curve"
    base::geometry::Spline<1> width;
    typedef base::geometry::Spline<1>::vector_t point_t;
    std::vector<point_t> points;
    std::vector<double> parameters;
    float delta = (src.median_curve.getEndParam() - src.median_curve.getStartParam()) / src.voronoi.size();
    for (float t = src.median_curve.getStartParam();
            t < src.median_curve.getEndParam(); t += delta)
    {
        Eigen::Vector3d p = src.median_curve.getPoint(t);
        nav::Corridor::voronoi_const_iterator median_it = src.findNearestMedian(nav::PointID(p.x(), p.y()));
        {
            point_t p;
            p(0, 0) = median_it->width;
            points.push_back(p);
        }
        parameters.push_back(t);
    }

    std::list<nav::VoronoiPoint>::const_iterator voronoi_it;
    float max_width = src.voronoi.front().width, min_width = max_width;
    for (voronoi_it = src.voronoi.begin(); voronoi_it != src.voronoi.end(); ++voronoi_it)
    {
        float this_width = voronoi_it->width;
        if (min_width > this_width)
            min_width = this_width;
        if (max_width < this_width)
            max_width = this_width;
    }
    dest.min_width = min_width;
    dest.max_width = max_width;

    width.interpolate(points, parameters);
    width.simplify(1);
    dest.width = width;
}

static void toWrapper(Plan& dest, nav::Plan& src,
        double scale, Eigen::Transform3d const& raster_to_world)
{
    wrapContainer(dest.corridors, src.corridors, scale, raster_to_world);

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
        Plan result;
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
