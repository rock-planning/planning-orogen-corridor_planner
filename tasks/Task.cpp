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
    TaskBase::configureHook();

    if (!_map_path.get().empty())
        mEnv = envire::Environment::unserialize(_map_path.get());
    else
        mEnv = new envire::Environment;

    return true;
}

bool Task::startHook()
{
    TaskBase::startHook();

    delete planner;
    planner  = new corridor_planner::CorridorPlanner();

    planner->setMarginFactor(_margin.get());
    return true;
}

void Task::errorHook()
{
    std::vector<envire::EnvireBinaryEvent> binary_event;
    while (_map.read(binary_event, false) == RTT::NewData) 
        mEnv->applyEvents(binary_event);

    envire::Grid<uint8_t>* traversability =
        mEnv->getItem< envire::Grid<uint8_t> >(_map_id.get()).get();
    if (!traversability)
    {
        std::cout << "no traversability map with ID " << _map_id.get() << std::endl;
        return;
    }

    envire::Grid<double>::Ptr geometry;
    if (_enable_strong_edge_filter.get())
    {
        StrongEdgeFilterConfig config = _strong_edge_filter;
        geometry = mEnv->getItem< envire::Grid<double> >(config.map_id);
        if (!geometry)
            return;
    }

    planner->init(_terrain_classes.get(), *traversability,
            _map_band.get(), _min_width.get(), _cost_cutoff.get());

    if (_enable_strong_edge_filter.get())
    {
        StrongEdgeFilterConfig config = _strong_edge_filter;
        planner->enableStrongEdgeFilter(geometry,
                config.band_name,
                config.threshold);
    }

    if (_enable_narrow_wide_filter.get())
    {
        NarrowWideFilterConfig config = _narrow_wide_filter;
        planner->enableNarrowWideFilter(config.narrow_threshold,
                config.wide_threshold);
    }

    if (_enable_known_unknown_filter.get())
    {
        KnownUnknownFilterConfig config = _known_unknown_filter;
        planner->enableKnownUnknownFilter(traversability, _map_band.get(), config.unknown_class);
    }

    Eigen::Vector3d p0 = _start_point.get();
    Eigen::Vector3d p1 = _target_point.get();
    planner->setWorldPositions(
            Eigen::Vector2d(p0.x(), p0.y()), 
            Eigen::Vector2d(p1.x(), p1.y()));

    recover();
    state(DSTAR);
    getActivity()->trigger();
}

void Task::updateHook()
{
    if (state() == RUNNING)
        error(WAIT_FOR_MAP);
    else if (state() == DSTAR)
    {
        try {
            planner->computeDStar();
            state(SKELETON);
        }
        catch(corridor_planner::CostCutoffReached const& e)
        {
            std::cout << e.what() << std::endl;
            exception(NO_SOLUTION);
        }
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
        state(ANNOTATIONS);
    }
    else if (state() == ANNOTATIONS)
    {
        planner->annotateCorridors();
        planner->done();
        _plan.write(planner->result());
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

void Task::cleanupHook()
{
    delete mEnv;
}
