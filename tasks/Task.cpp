#include "Task.hpp"
#include <corridor_planner.hh>

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

// void Task::cleanupHook()
// {
// }
