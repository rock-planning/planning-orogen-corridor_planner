/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Traversability.hpp"

using namespace corridor_planner;

Traversability::Traversability(std::string const& name)
    : TraversabilityBase(name)
{
}

Traversability::Traversability(std::string const& name, RTT::ExecutionEngine* engine)
    : TraversabilityBase(name, engine)
{
}

Traversability::~Traversability()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Traversability.hpp for more detailed
// documentation about them.

// bool Traversability::configureHook()
// {
//     if (! TraversabilityBase::configureHook())
//         return false;
//     return true;
// }
// bool Traversability::startHook()
// {
//     if (! TraversabilityBase::startHook())
//         return false;
//     return true;
// }
// void Traversability::updateHook()
// {
//     TraversabilityBase::updateHook();
// }
// void Traversability::errorHook()
// {
//     TraversabilityBase::errorHook();
// }
// void Traversability::stopHook()
// {
//     TraversabilityBase::stopHook();
// }
// void Traversability::cleanupHook()
// {
//     TraversabilityBase::cleanupHook();
// }

