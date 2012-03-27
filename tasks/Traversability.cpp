/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Traversability.hpp"

#include <envire/operators/SimpleTraversability.hpp>
#include <envire/operators/MLSSlope.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <envire/operators/ClassGridProjection.hpp>
#include <envire/Orocos.hpp>

using namespace corridor_planner;

Traversability::Traversability(std::string const& name)
    : TraversabilityBase(name)
    , mEnv(0), seq_number(0)
{
}

Traversability::Traversability(std::string const& name, RTT::ExecutionEngine* engine)
    : TraversabilityBase(name, engine)
    , mEnv(0), seq_number(0)
{
}

Traversability::~Traversability()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Traversability.hpp for more detailed
// documentation about them.

bool Traversability::configureHook()
{
    if (! TraversabilityBase::configureHook())
        return false;


    return true;
}

bool Traversability::startHook()
{
    if (! TraversabilityBase::startHook())
        return false;

    delete mEnv;
    mEnv = new envire::Environment;
    mEnv->setEnvironmentPrefix(_env_name.get());
    return true;
}

void Traversability::updateHook()
{
    TraversabilityBase::updateHook();

    // Read map data. Don't do anything until we get a new map
    std::vector<envire::EnvireBinaryEvent> binary_events;
    while (_mls_map.read(binary_events) == RTT::NewData) 
        mEnv->applyEvents(binary_events);

    envire::MLSGrid* mls = mEnv->getItem< envire::MLSGrid >(_mls_id.get()).get();
    if (! mls)
        return;

    envire::FrameNode* frame_node = mls->getFrameNode();

    size_t xSize = mls->getCellSizeX(), ySize = mls->getCellSizeY();

    // Create the slope and max step grids
    envire::Grid<double>* mls_geometry =
        new envire::Grid<double>(xSize, ySize, mls->getScaleX(), mls->getScaleY(),
                0, 0, "mls_geometry");
    mEnv->attachItem(mls_geometry, frame_node);
    envire::MLSSlope* op_mls_slope = new envire::MLSSlope;
    mEnv->attachItem(op_mls_slope);
    op_mls_slope->setInput(mls);
    op_mls_slope->setOutput(mls_geometry);

    // And convert to traversability
    envire::Grid<uint8_t>* traversability =
        new envire::Grid<uint8_t>(xSize, ySize, mls->getScaleX(), mls->getScaleY(),
                0, 0, "map");
    mEnv->attachItem(traversability, frame_node);
    envire::SimpleTraversability* op_trav = new envire::SimpleTraversability(_traversability_conf);
    mEnv->attachItem(op_trav);
    op_trav->setSlope(mls_geometry, "mean_slope");
    op_trav->setMaxStep(mls_geometry, "corrected_max_step");
    op_trav->setOutput(traversability, "");

    if (!_terrain_info.get().empty())
    {
        throw std::runtime_error("terrain type handling is not implemented yet");

        // // Generate the terrain type map
        // envire::Grid<uint16_t>* mls_terrains =
        //     new envire::Grid<uint16_t>(xSize, ySize,
        //             mls->getScaleX(), mls->getScaleY(),
        //             0, 0, "mls_terrains");
        // mEnv->attachItem(mls_terrains);
        // fillTerrainMap(*mls, *mls_terrains);

        // // Convert it to a max force map
        // envire::Grid<double>* max_force =
        //     new envire::Grid<double>(xSize, ySize,
        //             mls->getScaleX(), mls->getScaleY(),
        //             0, 0, "max_force");
        // mEnv->attachItem(max_force);
        // envire::ClassGridProjection<uint16_t, double>* op_max_force =
        //     new envire::ClassGridProjection<uint16_t, double>();
        // mEnv->attachItem(op_max_force);
        // op_max_force->class_map[0] = 0.1;
        // op_max_force->setInput(mls_terrains);
        // op_max_force->setOutput(max_force);

        // // And provide it to the traversability operator
        // op_trav->setMaxForce(max_force, "");
    }

    mEnv->updateOperators();

    if (!_env_save_path.get().empty())
    {
        std::string path = _env_save_path.get();
        path += "/" + boost::lexical_cast<std::string>(++seq_number);
        mEnv->serialize(path);
    }

    // Detach the result, add it to a new environment and dump that environment
    // on the port
    {
        envire::EnvironmentItem::Ptr map = mEnv->detachItem(traversability);
        envire::EnvironmentItem::Ptr node = mEnv->detachItem(frame_node);
        delete mEnv;
        mEnv = new envire::Environment;
        mEnv->setEnvironmentPrefix(_env_name.get());
        // These two have been kept alive by storing the return value of
        // detachItem
        mEnv->attachItem(traversability, frame_node);
    }

    // Do the export
    envire::OrocosEmitter emitter(mEnv, _traversability_map);
    emitter.flush();

    // Finally, reinitialise the environment for the next update
    delete mEnv;
    mEnv = new envire::Environment;
    mEnv->setEnvironmentPrefix(_env_name.get());
}

// void Traversability::errorHook()
// {
//     TraversabilityBase::errorHook();
// }
void Traversability::stopHook()
{
    TraversabilityBase::stopHook();

    delete mEnv;
    mEnv = 0;
}
// void Traversability::cleanupHook()
// {
//     TraversabilityBase::cleanupHook();
// }

