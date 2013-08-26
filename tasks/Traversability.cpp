/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Traversability.hpp"

#include <envire/operators/SimpleTraversability.hpp>
#include <envire/operators/MLSSlope.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <envire/maps/Grids.hpp>
#include <envire/operators/MergeMLS.hpp>
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

    mMaxExtent = _map_max_extent.get();

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
    envire::OrocosEmitter::Ptr binary_events;
    while (_mls_map.read(binary_events, false) == RTT::NewData) {
        mEnv->applyEvents(*binary_events);
        RTT::log(RTT::Info) << "Received new binary event" << RTT::endlog();
    }
    
    std::vector<envire::MLSGrid*> maps = mEnv->getItems<envire::MLSGrid>();
    // Lists all received MLS maps.
    std::stringstream ss;
    if(maps.size()) {
        ss << "Received MLS map(s): " << std::endl;
 
        std::string trav_map_id;
        std::vector<envire::MLSGrid*>::iterator it = maps.begin();
        for(int i=0; it != maps.end(); ++it, ++i)
        {
            ss << i << ": " << (*it)->getUniqueId() << std::endl;
        }
        RTT::log(RTT::Info) << ss.str() << RTT::endlog(); 
    } else {
        RTT::log(RTT::Info) << "Environment does not contain any MLS grids" << RTT::endlog();
        return;
    }

    envire::MLSGrid* mls_in = mEnv->getItem< envire::MLSGrid >(_mls_id.get()).get();
    if (! mls_in) {
        RTT::log(RTT::Warning) << "No mls found with id " <<  _mls_id.get() << RTT::endlog();
        return;
    }
    
    RTT::log(RTT::Info) << "Got a new mls map with id " << mls_in->getUniqueId() << RTT::endlog();

    envire::FrameNode* frame_node = mls_in->getFrameNode();

    // get the extents from the map, and extend it with the extents provided by
    // the parameters (if any)
    Eigen::AlignedBox<double, 2> extents = mls_in->getExtents();
    Eigen::Affine3d world2grid = mEnv->getRootNode()->relativeTransform( frame_node );
    for( std::vector<base::Vector2d>::iterator it = _map_extents.value().begin(); it != _map_extents.value().end(); it++ )
    {
	Eigen::Vector3d p;
	p << *it, 0;
    	extents.extend( (world2grid * p).head<2>() );
    }
    double xScale = mls_in->getScaleX(), yScale = mls_in->getScaleY();
    double xCellSize = mls_in->getCellSizeX(), yCellSize = mls_in->getCellSizeX();
    // Bounding the size of the mls_in map on the number of patches in each dimension
    size_t xSize = std::min(mMaxExtent, (size_t) (extents.sizes().x() / xScale));
    size_t ySize = std::min(mMaxExtent, (size_t) (extents.sizes().y() / yScale));
    double xOffset = extents.min().x(), yOffset = extents.min().y();

    RTT::log(RTT::Debug) << "Traversability: input MLS: xScale: '" << xScale << "' yScale: '" << yScale << "'" << RTT::endlog();
    RTT::log(RTT::Debug) << "Traversability: input MLS: xSize (max_extent: '" << mMaxExtent << "'): '" << xSize << "' ySize: '" << ySize << "'" << RTT::endlog();
    RTT::log(RTT::Debug) << "Traversability: input MLS: xCellSize: '" << xCellSize << "' yCellSize: '" << yCellSize << "'" << RTT::endlog();

    // see if we need to resize the input mls 
    envire::MLSGrid* mls = mls_in;
    if( xScale != mls_in->getCellSizeX() || yScale != mls_in->getCellSizeY() )
    {
	mls = new envire::MLSGrid(xSize, ySize, xScale, yScale,
		xOffset, yOffset);
	mEnv->setFrameNode( mls, mls_in->getFrameNode() );
	envire::MergeMLS* op_mls_merge = new envire::MergeMLS;
	mEnv->attachItem(op_mls_merge);
	op_mls_merge->setInput( mls_in );
	op_mls_merge->setOutput( mls );
	op_mls_merge->updateAll();
    }

    RTT::log(RTT::Debug) << "Traversability: create the slope and max step grids" << RTT::endlog();

    // Create the slope and max step grids
    envire::Grid<float>* mls_geometry =
        new envire::Grid<float>(xSize, ySize, xScale, yScale,
                xOffset, yOffset, "mls_geometry");
    mEnv->attachItem(mls_geometry, frame_node);
    envire::MLSSlope* op_mls_slope = new envire::MLSSlope;
    mEnv->attachItem(op_mls_slope);
    op_mls_slope->setInput(mls);
    op_mls_slope->setOutput(mls_geometry);

    RTT::log(RTT::Debug) << "Traversability: convert map to traversability map" << RTT::endlog();
    // And convert to traversability
    envire::TraversabilityGrid* traversability =
        new envire::TraversabilityGrid(xSize, ySize, xScale, yScale,
                xOffset, yOffset, "map");
    mEnv->attachItem(traversability, frame_node);
    envire::SimpleTraversability* op_trav = new envire::SimpleTraversability(_traversability_conf);
    mEnv->attachItem(op_trav);
    op_trav->setSlope(mls_geometry, "mean_slope");
    op_trav->setMaxStep(mls_geometry, "corrected_max_step");
    op_trav->setOutput(traversability, envire::TraversabilityGrid::TRAVERSABILITY);

    RTT::log(RTT::Debug) << "Traversability: retrieve terrain info" << RTT::endlog();
    if (!_terrain_info.get().empty())
    {
        throw std::runtime_error("terrain type handling is not implemented yet");

        // // Generate the terrain type map
        // envire::Grid<uint16_t>* mls_terrains =
        //     new envire::Grid<uint16_t>(xSize, ySize,
        //             xScale, yScale,
        //             0, 0, "mls_terrains");
        // mEnv->attachItem(mls_terrains);
        // fillTerrainMap(*mls, *mls_terrains);

        // // Convert it to a max force map
        // envire::Grid<float>* max_force =
        //     new envire::Grid<float>(xSize, ySize,
        //             xScale, yScale,
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

    RTT::log(RTT::Debug) << "Traversability: use env save path for serialization" << RTT::endlog();
    if (!_env_save_path.get().empty())
    {
        std::string path = _env_save_path.get();
        path += "/complete";
        mEnv->serialize(path);
    }

    RTT::log(RTT::Debug) << "Traversability: detach result" << RTT::endlog();
    // Detach the result, add it to a new environment and dump that environment
    // on the port
    {
        envire::Transform transform =
            mEnv->relativeTransform(frame_node, mEnv->getRootNode());

        envire::EnvironmentItem::Ptr map = mEnv->detachItem(traversability);
        envire::EnvironmentItem::Ptr geometry = mEnv->detachItem(mls_geometry);
        delete mEnv;
        mEnv = new envire::Environment;
        mEnv->setEnvironmentPrefix(_env_name.get());
        envire::FrameNode* frame_node = new envire::FrameNode(transform);
        mEnv->getRootNode()->addChild(frame_node);
        // These two have been kept alive by storing the return value of
        // detachItem
        mEnv->attachItem(traversability, frame_node);
        mEnv->attachItem(mls_geometry, frame_node);
        
        if (!_env_save_path.get().empty())
        {
            std::string path = _env_save_path.get();
            path += "/reduced";
            mEnv->serialize(path);
        }
    }

    RTT::log(RTT::Debug) << "Traversability: export result" << RTT::endlog();
    // Do the export. Do it in a block so that the emitter gets deleted before
    // the environment is.
    {
        envire::OrocosEmitter emitter(mEnv, _traversability_map);
        emitter.setTime((*binary_events)[0].time);
        emitter.flush();
    }

    RTT::log(RTT::Debug) << "Traversability: cleanup for next update" << RTT::endlog();
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

