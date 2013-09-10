/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/Logging.hpp>

using namespace trajectory_generation;

Task::Task(std::string const& name)
    : TaskBase(name), current_step(0), update_target(false)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), current_step(0), update_target(false)
{
}

Task::~Task()
{
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    limits = _limits.value();

    command.resize( limits.size() );
    command.names = limits.names;

    // TODO put into configuration 
    const double CYCLE_TIME_IN_SECONDS = 0.01;
    const int NUMBER_OF_DOFS = limits.size();
    if( NUMBER_OF_DOFS == 0 )
    {
	LOG_ERROR_S << "No joint limits have been configured" << std::endl;
	return false;
    }

    RML = new ReflexxesAPI( NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS );
    IP  = new RMLPositionInputParameters( NUMBER_OF_DOFS );
    OP  = new RMLPositionOutputParameters( NUMBER_OF_DOFS );

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    trajectory.clear();
    current_step = 0;
    update_target = 0;

    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    while( _target.read( trajectory, false ) == RTT::NewData )
    {
	// got a new trajectory reading, reset the current index
	current_step = 0;
	update_target = true;
    }	

    while( _joint_state.read( status, false ) == RTT::NewData
	    && current_step < trajectory.getTimeSteps() )
    {
	if( update_target )
	{
	    update_target = false;
	    assert( status.size() == limits.size() 
		    && status.size() == trajectory.size() );

	    for( size_t i=0; i<limits.size(); i++ )
	    {
		// TODO, check if the status data is actually compatible
		IP->CurrentPositionVector->VecData[i] = status[i].position;
		IP->CurrentVelocityVector->VecData[i] = status[i].speed;
		IP->CurrentAccelerationVector->VecData[i] = status[i].effort;
		IP->MaxVelocityVector->VecData[i] = limits[i].max.speed;
		IP->MaxAccelerationVector->VecData[i] = limits[i].max.effort;
		IP->MaxJerkVector->VecData[i] = 1.0; // TODO have no idea what to put here
		IP->TargetPositionVector->VecData[i] = trajectory[i][current_step].position;
		IP->TargetVelocityVector->VecData[i] = trajectory[i][current_step].speed;
		IP->SelectionVector->VecData[i] = true;
	    }
	}

	if( RML->RMLPosition( *IP, OP, Flags ) ==
		ReflexxesAPI::RML_FINAL_STATE_REACHED )
	{
	    current_step++;
	    update_target = true;
	}

	// fill in output structure
	for( size_t i=0; i<command.size(); ++i )
	{
	    command[i].position = OP->NewPositionVector->VecData[i];
	    command[i].speed = OP->NewVelocityVector->VecData[i];
	    command[i].effort = OP->NewAccelerationVector->VecData[i];
	}

	_cmd.write( command );
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    delete RML;
    delete IP;
    delete OP;

    TaskBase::cleanupHook();
}
