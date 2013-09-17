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

    const double CYCLE_TIME_IN_SECONDS = _cycle_time.get();

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
        LOG_DEBUG("Got new trajectory input");
        current_step = 0;
        update_target = true;
        state(FOLLOWING);
    }

    while( _joint_state.read( status, false ) == RTT::NewData
           && current_step < trajectory.getTimeSteps() )
    {
        LOG_DEBUG("Got new joint sample input");
        if( update_target )
        {
            LOG_DEBUG("status.size %d, limits.size: %d, trajectory.size: %d", status.size(), limits.size(), trajectory.size());
            //assert( status.size() == limits.size()
            //        && status.size() == trajectory.size() );
            std::string jname;
            for( size_t i=0; i<trajectory.size(); i++ )
            {
                jname = trajectory.names[i];
                int full_state_index = limits.mapNameToIndex(jname);
                // TODO, check if the status data is actually compatible
                IP->CurrentPositionVector->VecData[full_state_index] = status[full_state_index].position;
                IP->CurrentVelocityVector->VecData[full_state_index] = status[full_state_index].speed;
                IP->CurrentAccelerationVector->VecData[full_state_index] = status[full_state_index].effort;
                IP->MaxVelocityVector->VecData[full_state_index] = limits[full_state_index].max.speed;
                IP->MaxAccelerationVector->VecData[full_state_index] = limits[full_state_index].max.effort;
                IP->MaxJerkVector->VecData[full_state_index] = 1.0; //TODO have no idea what to put here
                IP->TargetPositionVector->VecData[full_state_index] = trajectory[i][current_step].position;
                IP->TargetVelocityVector->VecData[full_state_index] = trajectory[i][current_step].speed;
                IP->SelectionVector->VecData[full_state_index] = true;
            }
        }

        if( RML->RMLPosition( *IP, OP, Flags ) ==
                ReflexxesAPI::RML_FINAL_STATE_REACHED )
        {
            LOG_DEBUG("Waypoint %d/%d reached", current_step, trajectory.getTimeSteps());
            current_step++;
            update_target = true;
            if(current_step >= trajectory.getTimeSteps())
                state(REACHED);
        }

        // fill in output structure
        for( size_t i=0; i<command.size(); ++i )
        {
            command[i].position = OP->NewPositionVector->VecData[i];
            command[i].speed = OP->NewVelocityVector->VecData[i];
            command[i].effort = OP->NewAccelerationVector->VecData[i];
            command.time = base::Time::now();
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
