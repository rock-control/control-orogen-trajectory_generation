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
    cycle_time = _cycle_time.get();

    override_input_position = _override_input_position.value();
    override_input_speed = _override_input_speed.value();
    override_input_effort = _override_input_effort.value();

    override_output_speed = _override_output_speed.value();
    override_output_effort = _override_output_effort.value();

    command.resize( limits.size() );
    command.names = limits.names;
    desired_reflexes.resize(limits.size());
    desired_reflexes.names = limits.names;


    const int NUMBER_OF_DOFS = limits.size();
    if( NUMBER_OF_DOFS == 0 )
    {
        LOG_ERROR_S << "No joint limits have been configured" << std::endl;
        return false;
    }

    RML = new ReflexxesAPI( NUMBER_OF_DOFS, cycle_time );
    IP  = new RMLPositionInputParameters( NUMBER_OF_DOFS );
    OP  = new RMLPositionOutputParameters( NUMBER_OF_DOFS );
    Flags.SynchronizationBehavior   =   RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    trajectory.clear();
    current_step = 0;
    update_target = 0;

    first_it = true;
    diff_sum = 0;
    sample_ctn = 0;

    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    while( _trajectory_target.read( trajectory, false ) == RTT::NewData )
    {
        // got a new trajectory reading, reset the current index
        LOG_INFO("Got new trajectory input");
        current_step = 0;
        update_target = true;
        state(FOLLOWING);
        first_it=true;
    }
    while( _position_target.read( position_target, false ) == RTT::NewData )
    {
        LOG_INFO("Got joint position target");
        trajectory.names = position_target.names;
        trajectory.elements.resize(1);
        trajectory.elements[0].resize(trajectory.names.size());
        for(uint i=0; i<position_target.names.size(); i++){
            trajectory.elements[0][i] = position_target.elements[i];
        }
        current_step = 0;
        update_target = true;
        state(FOLLOWING);
        first_it=true;
    }

    if( _joint_state.read( status, false ) == RTT::NewData
            && current_step < trajectory.getTimeSteps() )
    {
        LOG_DEBUG("Got new joint sample input");
        for(uint i=0; i<status.size(); i++){
            if(base::isInfinity(status[i].position) || base::isNaN(status[i].position)){
                LOG_ERROR("Got invalid joint state sample. Position of joint %s is invalid.", status.names[i].c_str());
                return;
            }
        }

        if( update_target )
        {
            LOG_DEBUG("status.size %d, limits.size: %d, trajectory.size: %d", status.size(), limits.size(), trajectory.size());
            std::string jname;
            for(size_t i=0; i<limits.names.size(); i++){
                IP->SelectionVector->VecData[i] = false;
            }
            for( size_t i=0; i<trajectory.names.size(); i++ )
            {
                jname = trajectory.names[i];
                int full_state_index = limits.mapNameToIndex(jname);

                if(first_it){
                    desired_reflexes[full_state_index].position = status[full_state_index].position;
                    desired_reflexes[full_state_index].speed = 0.;
                    desired_reflexes[full_state_index].effort = status[full_state_index].effort;
                }

                //Current system state
                if(override_input_position){
                    LOG_DEBUG("Overriding position input for joint %d with %.4f", full_state_index, desired_reflexes[full_state_index].position);
                    IP->CurrentPositionVector->VecData[full_state_index] = desired_reflexes[full_state_index].position;
                }
                else{
                    IP->CurrentPositionVector->VecData[full_state_index] = status[full_state_index].position;
                }
                if(override_input_speed){
                    LOG_DEBUG("Overriding speed input for joint %d with %.4f", full_state_index, desired_reflexes[full_state_index].speed);
                    IP->CurrentVelocityVector->VecData[full_state_index] = desired_reflexes[full_state_index].speed;
                }
                else{
                    IP->CurrentVelocityVector->VecData[full_state_index] = status[full_state_index].speed;
                }
                if(override_input_effort){
                    LOG_DEBUG("Overriding acceleration input for joint %d with %.4f", full_state_index, desired_reflexes[full_state_index].effort);
                    IP->CurrentAccelerationVector->VecData[full_state_index] = desired_reflexes[full_state_index].effort;
                }
                else{
                    IP->CurrentAccelerationVector->VecData[full_state_index] = status[full_state_index].effort;
                }

                //Constraints
                IP->MaxVelocityVector->VecData[full_state_index] = limits[full_state_index].max.speed;
                IP->MaxAccelerationVector->VecData[full_state_index] = limits[full_state_index].max.effort;
                IP->MaxJerkVector->VecData[full_state_index] = 1.0; //TODO have no idea what to put here

                //Target system sate
                IP->TargetPositionVector->VecData[full_state_index] = trajectory[current_step][i].position;
                IP->TargetVelocityVector->VecData[full_state_index] = 0.1;//trajectory[current_step][i].speed;
                IP->SelectionVector->VecData[full_state_index] = true;
            }
            first_it=false;
        }

        int result = RML->RMLPosition( *IP, OP, Flags );
        switch(result){
        case ReflexxesAPI::RML_WORKING:
            // fill in output structure
            for( size_t i=0; i<command.size(); ++i )
            {
                desired_reflexes[i].position = OP->NewPositionVector->VecData[i];
                desired_reflexes[i].speed = OP->NewVelocityVector->VecData[i];
                desired_reflexes[i].effort = OP->NewAccelerationVector->VecData[i];

                command[i].position = OP->NewPositionVector->VecData[i];

                if(base::isUnset(override_output_speed)){
                    command[i].speed =  OP->NewVelocityVector->VecData[i];
                }
                else{
                    LOG_DEBUG("Overriding speed output for joint %d with %.4f", i, override_output_speed);
                    command[i].speed =  override_output_speed;
                }

                if(base::isUnset(override_output_effort)){
                    command[i].effort =  OP->NewAccelerationVector->VecData[i];
                }
                else{
                    LOG_DEBUG("Overriding effort output for joint %d with %.4f", i, override_output_effort);
                    command[i].effort =  override_output_effort;
                }

                command.time = base::Time::now();
            }
            break;
        case ReflexxesAPI::RML_FINAL_STATE_REACHED:
            LOG_WARN("Waypoint %d/%d reached", current_step, trajectory.getTimeSteps());
            current_step++;
            update_target = true;
            if(current_step >= trajectory.getTimeSteps())
                state(REACHED);
            break;
        case ReflexxesAPI::RML_ERROR:
            LOG_ERROR("Should not happen");
            break;
        case ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES:
            LOG_ERROR("RML_ERROR_INVALID_INPUT_VALUES");
            IP->Echo();
            break;
        case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION:
            LOG_ERROR("RML_ERROR_EXECUTION_TIME_CALCULATION");
            break;
        case ReflexxesAPI::RML_ERROR_SYNCHRONIZATION:
            LOG_ERROR("RML_ERROR_SYNCHRONIZATION");
            break;
        case ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS:
            LOG_ERROR("RML_ERROR_NUMBER_OF_DOFS");
            break;
        case ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION:
            LOG_ERROR("RML_ERROR_NO_PHASE_SYNCHRONIZATION");
            break;
        case ReflexxesAPI::RML_ERROR_NULL_POINTER:
            LOG_ERROR("RML_ERROR_NULL_POINTER");
            break;
        case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG:
            LOG_ERROR("RML_ERROR_EXECUTION_TIME_TOO_BIG");
            break;
        case ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE:
            LOG_ERROR("RML_ERROR_USER_TIME_OUT_OF_RANGE");
            break;
        }

        _cmd.write( command );

        base::Time time = base::Time::now();
        base::Time diff = time-prev_time;
        diff_sum += diff.toSeconds();
        sample_ctn++;
        LOG_INFO_S << diff << " #################### "<<diff.toSeconds()<<std::endl;
        if(sample_ctn == 1./cycle_time){
            double avg_time = diff_sum/sample_ctn;
            LOG_INFO_S << "Average cycle time: " << avg_time<<std::endl;
            LOG_INFO_S << " Configured: " << cycle_time<<std::endl;
            LOG_INFO_S << " diff_sum: " << diff_sum<<std::endl;
            LOG_INFO_S << " sample_ctn: " << sample_ctn<<std::endl;
            _average_cycle_rate.write(avg_time);
            sample_ctn = 0;
            diff_sum = 0.0;
        }

        prev_time = time;
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
