/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/Logging.hpp>
#include <base/NamedVector.hpp>

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

double interpolate(double a, double b, double c){
    double ba=b-a;
    double cb=c-b;
    return (ba+cb)/2.0;
}

void set_speeds(base::JointsTrajectory& traj, double target_speed){
    double cur, prev, next, speed;
    for(uint t=0; t<traj.getTimeSteps(); t++)
    {
        double max_speed = 0;
        for(uint joint_idx=0; joint_idx<traj.size(); joint_idx++)
        {
            cur = traj[joint_idx][t].position;
            if(t==0)
                prev = cur;
            else
                prev = traj[joint_idx][t-1].position;

            if(t==traj.getTimeSteps()-1)
                next = cur;
            else
                next = traj[joint_idx][t+1].position;

            speed = interpolate(prev, cur, next);
            if(fabs(speed) > max_speed){
                max_speed = fabs(speed);
            }
            LOG_INFO("Speed for joint %i in Wapoint %i to %f. Prev pos: %f, pos: %f, next pos: %f",joint_idx, t, speed, prev, cur, next);
            traj[joint_idx][t].speed = speed;
        }
        //Normalize speed to 1.0
        for(uint joint_idx=0; joint_idx<traj.size(); joint_idx++)
        {
            traj[joint_idx][t].speed = (traj[joint_idx][t].speed/max_speed) * target_speed;
            LOG_INFO("Speed for joint %i to %f. normalized to 1. Tareget speed: %f. Max Speed %f",joint_idx, traj[joint_idx][t].speed, target_speed, max_speed);
        }
        LOG_INFO("");
    }
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
    override_target_velocity = _override_target_velocity.value();

    override_output_speed = _override_output_speed.value();
    override_output_effort = _override_output_effort.value();

    command.resize( limits.size() );
    command.names = limits.names;
    desired_reflexes.resize(limits.size());
    desired_reflexes.names = limits.names;

    j_state_full.resize(limits.size());
    j_state_full.names = limits.names;

    const int NUMBER_OF_DOFS = limits.size();
    if( NUMBER_OF_DOFS == 0 )
    {
        LOG_ERROR_S << "No joint limits have been configured" << std::endl;
        return false;
    }

    for(uint i=0; i<limits.size(); i++){
        if(limits[i].max.speed < override_target_velocity){
            LOG_ERROR("override_target_velocity too high. Joint %s has a max speed of %f. override_target_velocity must be lower than that value.",limits.names[i].c_str(), limits[i].max.speed);
            return false;
        }
    }

    RML = new ReflexxesAPI( NUMBER_OF_DOFS, cycle_time );
    IP  = new RMLPositionInputParameters( NUMBER_OF_DOFS );
    OP  = new RMLPositionOutputParameters( NUMBER_OF_DOFS );
    Flags.SynchronizationBehavior = RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;
    for(uint i=0; i<limits.size(); i++){
        //Constraints
        IP->MaxVelocityVector->VecData[i] = limits[i].max.speed;
        IP->MaxAccelerationVector->VecData[i] = limits[i].max.effort;
        IP->MaxJerkVector->VecData[i] = 500.0; //TODO have no idea what to put here
    }

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

    prev_time = base::Time::now();


    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    while( _trajectory_target.read( trajectory, false ) == RTT::NewData )
    {
        // got a new trajectory reading, reset the current index
        current_step = 0;
        update_target = true;
        state(FOLLOWING);
        first_it=true;

        for(uint i=0; i<trajectory.names.size(); i++){
            try{j_state_full.mapNameToIndex(trajectory.names[i]);}
            catch(std::runtime_error){
                LOG_WARN("Joint '%s' is unknown to trajectory_generation. Check configuration", trajectory.names[i].c_str());
                continue;
            }
        }

        set_speeds(trajectory, override_target_velocity);

        LOG_DEBUG("Trajectory: ");
        for(size_t i = 0; i < trajectory.size(); i++)
            for(uint j = 0; j < trajectory.getTimeSteps(); j++)
                LOG_DEBUG("%f %f", trajectory[i][j].position, trajectory[i][j].speed);

    }
    while( _position_target.read( position_target, false ) == RTT::NewData )
    {
        trajectory.resize(limits.size(), 1);
        trajectory.names = position_target.names;

        for(size_t i = 0; i < limits.size(); i++){
            size_t idx;
            try{
                idx = position_target.mapNameToIndex(limits.names[i]);
            }
            catch(std::exception e){
                LOG_DEBUG("Joint %s has been configured in joint limits, but is not in target vector", limits.names[i].c_str());
                trajectory.elements[0][i].position = IP->CurrentPositionVector->VecData[i];
                trajectory.elements[0][i].speed = 0;
                continue;
            }

            trajectory.elements[0][i].position = position_target[idx].position;
            if(position_target[idx].hasSpeed())
                trajectory.elements[0][i].speed = std::max(std::min(position_target[idx].speed, limits[i].max.speed), limits[i].min.speed);
            else
                trajectory.elements[0][i].speed = 0.0;
        }
        current_step = 0;
        update_target = true;
        state(FOLLOWING);
        first_it=true;


    }

    base::samples::Joints j_state_new;
    if( _joint_state.read( j_state_new, false ) == RTT::NewData
            && current_step < trajectory.getTimeSteps() )
    {
        //Invalidate  j_state
        for(uint i=0; i<j_state_full.size(); i++){
            j_state_full[i] = base::JointState();
            j_state_full[i].position = base::unset<double>();
            j_state_full[i].speed = base::unset<float>();
            j_state_full[i].effort = base::unset<float>();
            j_state_full[i].raw = base::unset<float>();
        }

        for(uint i=0; i<j_state_new.size(); i++){
            //Check if data is okay
            if(base::isInfinity(j_state_new[i].position) || base::isNaN(j_state_new[i].position)){
                LOG_WARN("Got invalid joint state sample. Position of joint %s is invalid.", j_state_new.names[i].c_str());
                continue;
            }
            if(base::isInfinity(j_state_new[i].speed) || base::isNaN(j_state_new[i].speed)){
                LOG_WARN("Got invalid joint state sample. Speed of joint %s is invalid.", j_state_new.names[i].c_str());
                continue;
            }

            //Is the joint known to trajectory generation
            std::string j_name = j_state_new.names[i];
            int j_idx_full=0;
            try{j_idx_full = j_state_full.mapNameToIndex(j_name);}
            catch(std::runtime_error){
                LOG_DEBUG("Got joint sample of joint '%s', which is unknown to trajectory generation. Check configuration!", j_name.c_str());
                continue;
            }

            //Update state for joint
            j_state_full[j_idx_full] = j_state_new[i];
        }

        if( update_target )
        {
            std::string j_name;
            for(size_t i=0; i<limits.names.size(); i++){
                IP->SelectionVector->VecData[i] = false;
            }
            for( size_t i=0; i<trajectory.names.size(); i++ )
            {
                int j_idx_full=0;
                j_name = trajectory.names[i];
                try{j_idx_full = limits.mapNameToIndex(j_name);}
                catch(std::runtime_error){
                    LOG_DEBUG("Skipping unknown joint '%s' from input trajectory", j_name.c_str());
                    continue;
                }

                //Do we have a valid joint state?
                if(base::isInfinity(j_state_full[j_idx_full].position) || base::isNaN(j_state_full[j_idx_full].position)
                        || base::isInfinity(j_state_full[j_idx_full].speed) || base::isNaN(j_state_full[j_idx_full].speed)){
                    LOG_ERROR("A trajectory using joint '%s' was given, but no valid joint state was received. Will skip joint.");
                    continue;
                }

                //FIXME: Is this really necessary?position_target.names[i]
                if(first_it){
                    desired_reflexes[j_idx_full].position = j_state_full[j_idx_full].position;
                    desired_reflexes[j_idx_full].speed = 0.;
                    desired_reflexes[j_idx_full].effort = j_state_full[j_idx_full].effort;
                }

                //Current system state
                if(override_input_position){
                    IP->CurrentPositionVector->VecData[j_idx_full] = desired_reflexes[j_idx_full].position;
                }
                else{
                    IP->CurrentPositionVector->VecData[j_idx_full] = j_state_full[j_idx_full].position;
                }
                if(override_input_speed){
                    IP->CurrentVelocityVector->VecData[j_idx_full] = desired_reflexes[j_idx_full].speed;
                }
                else{
                    IP->CurrentVelocityVector->VecData[j_idx_full] = j_state_full[j_idx_full].speed;
                }
                if(override_input_effort){
                    IP->CurrentAccelerationVector->VecData[j_idx_full] = desired_reflexes[j_idx_full].effort;
                }
                else{
                    IP->CurrentAccelerationVector->VecData[j_idx_full] = j_state_full[j_idx_full].effort;
                }


                //
                // Target system state
                //
                //Just set posiiton
                IP->TargetPositionVector->VecData[j_idx_full] = trajectory[i][current_step].position;
                IP->TargetVelocityVector->VecData[j_idx_full] = trajectory[i][current_step].speed;

                //Everything set, use this joint for control
                IP->SelectionVector->VecData[j_idx_full] = true;
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

                int j_idx_full=0;
                std::string j_name = trajectory.names[i];
                try{j_idx_full = limits.mapNameToIndex(j_name);}
                catch(std::runtime_error){
                    LOG_DEBUG("Skipping unknown joint '%s' from input trajectory", j_name.c_str());
                    continue;
                }
                if(base::isUnset(override_output_speed)){
                    command[i].speed =  OP->NewVelocityVector->VecData[i];
                }
                else{
                    command[i].speed =  override_output_speed;
                }

                if(base::isUnset(override_output_effort)){
                    command[i].effort =  OP->NewAccelerationVector->VecData[i];
                }
                else{
                    command[i].effort =  override_output_effort;
                }

                command.time = base::Time::now();
            }
            break;
        case ReflexxesAPI::RML_FINAL_STATE_REACHED:
            current_step++;
            LOG_WARN("Waypoint %d/%d reached", current_step, trajectory.getTimeSteps());
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
        /*base::Time diff = time-prev_time;
        diff_sum += diff.toSeconds();
        sample_ctn++;
        LOG_INFO_S << "Sample time:  "<<diff.toSeconds()<< " diff_sum: " << diff_sum << " Sample ctn: " << sample_ctn << " cycle_time: "<<cycle_time<<" 1/cycle_time: "<<1./cycle_time<<std::endl;
        if(sample_ctn >= 1./cycle_time){
            double avg_time = diff_sum/sample_ctn;
            LOG_INFO_S << "Average cycle time: " << avg_time<<std::endl;
            LOG_INFO_S << " Configured: " << cycle_time<<std::endl;
            LOG_INFO_S << " diff_sum: " << diff_sum<<std::endl;
            LOG_INFO_S << " sample_ctn: " << sample_ctn<<std::endl;
            _average_cycle_rate.write(avg_time);
            sample_ctn = 0;
            diff_sum = 0.0;
        }*/

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
