/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/Logging.hpp>
#include <base/NamedVector.hpp>

using namespace trajectory_generation;

Task::Task(std::string const& name)
    : TaskBase(name), current_step(0)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), current_step(0)
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
                prev = next = cur;
            else
                next = traj[joint_idx][t+1].position;

            speed = interpolate(prev, cur, next);
            if(fabs(speed) > max_speed)
                max_speed = fabs(speed);

            traj[joint_idx][t].speed = speed;
        }
        if(base::isUnknown(target_speed)){
            target_speed = max_speed;
        }
        //Normalize speed to 1.0 and scale to target speed
        for(uint joint_idx=0; joint_idx<traj.size(); joint_idx++)
        {
            if(max_speed == 0)
                traj[joint_idx][t].speed = 0;
            else
                traj[joint_idx][t].speed = (traj[joint_idx][t].speed/max_speed) * target_speed;
        }
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

    nDof = limits.size();

    cycle_time = _cycle_time.get();

    override_input_position = _override_input_position.value();
    override_input_speed = _override_input_speed.value();
    override_input_acceleration = _override_input_acceleration.value();
    treat_effort_as_acceleration = _treat_effort_as_acceleration.value();
    override_target_velocity = _override_target_velocity.value();

    override_output_speed = _override_output_speed.value();
    override_output_effort = _override_output_effort.value();
    override_speed_value = _override_speed_value.value();
    override_acceleration_value = _override_acceleration_value.value();

    command.resize( nDof );
    command.names = limits.names;

    desired_reflexes.resize(nDof);
    desired_reflexes.names = limits.names;
    output_sample.resize(nDof);
    output_sample.names = limits.names;

    rml_input_params = RMLInputParams(nDof);
    rml_output_params = RMLOutputParams(nDof);

    //Resize trajectory to 1, so that it has correct size for position_target input
    trajectory.resize(nDof, 1);
    trajectory.names = limits.names;

    if( nDof == 0 )
    {
        LOG_ERROR_S << "No joint limits have been configured" << std::endl;
        return false;
    }

    for(uint i=0; i<nDof; i++){
        if(limits[i].max.speed < override_target_velocity){
            LOG_ERROR("override_target_velocity too high. Joint %s has a max speed of %f. override_target_velocity must be lower than that value.",limits.names[i].c_str(), limits[i].max.speed);
            return false;
        }
    }

    dist_to_upper.resize(nDof);
    dist_to_lower.resize(nDof);

    RML = new ReflexxesAPI( nDof, cycle_time );
    IP  = new RMLPositionInputParameters( nDof );
    OP  = new RMLPositionOutputParameters( nDof );
    Flags.SynchronizationBehavior = _sync_behavior.value();
    std::vector<double> max_jerk = _max_jerk.get();
    if(max_jerk.size() != limits.size())
    {
        LOG_ERROR("Size of max jerk property is %i, but size of joint limits is %i", max_jerk.size(), limits.size());
        return false;
    }
    for(uint i=0; i<nDof; i++){
        //Constraints
        IP->MaxVelocityVector->VecData[i] = limits[i].max.speed;
        IP->MaxAccelerationVector->VecData[i] = limits[i].max.effort;
        IP->MaxJerkVector->VecData[i] = _max_jerk.value()[i];
    }

#ifdef USING_REFLEXXES_TYPE_IV
    Flags.PositionalLimitsBehavior = _positional_limits_behavior.get();
    LOG_DEBUG("Joint Limits are: ");
    for(uint i = 0; i < nDof; i++)
    {
        IP->MaxPositionVector->VecData[i] = limits[i].max.position;
        IP->MinPositionVector->VecData[i] = limits[i].min.position;
        LOG_DEBUG("%s: Max %f Min %f", limits.names[i].c_str(), limits[i].max.position, limits[i].min.position);
    }
#endif

    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    current_step = 0;
    has_rml_been_called_once = has_target = false;
    prev_time = base::Time::now();

    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    //If no joint state is avaliable, don't do anything. RML will be uninitialized otherwise
    if(_joint_state.read(j_state) == RTT::NoData){
        LOG_DEBUG("No data on joint state port");
        return;
    }

    //
    // Update current state
    //
    for(size_t i = 0; i < limits.names.size(); i++)
    {
        size_t idx;
        std::string joint_name = limits.names[i];
        try{
            idx = j_state.mapNameToIndex(joint_name);
        }
        catch(std::exception e){ //Only catch exception to write more explicit error msgs
            LOG_ERROR("Joint %s is configured in joint limits, but not available in joint state", limits.names[i].c_str());
            throw e;
        }

        output_sample[i].position = j_state[idx].position;
        output_sample[i].speed = j_state[idx].speed;
        output_sample[i].effort= j_state[idx].effort;

        if(override_input_position && has_rml_been_called_once)
            IP->CurrentPositionVector->VecData[i] = OP->NewPositionVector->VecData[i];
        else
            IP->CurrentPositionVector->VecData[i] = j_state[idx].position;

#ifdef USING_REFLEXXES_TYPE_IV
        //Avoid invalid input here (RML with active Positonal Limits prevention has problems with positional input that is out of limits,
        //which may happen due to noisy position readings)
        if(Flags.PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT)
            IP->CurrentPositionVector->VecData[i] = std::max(std::min(limits[i].max.position, IP->CurrentPositionVector->VecData[i]), limits[i].min.position);
#endif
        //Only use NewVelocityVector from previous cycle if there has been a previous cycle (is_initialized_ == true)
        if(override_input_speed && has_rml_been_called_once){
            IP->CurrentVelocityVector->VecData[i] = OP->NewVelocityVector->VecData[i];
            LOG_DEBUG("Overide current velocity for joint %s to %f", joint_name.c_str(), IP->CurrentVelocityVector->VecData[i]);
        }
        else
            IP->CurrentVelocityVector->VecData[i] = j_state[idx].speed;

        //We can only use 'real' acceleration values from joint status if we treat the effort field as acceleration and don't want to override effort
        if(treat_effort_as_acceleration && !override_input_acceleration){
            //Effort field from joint status is treaded as accerlation and 'real' state should be used. Set it accordingly.
            IP->CurrentAccelerationVector->VecData[i] = j_state[idx].effort;
        }
        //Otherwise we must check whether there was a reference generated before. If so use it, otherwise assume 0
        else{
            if(!has_rml_been_called_once){
                //No reference acceleration was genereated before. Assume zero.
                IP->CurrentAccelerationVector->VecData[i] = 0;
                LOG_DEBUG("Overide current acceleration for joint %s to %f (first cycle)", joint_name.c_str(), IP->CurrentAccelerationVector->VecData[i]);
            }
            else{
                //Override with reference from previous cycle
                IP->CurrentAccelerationVector->VecData[i] = OP->NewAccelerationVector->VecData[i];
                LOG_DEBUG("Overide current acceleration for joint %s to %f", joint_name.c_str(), IP->CurrentAccelerationVector->VecData[i]);
            }
        }
    }

    //
    // Get Trajectory input
    //
    while( _trajectory_target.read( trajectory_from_port, false ) == RTT::NewData )
    {
        for(size_t i=0; i<limits.names.size(); i++)
            IP->SelectionVector->VecData[i] = false;

        // got a new trajectory reading, reset the current index
        current_step = 0;
        has_target = true;

        if(trajectory.getTimeSteps() != trajectory_from_port.getTimeSteps())
            trajectory.resize(nDof, trajectory_from_port.getTimeSteps());

        //Set all trajectory entries to current state
        for(size_t i = 0; i < trajectory.size(); i++)
        {
            for(size_t t = 0; t < trajectory.getTimeSteps(); t++)
            {
                trajectory[i][t].position = IP->CurrentPositionVector->VecData[i];
                trajectory[i][t].speed = 0;
                if(treat_effort_as_acceleration)
                    trajectory[i][t].effort = 0;
                else
                    trajectory[i][t].effort = base::unset<float>();
            }
        }

        for(size_t i = 0; i < trajectory.size(); i++)
        {
            for(size_t t = 0; t < trajectory.getTimeSteps(); t++)
            {
                //This allows partial input, i.e. not all joints that have been configured in joint limits, have to be passed here
                //Also, redundant inputs are ok. They will be ignored, but a warning will be written
                size_t idx;
                try{
                    idx = limits.mapNameToIndex(trajectory_from_port.names[i]);
                }
                catch(std::exception e){
                    LOG_WARN("Joint %s has been given in trajectory, but is not in joint limits. This joint will be ignored", trajectory_from_port.names[i].c_str());
                    continue;
                }

                if(base::isNaN<double>(trajectory_from_port[i][t].position)){
                    LOG_ERROR("Received invalid goal. Joint %d is invalid in timepoint %d",i,t);
                    error();
                }
                trajectory[idx][t].position = trajectory_from_port[i][t].position;

                if(trajectory_from_port[i][t].hasSpeed())//Check min/max speed for input:
                    trajectory[idx][t].speed =  std::max(std::min(trajectory_from_port[i][t].speed, limits[idx].max.speed), limits[idx].min.speed);

                //Enable only joints that are in input trajectory
                IP->SelectionVector->VecData[idx] = true;

            }
        }
        set_speeds(trajectory, override_target_velocity);

        LOG_DEBUG("Got new Trajectory: ");
        for(size_t i = 0; i < trajectory.size(); i++)
            for(uint j = 0; j < trajectory.getTimeSteps(); j++)
                LOG_DEBUG("Pos: %f Vel: %f", trajectory[i][j].position, trajectory[i][j].speed);
    }

    //
    // Get Single position input
    //
    while( _position_target.read( position_target, false ) == RTT::NewData )
    {
        for(size_t i=0; i<limits.size(); i++)
            IP->SelectionVector->VecData[i] = false;

        if(trajectory.getTimeSteps() != 1)
            trajectory.resize(nDof, 1);

        //Set all trajectory position entries to current position and speed/effort to zero
        for(size_t i = 0; i < trajectory.size(); i++)
        {
            trajectory[i][0].position = IP->CurrentPositionVector->VecData[i];
            trajectory[i][0].speed = 0;
            if(treat_effort_as_acceleration)
                trajectory[i][0].effort = 0;
            else
                trajectory[i][0].effort = base::unset<float>();
        }

        for(size_t i = 0; i < limits.size(); i++){
            size_t idx;
            //This allows partial input, i.e. not all joints that have been configured in joint limits, have to be passed here
            //Also, redundant inputs are ok. They will be ignored, but a warning will be written
            try{
                idx = position_target.mapNameToIndex(limits.names[i]);
            }
            catch(std::exception e){
                IP->SelectionVector->VecData[i] = false;
                continue;
            }
            if(base::isNaN<double>(position_target[idx].position)){
                LOG_ERROR("Received invalid goal. Joint %d(%s)",i,limits.names[i].c_str());
                error();
                return;
            }
            trajectory.elements[i][0].position = position_target[idx].position;
            if(position_target[idx].hasSpeed())//Check min/max speed for input:
                trajectory.elements[i][0].speed = std::max(std::min(position_target[idx].speed, limits[i].max.speed), limits[i].min.speed);

            //Enable only joints that are in input trajectory
            IP->SelectionVector->VecData[i] = true;

        }
        current_step = 0;
        has_target = true;
    }

    if( current_step < trajectory.getTimeSteps() )
    {
        for( size_t i=0; i<limits.names.size(); i++ )
        {
            IP->TargetPositionVector->VecData[i] = trajectory[i][current_step].position;
            IP->TargetVelocityVector->VecData[i] = trajectory[i][current_step].speed;
        }
    }

    //
    // Handle Reset commands
    //
    if(_reset.read(reset_command) == RTT::NewData){
        for(size_t i = 0; i < reset_command.size(); i++){
            size_t joint_idx;
            std::string joint_name = reset_command.names[i];
            try{
                joint_idx = limits.mapNameToIndex(joint_name);
            }
            catch(std::exception e){
                continue;
            }
            if(reset_command[i].hasPosition()){
                IP->CurrentPositionVector->VecData[joint_idx] = reset_command[i].position;
                IP->TargetPositionVector->VecData[joint_idx] = reset_command[i].position;
            }
            if(reset_command[i].hasSpeed()){
                IP->CurrentVelocityVector->VecData[joint_idx] = reset_command[i].speed;
                IP->TargetVelocityVector->VecData[joint_idx] = reset_command[i].speed;
            }
            if(treat_effort_as_acceleration && reset_command[i].hasEffort()){
                IP->CurrentAccelerationVector->VecData[joint_idx] = reset_command[i].effort;
                LOG_DEBUG("Reset current acceleration of joint %s to %f", joint_name.c_str(), IP->CurrentAccelerationVector->VecData[joint_idx]);
            }
            else{
                //We dont have acceleration information. Assume zero.
                // FIXME: Is this really better than assuming previous reference? I think so, but not sure.
                IP->CurrentAccelerationVector->VecData[joint_idx] = 0;
            }

        }
    }

    //
    // Sample point
    //

    //Only start sampling after the first input command has arrived
    if(has_target)
    {
        int result = RML->RMLPosition( *IP, OP, Flags );
        has_rml_been_called_once = true;

        for( size_t i=0; i<command.size(); ++i )
        {


            output_sample[i].position = OP->NewPositionVector->VecData[i];
            output_sample[i].speed = OP->NewVelocityVector->VecData[i];
            output_sample[i].effort= OP->NewAccelerationVector->VecData[i];

            if(base::isNaN<double>(OP->NewPositionVector->VecData[i])){
                LOG_ERROR_S << "Relexxes calculated reference position as NaN!";
                IP->Echo();
                OP->Echo();
            }

            command[i].position = OP->NewPositionVector->VecData[i];

            if(override_output_speed)
                command[i].speed =  override_speed_value;
            else
                command[i].speed =  OP->NewVelocityVector->VecData[i];

            if(treat_effort_as_acceleration){
                if(override_output_effort)
                    command[i].effort =  override_acceleration_value;
                else
                    command[i].effort =  OP->NewAccelerationVector->VecData[i];
            }
            else{
                command[i].effort = base::unset<float>();
            }

            command.time = base::Time::now();
        }

        switch(result){
        case ReflexxesAPI::RML_WORKING:
            state(FOLLOWING);
            break;
        case ReflexxesAPI::RML_FINAL_STATE_REACHED:
            if(state() != REACHED)
            {
                current_step++;
                LOG_INFO("Waypoint %d/%d reached", current_step, trajectory.getTimeSteps());
                if(current_step >= trajectory.getTimeSteps())
                    state(REACHED);
            }
            break;
        case ReflexxesAPI::RML_ERROR:
            LOG_ERROR("RML_ERROR");
            IP->Echo();
            OP->Echo();
            error();
            break;
        case ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES:
            LOG_ERROR("RML_ERROR_INVALID_INPUT_VALUES");
            IP->Echo();
            OP->Echo();
            error();
            break;
        case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION:
            LOG_ERROR("RML_ERROR_EXECUTION_TIME_CALCULATION");
            IP->Echo();
            OP->Echo();
            error();
            break;
        case ReflexxesAPI::RML_ERROR_SYNCHRONIZATION:
            LOG_ERROR("RML_ERROR_SYNCHRONIZATION");
            IP->Echo();
            OP->Echo();
            error();
            break;
        case ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS:
            LOG_ERROR("RML_ERROR_NUMBER_OF_DOFS");
            IP->Echo();
            OP->Echo();
            error();
            break;
        case ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION:
            LOG_ERROR("RML_ERROR_NO_PHASE_SYNCHRONIZATION");
            IP->Echo();
            OP->Echo();
            error();
            break;
        case ReflexxesAPI::RML_ERROR_NULL_POINTER:
            LOG_ERROR("RML_ERROR_NULL_POINTER");
            IP->Echo();
            OP->Echo();
            error();
            break;
        case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG:
            LOG_ERROR("RML_ERROR_EXECUTION_TIME_TOO_BIG");
            IP->Echo();
            OP->Echo();
            error();
            break;
        case ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE:
            LOG_ERROR("RML_ERROR_USER_TIME_OUT_OF_RANGE");
            IP->Echo();
            OP->Echo();
            error();
            break;
#ifdef USING_REFLEXXES_TYPE_IV
        case ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS:
            state(IN_LIMITS);
            break;
#endif
        }

        _cmd.write( command );
    }

    _output_sample.write(output_sample);

    //
    // Write debug data
    //

    base::Time time = base::Time::now();
    base::Time diff = time-prev_time;
    _actual_cycle_time.write(diff.toSeconds());
    prev_time = time;

    for(uint i = 0; i < nDof; i++){
        rml_input_params.CurrentPositionVector[i] = IP->CurrentPositionVector->VecData[i];
        rml_input_params.CurrentVelocityVector[i] = IP->CurrentVelocityVector->VecData[i];
        rml_input_params.CurrentAccelerationVector[i] = IP->CurrentAccelerationVector->VecData[i];
        rml_input_params.TargetPositionVector[i] = IP->TargetPositionVector->VecData[i];
        rml_input_params.TargetVelocityVector[i] = IP->TargetVelocityVector->VecData[i];
        rml_input_params.MaxVelocityVector[i] = IP->MaxVelocityVector->VecData[i];
        rml_input_params.MaxAccelerationVector[i] = IP->MaxAccelerationVector->VecData[i];
        rml_input_params.MaxJerkVector[i] = IP->MaxJerkVector->VecData[i];
        rml_input_params.SelectionVector[i] = IP->SelectionVector->VecData[i];

        rml_output_params.ExecutionTimes[i] = OP->ExecutionTimes->VecData[i];
        rml_output_params.NewPositionVector[i] = OP->NewPositionVector->VecData[i];
        rml_output_params.NewVelocityVector[i] = OP->NewVelocityVector->VecData[i];
        rml_output_params.NewAccelerationVector[i] = OP->NewAccelerationVector->VecData[i];
    }
    rml_input_params.NumberOfDOFs = IP->NumberOfDOFs;
    rml_input_params.MinimumSynchronizationTime = IP->MinimumSynchronizationTime;
    rml_output_params.ANewCalculationWasPerformed = OP->ANewCalculationWasPerformed;
    rml_output_params.NumberOfDOFs = OP->NumberOfDOFs;
    rml_output_params.DOFWithTheGreatestExecutionTime = OP->DOFWithTheGreatestExecutionTime;
    rml_output_params.SynchronizationTime = OP->SynchronizationTime;
    rml_output_params.TrajectoryIsPhaseSynchronized = OP->TrajectoryIsPhaseSynchronized;

#ifdef USING_REFLEXXES_TYPE_IV
    for(uint i = 0; i < nDof; i++){
        rml_input_params.MinPositionVector[i] = IP->MinPositionVector->VecData[i];
        rml_input_params.MaxPositionVector[i] = IP->MaxPositionVector->VecData[i];
    }
    for(uint i = 0; i < nDof; i++)
    {
        dist_to_upper(i) = limits[i].max.position - IP->CurrentPositionVector->VecData[i];
        dist_to_lower(i) = IP->CurrentPositionVector->VecData[i] - limits[i].min.position;
    }
    _dist_lower_limit.write(dist_to_lower);
    _dist_upper_limit.write(dist_to_upper);
#endif

    _rml_input_params.write(rml_input_params);
    _rml_output_params.write(rml_output_params);

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
