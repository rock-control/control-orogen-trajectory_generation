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

    write_debug_data = _write_debug_data.get();

    override_input_position = _override_input_position.value();
    override_input_speed = _override_input_speed.value();
    override_input_acceleration = _override_input_acceleration.value();
    treat_effort_as_acceleration = _treat_effort_as_acceleration.value();
    override_target_velocity = _override_target_velocity.value();

    override_output_speed = _override_output_speed.value();
    override_output_effort = _override_output_effort.value();
    override_speed_value = _override_speed_value.value();
    override_acceleration_value = _override_acceleration_value.value();

    output_command.resize( nDof );
    output_command.names = limits.names;

    if(write_debug_data){
        debug_output_command_unmodified.resize(nDof);
        debug_output_command_unmodified.names = limits.names;

        debug_rml_input_params = RMLInputParams(nDof);
        debug_rml_output_params = RMLOutputParams(nDof);
    }

    //Resize trajectory to 1, so that it has correct size for position_target input
    current_trajectory.resize(nDof, 1);
    current_trajectory.names = limits.names;

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
    IP_static  = new RMLPositionInputParameters( nDof );
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
        IP_static->MaxVelocityVector->VecData[i] = limits[i].max.speed;
        IP_static->MaxAccelerationVector->VecData[i] = limits[i].max.effort;
        IP_static->MaxJerkVector->VecData[i] = _max_jerk.value()[i];
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

    //Create copy for per-sample change of properties
    IP_active  = new RMLPositionInputParameters( nDof );

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

size_t Task::map_joint_name_to_index(const std::string& joint_name)
{
    return limits.mapNameToIndex(joint_name);
}

void Task::set_active_joints(const std::vector &joint_names)
{
    assert(IP_static->SelectionVector->VecData == limits.names.size());

    //Go throu all known joints and test if it is also in the joint_names vector.
    //If so, set corresponding joint as active. If not set corresponding joint as
    //inactive.
    std::vector<std::string>::iterator it;
    for(uint i=0; i<limits.size(); i++){
        it = std::find(joint_names.begin(), joint_names.end(), limits.names[i]);
        if(it == joint_names.end())
            IP_static->SelectionVector->VecData[i] = false;
        else
            IP_static->SelectionVector->VecData[i] = true;
    }
}

void Task::set_active_motion_constraints(const JointsMotionConstraints& motion_constraints)
{
    //Preinitialize with configured default values
    IP_active->MaxVelocityVector = IP_static->MaxVelocityVector;
    IP_active->MaxJerkVector = IP_static->MaxJerkVector;
    IP_active->MaxAccelerationVector = IP_static->MaxAccelerationVector;
#ifdef USING_REFLEXXES_TYPE_IV
    IP_temporary->MaxPositionVector = IP_static->MaxPositionVector;
    IP_temporary->MinPositionVector = IP_static->MinPositionVector;
#endif

    //Override if new constraints are set
    size_t idx;
    for(size_t i=0; i<motion_constraints.size(); i++){
        idx = map_joint_name_to_index(motion_constraints.names[i]);
        if(idx){
#ifdef USING_REFLEXXES_TYPE_IV
            //Only override position constraint, if a valid value is set
            if(!base::isNaN<double>(motion_constraints[i].min.position))
                IP_active->MinPositionVector[idx] = motion_constraints[i].min.position;
            if(!base::isNaN<double>(motion_constraints[i].max.position))
                IP_active->MaxPositionVector[idx] = motion_constraints[i].max.position;
#endif

            //Only override velocity constraint, if a valid value is set
            if(!base::isNaN<float>(motion_constraints[i].max.speed))
                IP_active->MaxVelocityVector[idx] = motion_constraints[i].max.speed;

            //Only override acceleration constraint, if a valid value is set
            if(!base::isNaN<float>(motion_constraints[i].max.speed))
                IP_active->MaxAccelerationVector[idx] = motion_constraints[i].max.acceleration;

            //Only override jerk constraint, if a valid value is set
            if(!base::isNaN<float>(motion_constraints[i].max.speed))
                IP_active->MaxJerkVector[idx] = motion_constraints[i].max_jerk;
        }
    }
}

void Task::set_active_motion_constraints_to_default()
{
    set_active_motion_constraints(JointMotionConstraints());
}

void Task::get_default_motion_constraints(size_t internal_index, JointMotionConstraints& constraints)
{
    constraints.min = limits[idx].min;
    constraints.max = limits[idx].max;
    constraints.max_jerk = max_jerk[idx];
}

void Task::get_default_motion_constraints(const std::string &joint_name, JointMotionConstraints& constraints)
{
    size_t idx = map_joint_name_to_index(joint_name);
    if(!idx)
        throw(base::JointLimits::InvalidName(joint_name));

    get_default_motion_constraints(idx, constraints);
}

bool Task::make_feasible(ConstrainedJointsTrajectory& sample)
{
    bool was_feasible = sample.makeFeasible();
    if(!was_feasible){
        LOG_INFO_S << "The trajectory was not within its constraints. It had to be corrected.";
    }
    return was_feasible;
}

bool Task::handle_position_target(const base::commands::Joints& sample)
{
    //Create Contraint trajectory from saple with with one via point. Use default
    //motion constraints.
    current_trajectory.resize(nDof, 1);

    //Set active joints according to sample
    set_active_joints(sample.names);

    size_t internal_idx;
    for(size_t i=0; i<sample.size(); i++){
        internal_idx = map_joint_name_to_index(sample.names[i]);
        current_trajectory.motion_constraints[internal_idx][0] = get_default_motion_constraints(internal_idx);
        current_trajectory.elements[internal_idx][0] = sample[i];
    }

    return make_feasible(current_trajectory);
}

bool Task::handle_trajectory_target(const base::JointsTrajectory& sample)
{
    //Create constraint trajectory from sample with with via points form sampole. Use default
    //motion constraints.
    current_trajectory.resize(nDof, sample.getTimeSteps());

    //Set active joints according to sample
    set_active_joints(sample.names);

    size_t internal_joint_idx;
    for(size_t joint_idx=0; joint_idx<sample.size(); joint_idx++){
        internal_joint_idx = map_joint_name_to_index(sample.names[i]);
        for(size_t time_idx=0; time_idx<sample.size(); time_idx++){
            current_trajectory.motion_constraints[internal_idx][time_idx] = get_default_motion_constraints(internal_joint_idx);
            current_trajectory.elements[internal_idx][time_idx] = sample[joint_idx][time_idx];
        }
    }

    return make_feasible(current_trajectory);
}

bool Task::handle_constrained_trajectory_target(const ConstrainedJointsTrajectory& sample)
{
    //Create constraint trajectory from sample with with via points form sampole. Use default
    //motion constraints.
    current_trajectory.resize(nDof, sample.getTimeSteps());

    //Set active joints according to sample
    set_active_joints(sample.names);

    size_t internal_joint_idx;
    for(size_t joint_idx=0; joint_idx<sample.size(); joint_idx++){
        internal_joint_idx = map_joint_name_to_index(sample.names[i]);
        for(size_t time_idx=0; time_idx<sample.size(); time_idx++){
            current_trajectory.motion_constraints[internal_idx][time_idx] = sample.motion_constraints[joint_idx][time_idx];
            current_trajectory.elements[internal_idx][time_idx] = sample[joint_idx][time_idx];
        }
    }

    return make_feasible(current_trajectory);
}

void Task::set_current_joint_state(const base::samples::Joints& sample)
{
    size_t index_in_sample;
    std::string joint_name;
    for(size_t internal_index = 0; internal_index < limits.names.size(); internal_index++){
        joint_name = limits.names[internal_index];

        //See if required joint is found in sample. Throw if not
        try{
            index_in_sample = input_joint_state.mapNameToIndex(joint_name);
        }
        catch(base::samples::Joints::InvalidName e){ //Only catch exception to write more explicit error msgs
            LOG_ERROR("Joint %s is configured in joint limits, but not available in joint state", joint_name.c_str());
            throw e;
        }


        //If there was no control command previously generated, set current state from sample in any case
        //Otherwise perfor check wether input should be overridden or note
        if(override_input_position && has_rml_been_called_once)
            IP_active->CurrentPositionVector->VecData[internal_index] = OP->NewPositionVector->VecData[internal_index];
        else{
            if(!sample[index_in_sample].hasPosition()){
                LOG_ERROR("Joint %s does not have a position value.", joint_name.c_str());
                throw(std::runtime_error("Invalid joint sample"));
            }
            IP_active->CurrentPositionVector->VecData[internal_index] = sample[index_in_sample].position;
        }

#ifdef USING_REFLEXXES_TYPE_IV
        //Avoid invalid input here (RML with active Positonal Limits prevention has problems with positional input that is out of limits,
        //which may happen due to noisy position readings)
        //FIXME: What exactly is this 'problem'? Is it a crash or undesired behavior? If the second, what make the behavior undesireable?
        if(Flags.PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT)
            IP->CurrentPositionVector->VecData[internal_index] = std::max(std::min(limits[internal_index].max.position, IP->CurrentPositionVector->VecData[internal_index]), limits[internal_index].min.position);
#endif
        //Only use NewVelocityVector from previous cycle if there has been a previous cycle
        if(override_input_speed && has_rml_been_called_once){
            IP_active->CurrentVelocityVector->VecData[internal_index] = OP->NewVelocityVector->VecData[internal_index];
            LOG_DEBUG("Overide current velocity for joint %s to %f", joint_name.c_str(), IP_static->CurrentVelocityVector->VecData[internal_index]);
        }
        else{
            if(input_joint_state[idx].hasSpeed())
                IP_active->CurrentVelocityVector->VecData[internal_index] = input_joint_state[idx].speed;
            else{
                IP_active->CurrentVelocityVector->VecData[internal_index] = 0.0;
                if(!override_input_speed){
                    throw(std::runtime_error("override_input_speed was configured to false, but not all joints have a speed measurement."));
                }
            }
        }

        //We can only use 'real' acceleration values from joint status if we treat the effort field as acceleration and don't want to override effort
        if(treat_effort_as_acceleration && !override_input_acceleration){
            //Effort field from joint status is treaded as accerlation and 'real' state should be used. Set it accordingly.
            IP_active->CurrentAccelerationVector->VecData[internal_index] = input_joint_state[idx].effort;
            if(!input_joint_state[idx].hasEffort()){
                throw(std::runtime_error("override_input_acceleration was configured to false, but not all joints have a acceleration measurement on effort field."));
            }
        }
        //Otherwise we must check whether there was a reference generated before. If so use it, otherwise assume 0
        else{
            if(!has_rml_been_called_once){
                //No reference acceleration was genereated before. Assume zero.
                IP_active->CurrentAccelerationVector->VecData[internal_index] = 0;
                LOG_DEBUG("Overide current acceleration for joint %s to %f (first cycle)", joint_name.c_str(), IP_static->CurrentAccelerationVector->VecData[internal_index]);
            }
            else{
                //Override with reference from previous cycle
                IP_active->CurrentAccelerationVector->VecData[internal_index] = OP->NewAccelerationVector->VecData[internal_index];
                LOG_DEBUG("Overide current acceleration for joint %s to %f", joint_name.c_str(), IP_static->CurrentAccelerationVector->VecData[internal_index]);
            }
        }
    }
}

void Task::handle_joint_state(const base::samples::Joints &sample)
{
    //Set current joint state from sample
    set_current_joint_state(sample);

    //Pre-initialize output command with current joint state
    output_command[i].position = input_joint_state[idx].position;
    output_command[i].speed = input_joint_state[idx].speed;
    output_command[i].effort= input_joint_state[idx].effort;
}

void Task::handle_reflexxes_result_value(const int& result)
{
    switch(result){
    case ReflexxesAPI::RML_WORKING:
        state(FOLLOWING);
        break;
    case ReflexxesAPI::RML_FINAL_STATE_REACHED:
        if(state() != REACHED)
        {
            current_step++;
            LOG_INFO("Waypoint %d/%d reached", current_step, current_trajectory.getTimeSteps());
            if(current_step >= current_trajectory.getTimeSteps())
                state(REACHED);
        }
        break;
    case ReflexxesAPI::RML_ERROR:
        LOG_ERROR("RML_ERROR");
        IP_static->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES:
        LOG_ERROR("RML_ERROR_INVALID_INPUT_VALUES");
        IP_static->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION:
        LOG_ERROR("RML_ERROR_EXECUTION_TIME_CALCULATION");
        IP_static->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_SYNCHRONIZATION:
        LOG_ERROR("RML_ERROR_SYNCHRONIZATION");
        IP_static->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS:
        LOG_ERROR("RML_ERROR_NUMBER_OF_DOFS");
        IP_static->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION:
        LOG_ERROR("RML_ERROR_NO_PHASE_SYNCHRONIZATION");
        IP_static->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_NULL_POINTER:
        LOG_ERROR("RML_ERROR_NULL_POINTER");
        IP_static->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG:
        LOG_ERROR("RML_ERROR_EXECUTION_TIME_TOO_BIG");
        IP_static->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE:
        LOG_ERROR("RML_ERROR_USER_TIME_OUT_OF_RANGE");
        IP_static->Echo();
        OP->Echo();
        error();
        break;
#ifdef USING_REFLEXXES_TYPE_IV
    case ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS:
        state(IN_LIMITS);
        break;
#endif
    }
}

void Task::updateHook()
{
    TaskBase::updateHook();

    if(_trajectory_target.readNewest(input_trajectory_target, false) == RTT::NewData){
        //TODO: Function that prepares data structures to ahnde a new command:
        //      reset_for_new_command();
        //      It should at least set current_step to zero and has_target = true;
        LOG_DEBUG("Received a new target on trajectory input port");
        handle_trajectory_target(input_trajectory_target);
    }

    if(_position_target.read( input_position_target, false ) == RTT::NewData){
        //TODO: Function that prepares data structures to ahnde a new command:
        //      reset_for_new_command();
        //      It should at least set current_step to zero and has_target = true;
        LOG_DEBUG("Received a new target on trajectory input port");
        handle_position_target(input_position_target_target);
    }

    //TODO: Read from constrained trajectory input port


    //If no joint state is avaliable, don't do anything. RML will be uninitialized otherwise
    if(_joint_state.readNewest(input_joint_state) == RTT::NoData){
        LOG_DEBUG("No data on joint state port");
        return;
    }
    handle_joint_state(input_joint_state);


    //As long as no target was received, do nothing
    if(!has_target)
        return;

    //Prepare data for Reflexxes
    if( current_step < current_trajectory.getTimeSteps() )
    {
        for( size_t i=0; i<limits.names.size(); i++ )
        {
            IP_active->TargetPositionVector->VecData[i] = current_trajectory[i][current_step].position;
            IP_active->TargetVelocityVector->VecData[i] = current_trajectory[i][current_step].speed;
            //TODO: Set constraints from current_trajectory
        }
    }

    //FIXME: Deleted a bunch of code dealing with 'allow_positive' and 'allow_negative'.
    //       It seems rather hacky, what was it for? I don't think it should be here.

    //Perform control step with reflexxes
    int result = RML->RMLPosition( *IP_static, OP, Flags );
    has_rml_been_called_once = true;
    handle_reflexxes_result_value(result);

    //Prepare output
    for( size_t i=0; i<output_command.size(); ++i )
    {
        //Check if command is okay
        if(base::isNaN<double>(OP->NewPositionVector->VecData[i])){
            LOG_ERROR("Relexxes calculated reference position for joint %d as NaN.", i);
            IP_static->Echo();
            OP->Echo();
            throw(std::runtime_error("Command generated by reflexxes was invalid"));
        }

        //Copy data to base type
        output_command[i].position = OP->NewPositionVector->VecData[i];
        output_command[i].speed = OP->NewVelocityVector->VecData[i];
        output_command[i].effort= OP->NewAccelerationVector->VecData[i];
        output_command.time = base::Time::now();

        if(write_debug_data){
            debug_output_command_unmodified = output_command;
            //FIXME: Rename this port to something more understandable. Maybe: debug_output_command_unmodified
            _output_sample.write(debug_output_command_unmodified);
        }

        //Override output if necessary
        if(override_output_speed)
            output_command[i].speed =  override_speed_value;

        if(treat_effort_as_acceleration){
            if(override_output_effort)
                output_command[i].effort =  override_acceleration_value;
            else
                output_command[i].effort =  OP->NewAccelerationVector->VecData[i];
        }
        else{
            output_command[i].effort = base::unset<float>();
        }
    }

    //Write command output
    _cmd.write( output_command );

    //
    // Write debug data
    //
    if(write_debug_data){
        //Write timing information to output port
        base::Time time = base::Time::now();
        base::Time diff = time-prev_time;
        _actual_cycle_time.write(diff.toSeconds());
        prev_time = time;


        for(uint i = 0; i < nDof; i++){
            debug_rml_input_params.CurrentPositionVector[i] = IP_static->CurrentPositionVector->VecData[i];
            debug_rml_input_params.CurrentVelocityVector[i] = IP_static->CurrentVelocityVector->VecData[i];
            debug_rml_input_params.CurrentAccelerationVector[i] = IP_static->CurrentAccelerationVector->VecData[i];
            debug_rml_input_params.TargetPositionVector[i] = IP_static->TargetPositionVector->VecData[i];
            debug_rml_input_params.TargetVelocityVector[i] = IP_static->TargetVelocityVector->VecData[i];
            debug_rml_input_params.MaxVelocityVector[i] = IP_static->MaxVelocityVector->VecData[i];
            debug_rml_input_params.MaxAccelerationVector[i] = IP_static->MaxAccelerationVector->VecData[i];
            debug_rml_input_params.MaxJerkVector[i] = IP_static->MaxJerkVector->VecData[i];
            debug_rml_input_params.SelectionVector[i] = IP_static->SelectionVector->VecData[i];

            debug_rml_output_params.ExecutionTimes[i] = OP->ExecutionTimes->VecData[i];
            debug_rml_output_params.NewPositionVector[i] = OP->NewPositionVector->VecData[i];
            debug_rml_output_params.NewVelocityVector[i] = OP->NewVelocityVector->VecData[i];
            debug_rml_output_params.NewAccelerationVector[i] = OP->NewAccelerationVector->VecData[i];
        }
        debug_rml_input_params.NumberOfDOFs = IP_static->NumberOfDOFs;
        debug_rml_input_params.MinimumSynchronizationTime = IP_static->MinimumSynchronizationTime;
        debug_rml_output_params.ANewCalculationWasPerformed = OP->ANewCalculationWasPerformed;
        debug_rml_output_params.NumberOfDOFs = OP->NumberOfDOFs;
        debug_rml_output_params.DOFWithTheGreatestExecutionTime = OP->DOFWithTheGreatestExecutionTime;
        debug_rml_output_params.SynchronizationTime = OP->SynchronizationTime;
        debug_rml_output_params.TrajectoryIsPhaseSynchronized = OP->TrajectoryIsPhaseSynchronized;

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

        _rml_input_params.write(debug_rml_input_params);
        _rml_output_params.write(debug_rml_output_params);
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
    delete IP_static;
    delete OP;

    TaskBase::cleanupHook();
}
