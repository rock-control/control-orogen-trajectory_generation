/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/Logging.hpp>
#include <base/NamedVector.hpp>
#include <algorithm>

using namespace trajectory_generation;

Task::Task(std::string const& name)
    : TaskBase(name), current_step(0)
{
    LOG_CONFIGURE(DEBUG, stdout);
    do_write_command=true;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), current_step(0)
{
    LOG_CONFIGURE(DEBUG, stdout);
    do_write_command=true;
}

Task::~Task()
{
}

double interpolate(double a, double b, double c){
    double ba=b-a;
    double cb=c-b;
    return (ba+cb)/2.0;
}

void set_speeds(trajectory_generation::ConstrainedJointsTrajectory& traj, const base::samples::Joints &joint_state){
    // Fixme: Setting the speeds like this leads to overshoot, if the maximum speed is too high with respect to max_jerk and max_acceleration.
    //        The speed computation should take max jerk and max acceleration into account. How?
    double cur, prev, next, speed;
    for(size_t t=0; t<traj.getTimeSteps(); t++)
    {
        double max_speed = 0;
        for(size_t joint_idx=0; joint_idx<traj.size(); joint_idx++)
        {
            cur = traj[joint_idx][t].position;
            const base::JointState &state = joint_state.getElementByName(traj.names[joint_idx]);

            if(t==0)
                prev = state.position;
            else
                prev = traj[joint_idx][t-1].position;

            if(t==traj.getTimeSteps()-1)
                prev = next = cur;
            else
                next = traj[joint_idx][t+1].position;

            speed = interpolate(prev, cur, next);

            //Check for inflection points. Target velocity should always be zero at these points
            if( (next > cur && prev > cur) ||
                (next < cur && prev < cur) ||
                (next == cur) )
               speed = 0;

            if(fabs(speed) > max_speed)
                max_speed = fabs(speed);

            traj[joint_idx][t].speed = speed;
        }

        //Normalize speed to 1.0 and scale to target speed
        for(size_t joint_idx=0; joint_idx<traj.size(); joint_idx++)
        {
            if(max_speed == 0)
                traj[joint_idx][t].speed = 0;
            else{
                traj[joint_idx][t].speed = (traj[joint_idx][t].speed/max_speed) * traj.motion_constraints[joint_idx][t].max.speed;
                if(fabs(traj[joint_idx][t].speed) > 1e-10)
                    traj.motion_constraints[joint_idx][t].max.speed = fabs(traj[joint_idx][t].speed);
            }
        }
    }
}


bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    limits = _limits.get();
    nDof = limits.size();
    cycle_time = _cycle_time.get();
    if(cycle_time <= 0){
        LOG_ERROR("Cycle time has to be > 0! Note that the cycle time has to be configured to be the same as the period of your component");
        return false;
    }
    if(limits.elements.size() != limits.names.size()){
        LOG_ERROR("Limits property defines %i joint names but has %i elements", limits.names.size(), limits.elements.size());
        return false;
    }

    write_debug_data = _write_debug_data.get();
    override_input_position = _override_input_position.get();
    override_input_speed = _override_input_speed.get();
    override_input_acceleration = _override_input_acceleration.get();
    override_output_speed = _override_output_speed.get();
    override_output_acceleration = _override_output_acceleration.get();
    throw_on_infeasible_input = _throw_on_infeasible_input.get();

    if(override_output_speed.elements.size() != override_output_speed.names.size()){
        LOG_ERROR("override_output_speed property defines %i names, but has %i elements", override_output_speed.names.size(), override_output_speed.elements.size());
        return false;
    }
    if(override_output_acceleration.elements.size() != override_output_acceleration.names.size()){
        LOG_ERROR("override_output_acceleration property defines %i names, but has %i elements", override_output_acceleration.names.size(), override_output_acceleration.elements.size());
        return false;
    }

    output_command.resize( nDof );
    output_command.names = limits.names;

    rml_output_sample.resize(nDof);
    rml_output_sample.names = limits.names;

    if(write_debug_data){
        debug_rml_input_params = RMLInputParams(nDof);
        debug_rml_output_params = RMLOutputParams(nDof);
    }

    //Resize trajectory to 1, so that it has correct size for position_target input
    current_trajectory.resize(nDof, 1);
    current_trajectory.names = limits.names;

    if( nDof == 0 )
    {
        LOG_ERROR_S << "Size of joint limits is zero!" << std::endl;
        return false;
    }

    RML = new ReflexxesAPI( nDof, cycle_time );
    IP_static  = new RMLPositionInputParameters( nDof );
    OP  = new RMLPositionOutputParameters( nDof );
    Flags.SynchronizationBehavior = _sync_behavior.value();

    //Set initial constraints
    for(size_t i=0; i<nDof; i++){
        if(limits[i].max.hasSpeed() && limits[i].max.speed > 0)
            IP_static->SetMaxVelocityVectorElement(limits[i].max.speed, i);
        else{
            LOG_ERROR("Limits vector: Element %i (%s) does not define a valid maximum speed", i, limits.names[i].c_str());
            return false;
        }
        if(limits[i].max.hasAcceleration() && limits[i].max.acceleration > 0)
            IP_static->SetMaxAccelerationVectorElement(limits[i].max.acceleration, i);
        else {
            LOG_ERROR("Limits vector: Element %i (%s) does not define a valid maximum acceleration", i, limits.names[i].c_str());
            return false;
        }
        if(!base::isUnset(limits[i].max_jerk) && limits[i].max_jerk > 0)
            IP_static->SetMaxJerkVectorElement(limits[i].max_jerk, i);
        else{
            LOG_ERROR("Limits vector: Element %i (%s) does not define a valid maximum jerk", i, limits.names[i].c_str());
            return false;
        }
    }

#ifdef USING_REFLEXXES_TYPE_IV
    Flags.PositionalLimitsBehavior = _positional_limits_behavior.get();
    LOG_DEBUG("Joint Limits are: ");
    for(uint i = 0; i < nDof; i++)
    {
        if(limits[i].max.position <= limits[i].min.position){
            LOG_ERROR("Maximum position of joint %i (%s) is <= minimum position of that joint", i, limits.names[i].c_str());
            return false;
        }
        IP_static->SetMaxPositionVectorElement(limits[i].max.position, i);
        IP_static->SetMinPositionVectorElement(limits[i].min.position, i);
        LOG_DEBUG("%s: Max %f Min %f", limits.names[i].c_str(), limits[i].max.position, limits[i].min.position);
    }
#endif

    //Create copy for per-sample change of properties
    IP_active  = new RMLPositionInputParameters( nDof );
    current_motion_constraints.resize(nDof);
    current_motion_constraints.names = limits.names;

    assert(limits.size() == nDof);
    assert(current_motion_constraints.size() == nDof);
    assert(output_command.size() == nDof);
    assert(current_trajectory.getNumberOfJoints() == nDof);
    assert(rml_output_sample.size() == nDof);

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

void Task::set_active_joints(const std::vector<std::string> &joint_names)
{
    //Go through all known joints and test if it is also in the joint_names vector.
    //If so, set corresponding joint as active. If not set corresponding joint as
    //inactive.
    for(uint i=0; i<limits.size(); i++){
        if(std::find(joint_names.begin(), joint_names.end(), limits.names[i]) == joint_names.end())
            IP_active->SetSelectionVectorElement(false, i);
        else
            IP_active->SetSelectionVectorElement(true, i);
    }
}

void Task::set_active_motion_constraints(const JointsMotionConstraints& motion_constraints)
{
    //Preinitialize with configured default values
    IP_active->SetMaxVelocityVector(*IP_static->MaxVelocityVector);
    IP_active->SetMaxJerkVector(*IP_static->MaxJerkVector);
    IP_active->SetMaxAccelerationVector(*IP_static->MaxAccelerationVector);

    assert(IP_active->MaxVelocityVector->GetVecDim() == nDof);
    assert(IP_active->MaxJerkVector->GetVecDim() == nDof);
    assert(IP_active->MaxAccelerationVector->GetVecDim() == nDof);
    assert(motion_constraints.size() == nDof);

#ifdef USING_REFLEXXES_TYPE_IV
    IP_active->SetMaxPositionVector(*IP_static->MaxPositionVector);
    IP_active->SetMinPositionVector(*IP_static->MinPositionVector);
    assert(IP_active->MinPositionVector->GetVecDim() == nDof);
    assert(IP_active->MaxPositionVector->GetVecDim() == nDof);
#endif

    //Override if new constraints are set
    size_t idx;
    for(size_t i=0; i<motion_constraints.size(); i++)
    {
        try{
            idx = limits.mapNameToIndex(motion_constraints.names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("Motion constraints contain joint %s, but this joint has not been configured", motion_constraints.names[i].c_str());
            throw e;
        }

#ifdef USING_REFLEXXES_TYPE_IV
        //Only override position constraint, if a valid value is set
        if(!base::isNaN<double>(motion_constraints[i].min.position))
            IP_active->SetMinPositionVectorElement(motion_constraints[i].min.position, idx);

        if(!base::isNaN<double>(motion_constraints[i].max.position))
            IP_active->SetMaxPositionVectorElement(motion_constraints[i].max.position, idx);
#endif

        //Only override velocity constraint, if a valid value is set
        if(!base::isNaN<float>(motion_constraints[i].max.speed))
            IP_active->SetMaxVelocityVectorElement(motion_constraints[i].max.speed, idx);

        //Only override acceleration constraint, if a valid value is set
        if(!base::isNaN<float>(motion_constraints[i].max.acceleration))
            IP_active->SetMaxAccelerationVectorElement(motion_constraints[i].max.acceleration, idx);

        //Only override jerk constraint, if a valid value is set
        if(!base::isNaN<float>(motion_constraints[i].max_jerk))
            IP_active->SetMaxJerkVectorElement(motion_constraints[i].max_jerk, idx);
    }
}

void Task::get_default_motion_constraints(size_t internal_index, JointMotionConstraints& constraints)
{
    constraints.min = limits[internal_index].min;
    constraints.max = limits[internal_index].max;
    constraints.max_jerk = limits[internal_index].max_jerk;
}

void Task::get_default_motion_constraints(const std::string &joint_name, JointMotionConstraints& constraints)
{
    size_t idx;
    try{
        idx = limits.mapNameToIndex(joint_name);
    }
    catch(std::exception e){
        LOG_ERROR("Joint %s has not been configured");
        throw e;
    }
    get_default_motion_constraints(idx, constraints);
}

bool Task::make_feasible(ConstrainedJointsTrajectory& sample)
{
    //Check if velocities have been set.
    bool velocities_set = true;
    for(size_t t=0; t<sample.getTimeSteps(); t++){
        for(size_t j=0; j<nDof; j++){
            if(IP_active->GetSelectionVectorElement(j) && !sample.elements[j][t].hasSpeed()){
                velocities_set = false;
                LOG_INFO("Not all joints have a velocity set in all time points. Will guess _all_ velocities!");
                break;
            }
        }
        if(!velocities_set)
            break;
    }

    if(!velocities_set){
        LOG_DEBUG("Trajectory has no velocities set. Will gues them.");
        set_speeds(sample, input_joint_state);
    }

    //Check if all samples are withing the constraints boundaries and correct if necessary
    bool was_feasible = sample.makeFeasible(feasibility_err);
    if(!was_feasible){
        if(throw_on_infeasible_input)
        {
            LOG_ERROR("%s", feasibility_err.str().c_str());
            throw(std::runtime_error("Target values out of bounds!"));
        }
    }

    return was_feasible;
}

size_t Task::map_name_to_joint_idx(const std::string joint_name)
{
    size_t idx;
    try{
        idx = limits.mapNameToIndex(joint_name);
    }
    catch(std::exception e){
        LOG_ERROR("Joint with name %s has not been configured in limits", joint_name.c_str());
        throw e;
    }
    return idx;
}

void Task::check_ctrl_mode(const base::JointState& command)
{
    //Check for position control mode
    if(!command.hasPosition()){
        LOG_ERROR("%s: Supports only position control mode, but input command does not provide a position value", this->getName().c_str());
        throw std::invalid_argument("Invalid control mode");
    }
}

bool Task::handle_position_target(const base::commands::Joints& sample)
{
    reset_for_new_command();

    //Create Contraint trajectory from sample with with one via point. Use default motion constraints.
    current_trajectory.resize(nDof, 1);
    //Set active joints according to sample
    set_active_joints(sample.names);

    size_t internal_joint_idx;
    for(size_t i=0; i<sample.size(); i++){

        internal_joint_idx = map_name_to_joint_idx(sample.names[i]);
        check_ctrl_mode(sample[i]);

        get_default_motion_constraints(internal_joint_idx, current_trajectory.motion_constraints[internal_joint_idx][0]);
        current_trajectory.elements[internal_joint_idx][0] = sample[i];
    }
    return make_feasible(current_trajectory);
}

bool Task::handle_constrained_position_target(const trajectory_generation::ConstrainedJointsCmd& sample)
{
    reset_for_new_command();

    //Create Contraint trajectory from sample with with one via point. Use default given motion constraints
    current_trajectory.resize(nDof, 1);
    //Set active joints according to sample
    set_active_joints(sample.names);

    size_t internal_joint_idx;
    for(size_t i=0; i<sample.size(); i++){

        internal_joint_idx = map_name_to_joint_idx(sample.names[i]);
        check_ctrl_mode(sample[i]);

        current_trajectory.elements[internal_joint_idx][0] = sample[i];
        current_trajectory.motion_constraints[internal_joint_idx][0] = sample.motion_constraints[i];
    }
    return make_feasible(current_trajectory);
}

bool Task::handle_trajectory_target(const base::JointsTrajectory& sample)
{
    reset_for_new_command();

    //Create constraint trajectory from sample with with via points form sampole. Use default motion constraints.
    current_trajectory.resize(nDof, sample.getTimeSteps());
    //Set active joints according to sample
    set_active_joints(sample.names);

    size_t internal_joint_idx;
    for(size_t i=0; i<sample.getNumberOfJoints(); i++){

        internal_joint_idx = map_name_to_joint_idx(sample.names[i]);

        for(size_t time_idx=0; time_idx<sample.getTimeSteps(); time_idx++){
            check_ctrl_mode(sample[i][time_idx]);

            get_default_motion_constraints(internal_joint_idx, current_trajectory.motion_constraints[internal_joint_idx][time_idx]);
            current_trajectory.elements[internal_joint_idx][time_idx] = sample[i][time_idx];
        }
    }
    return make_feasible(current_trajectory);
}

bool Task::handle_constrained_trajectory_target(const ConstrainedJointsTrajectory& sample)
{
    reset_for_new_command();

    //Create constraint trajectory from sample with with via points form sampole. Use default motion constraints.
    current_trajectory.resize(nDof, sample.getTimeSteps());
    //Set active joints according to sample
    set_active_joints(sample.names);

    size_t internal_joint_idx;
    for(size_t i=0; i<sample.getNumberOfJoints(); i++){

        internal_joint_idx = map_name_to_joint_idx(sample.names[i]);

        for(size_t time_idx=0; time_idx<sample.getTimeSteps(); time_idx++){
            check_ctrl_mode(sample[i][time_idx]);

            current_trajectory.motion_constraints[internal_joint_idx][time_idx] = sample.motion_constraints[i][time_idx];
            current_trajectory.elements[internal_joint_idx][time_idx] = sample[i][time_idx];
        }
    }
    return make_feasible(current_trajectory);
}

void Task::set_current_joint_state(const base::samples::Joints& sample)
{
    for(size_t i = 0; i < limits.names.size(); i++){

        const std::string& joint_name = limits.names[i];

        //See if required joint is found in sample. Throw if not
        try{
            input_joint_state.mapNameToIndex(joint_name);
        }
        catch(base::samples::Joints::InvalidName e){ //Only catch exception to write more explicit error msgs
            LOG_ERROR("Joint %s is configured in joint limits, but not available in joint state", joint_name.c_str());
            throw e;
        }

        const base::JointState& joint_state = input_joint_state.getElementByName(limits.names[i]);

        //If there was no control command previously generated, set current state from sample in any case
        //Otherwise perform check wether input should be overridden or not
        if(override_input_position && has_rml_been_called_once)
            IP_active->CurrentPositionVector->VecData[i] = OP->NewPositionVector->VecData[i];
        else{
            if(!joint_state.hasPosition()){
                LOG_ERROR("Joint %s from joint_state input does not have a position value.", joint_name.c_str());
                throw(std::runtime_error("Invalid joint sample"));
            }
            IP_active->SetCurrentPositionVectorElement(joint_state.position, i);
        }

#ifdef USING_REFLEXXES_TYPE_IV
        //Avoid invalid input here (RML with active Positonal Limits prevention crashes with positional input that is out of limits,
        //which may happen due to noisy position readings)
        if(Flags.PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
            double inbounds = IP_active->GetCurrentPositionVectorElement(i);
            if(inbounds >= limits[i].max.position){
                inbounds = limits[i].max.position - 1e-5; //Avoid numerical problems here: If the initial state is exactly at the limits, return value of RML will always be RML_ERROR_POSITIONAL_LIMITS
            }
            if(inbounds <= limits[i].min.position){
                inbounds = limits[i].min.position + 1e-5; //Avoid numerical problems here: If the initial state is exactly at the limits, return value of RML will always be RML_ERROR_POSITIONAL_LIMITS
            }
            IP_active->SetCurrentPositionVectorElement(inbounds, i);
        }
#endif
        //Only use NewVelocityVector from previous cycle if there has been a previous cycle
        if(override_input_speed && has_rml_been_called_once){
            IP_active->SetCurrentVelocityVectorElement(OP->NewVelocityVector->VecData[i], i);
        }
        else{
            if(joint_state.hasSpeed())
                IP_active->SetCurrentVelocityVectorElement(joint_state.speed, i);
            else{
                if(!override_input_speed){
                    LOG_ERROR("override_input_speed was configured to false, but joint_state does not provide a speed value for joint %s", joint_name.c_str());
                    throw(std::runtime_error("Invalid joint state input"));
                }
                IP_active->SetCurrentVelocityVectorElement(0.0, i);
            }
        }

        //Only use NewVelocityVector from previous cycle if there has been a previous cycle
        if(override_input_acceleration && has_rml_been_called_once){
            IP_active->SetCurrentAccelerationVectorElement(OP->NewAccelerationVector->VecData[i], i);
        }
        else{
            if(joint_state.hasAcceleration())
                IP_active->SetCurrentAccelerationVectorElement(joint_state.acceleration, i);
            else{
                if(!override_input_acceleration){
                    LOG_ERROR("override_input_acceleration was configured to false, but joint_state does not provide a acceleration value for joint %s", joint_name.c_str());
                    throw(std::runtime_error("Invalid joint state input"));
                }
                IP_active->SetCurrentAccelerationVectorElement(0.0, i);
            }
        }
    }
}

void Task::reset_for_new_command()
{
    current_step = 0;
    has_target = true;
    do_write_command = true;
}

void Task::handle_reflexxes_result_value(const int& result)
{

    /*
     * We need a different method to check whether a trajectory point has been reached! RML_FINAL_STATE_REACHED will be
     * overwritten by RML_ERROR_POSITIONAL_LIMITS. Thus, the current_step variable will not be increased and the
     * component gets stuck at a waypoint, with possibly non-zero velocity, which leads to unexpected behavior.
     */

    switch(result){
    case ReflexxesAPI::RML_WORKING:
        if(state() != FOLLOWING)
            state(FOLLOWING);
        break;
    case ReflexxesAPI::RML_FINAL_STATE_REACHED:
        if(state() != REACHED)
        {
            _via_point_reached.write(current_step++);
            LOG_DEBUG("Waypoint %d/%d reached", current_step, current_trajectory.getTimeSteps());
            if(current_step >= current_trajectory.getTimeSteps()){
                state(REACHED);
                LOG_DEBUG("Final waypoint reached");
            }
        }
        //After end of trajectory was reached, decide what to do, keep sending or stop
        else{
            if(_target_reached_behavior.get() == KEEP_SENDING_COMMANDS){
                do_write_command = true;
            }
            else if(_target_reached_behavior.get() == STOP_SENDING_COMMANDS){
                do_write_command = false;
            }
            else{
                LOG_ERROR_S << "Unexpected target_reached_behavior value: " << _target_reached_behavior.get();
                throw(std::runtime_error("Unexpected target_reached_behavior value."));
            }
        }
        break;
    case ReflexxesAPI::RML_ERROR:
        LOG_ERROR("RML_ERROR");
        IP_active->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES:
        LOG_ERROR("RML_ERROR_INVALID_INPUT_VALUES");
        IP_active->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION:
        LOG_ERROR("RML_ERROR_EXECUTION_TIME_CALCULATION");
        IP_active->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_SYNCHRONIZATION:
        LOG_WARN("RML_ERROR_SYNCHRONIZATION");
        IP_active->Echo();
        OP->Echo();
        //error();
        break;
    case ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS:
        LOG_ERROR("RML_ERROR_NUMBER_OF_DOFS");
        IP_active->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION:
        LOG_ERROR("RML_ERROR_NO_PHASE_SYNCHRONIZATION");
        IP_active->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_NULL_POINTER:
        LOG_ERROR("RML_ERROR_NULL_POINTER");
        IP_active->Echo();
        OP->Echo();
        error();

        break;
    case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG:
        LOG_ERROR("RML_ERROR_EXECUTION_TIME_TOO_BIG");
        IP_active->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE:
        LOG_ERROR("RML_ERROR_USER_TIME_OUT_OF_RANGE");
        IP_active->Echo();
        OP->Echo();
        error();
        break;
#ifdef USING_REFLEXXES_TYPE_IV
    case ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS:
        if(state() != IN_LIMITS){
            LOG_WARN("%s", OP->GetErrorString());
            state(IN_LIMITS);
            IP_active->Echo();
            OP->Echo();
        }
        break;
#endif
    }
}

void Task::updateHook()
{
    //Write timing information to output port
    base::Time time = base::Time::now();
    base::Time diff = time-prev_time;
    _actual_cycle_time.write(diff.toSeconds());
    prev_time = time;

    base::Time start = base::Time::now();
    TaskBase::updateHook();

    //If no joint state is avaliable, don't do anything. RML will be uninitialized otherwise
    if(_joint_state.read(input_joint_state) == RTT::NoData){ //TODO
        LOG_DEBUG("No joint state on input port");
        return;
    }
    else
        set_current_joint_state(input_joint_state);

    //Read from all the input ports. Notice, that we have implicitly a priorization here:
    //If there is data on _position_target, it will be preferred over the other ports
    if(_trajectory_target.readNewest(input_trajectory_target, false) == RTT::NewData)
        handle_trajectory_target(input_trajectory_target);

    if(_constrained_trajectory_target.readNewest( input_constrained_trajectory_target, false) == RTT::NewData)
        handle_constrained_trajectory_target(input_constrained_trajectory_target);

    if(_constrained_position_target.readNewest(input_constrained_position_target_) == RTT::NewData)
        handle_constrained_position_target(input_constrained_position_target_);

    if(_position_target.readNewest( input_position_target, false ) == RTT::NewData)
        handle_position_target(input_position_target);

    //As long as no target was received, do nothing
    if(!has_target){
        //Initialize targets for reflexxes. In case the first command is incomplete, target values will be .nan otherwise
        for(uint i = 0; i < limits.size(); i++)
        {
            rml_output_sample[i] = input_joint_state.getElementByName(limits.names[i]);
            IP_active->TargetPositionVector->VecData[i] = rml_output_sample[i].position;
            IP_active->TargetVelocityVector->VecData[i] = 0;
        }
        rml_output_sample.time = base::Time::now();
        _output_sample.write(rml_output_sample);
        return;
    }

    //Prepare data for Reflexxes
    if( current_step < current_trajectory.getTimeSteps() ){
        for( size_t i=0; i<limits.names.size(); i++ )
        {
            if(IP_active->SelectionVector->VecData[i]){
                IP_active->TargetPositionVector->VecData[i] = current_trajectory[i][current_step].position;
                IP_active->TargetVelocityVector->VecData[i] = current_trajectory[i][current_step].speed;
            }

            //Extract current motion constraints from trajectory and set them to reflexxes input parameters
            current_trajectory.getJointsMotionConstraintsAtSample(current_step, current_motion_constraints);
            set_active_motion_constraints(current_motion_constraints);
        }
    }

    //Perform control step with reflexxes
    int result = RML->RMLPosition( *IP_active, OP, Flags );
    _rml_result_value.write(result);

    double time_until_via_point = OP->GetGreatestExecutionTime();
    _time_until_via_point.write(time_until_via_point);

#ifdef USING_REFLEXXES_TYPE_IV
    if(OP->WillTheTargetPositionBeExceeded()){
         LOG_DEBUG("At least one joint of the current trajectory will exceed its target position, in order to reach the desired target state ");
    }
#endif
    has_rml_been_called_once = true;
    handle_reflexxes_result_value(result);

    //Prepare output
    for( size_t i=0; i<output_command.size(); ++i )
    {
        //Copy data to base type
        output_command[i].position = OP->GetNewPositionVectorElement(i);
        output_command[i].speed = OP->GetNewVelocityVectorElement(i);
        output_command[i].acceleration= OP->GetNewAccelerationVectorElement(i);
    }
    output_command.time = base::Time::now();

    rml_output_sample = output_command;
    rml_output_sample.time = base::Time::now();
    _output_sample.write(rml_output_sample);

    //Override output speed if required
    for(size_t i = 0; i < override_output_speed.size(); i++){
        size_t idx;
        try{
            idx = limits.mapNameToIndex(override_output_speed.names[i]);
        }
        catch(std::exception e){ //Only catch exception to give more meaningful error msg
            LOG_ERROR("override_output_speed property has joint with name %s, but this joint has not been configured in limits", override_output_speed.names[i].c_str());
            throw e;
        }
        output_command[idx].speed =  override_output_speed[i].speed;
    }

    //Override output acceleration if required
    for(size_t i = 0; i < override_output_acceleration.size(); i++){
        size_t idx;
        try{
            idx = limits.mapNameToIndex(override_output_acceleration.names[i]);
        }
        catch(std::exception e){ //Only catch exception to give more meaningful error msg
            LOG_ERROR("override_output_acceleration property has joint with name %s, but this joint has not been configured in limits", override_output_acceleration.names[i].c_str());
            throw e;
        }
        output_command[idx].acceleration =  override_output_acceleration[i].acceleration;
    }

    //Write command output
    if(do_write_command)
        _cmd.write( output_command );

    //
    // Write debug data
    //
    if(write_debug_data){

        for(uint i = 0; i < nDof; i++){
            debug_rml_input_params.CurrentPositionVector[i] = IP_active->CurrentPositionVector->VecData[i];
            debug_rml_input_params.CurrentVelocityVector[i] = IP_active->CurrentVelocityVector->VecData[i];
            debug_rml_input_params.CurrentAccelerationVector[i] = IP_active->CurrentAccelerationVector->VecData[i];
            debug_rml_input_params.TargetPositionVector[i] = IP_active->TargetPositionVector->VecData[i];
            debug_rml_input_params.TargetVelocityVector[i] = IP_active->TargetVelocityVector->VecData[i];
            debug_rml_input_params.MaxVelocityVector[i] = IP_active->MaxVelocityVector->VecData[i];
            debug_rml_input_params.MaxAccelerationVector[i] = IP_active->MaxAccelerationVector->VecData[i];
            debug_rml_input_params.MaxJerkVector[i] = IP_active->MaxJerkVector->VecData[i];
            debug_rml_input_params.SelectionVector[i] = IP_active->SelectionVector->VecData[i];

            debug_rml_output_params.ExecutionTimes[i] = OP->ExecutionTimes->VecData[i];
            debug_rml_output_params.NewPositionVector[i] = OP->NewPositionVector->VecData[i];
            debug_rml_output_params.NewVelocityVector[i] = OP->NewVelocityVector->VecData[i];
            debug_rml_output_params.NewAccelerationVector[i] = OP->NewAccelerationVector->VecData[i];
        }
        debug_rml_input_params.NumberOfDOFs = IP_active->NumberOfDOFs;
        debug_rml_input_params.MinimumSynchronizationTime = IP_active->MinimumSynchronizationTime;
        debug_rml_output_params.ANewCalculationWasPerformed = OP->ANewCalculationWasPerformed;
        debug_rml_output_params.NumberOfDOFs = OP->NumberOfDOFs;
        debug_rml_output_params.DOFWithTheGreatestExecutionTime = OP->DOFWithTheGreatestExecutionTime;
        debug_rml_output_params.SynchronizationTime = OP->SynchronizationTime;
        debug_rml_output_params.TrajectoryIsPhaseSynchronized = OP->TrajectoryIsPhaseSynchronized;

#ifdef USING_REFLEXXES_TYPE_IV
        for(uint i = 0; i < nDof; i++){
            debug_rml_input_params.MinPositionVector[i] = IP_active->MinPositionVector->VecData[i];
            debug_rml_input_params.MaxPositionVector[i] = IP_active->MaxPositionVector->VecData[i];
        }
#endif
        _rml_input_params.write(debug_rml_input_params);
        _rml_output_params.write(debug_rml_output_params);
    }
    _computation_time.write((base::Time::now() - start).toSeconds());
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
