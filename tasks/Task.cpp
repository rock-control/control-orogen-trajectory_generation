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
    throw_on_infeasible_input = true;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), current_step(0)
{
    LOG_CONFIGURE(DEBUG, stdout);
    throw_on_infeasible_input = true;
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
    // TODO:
    // Caution: This currently might result in a infeasible trajectory which will only be detected during execution.
    // This behviour comes because feasibility check is only a check for joint limits.
    // During execution it is checked if e.g. max jerk is high enough to avoid joint limits.
    double cur, prev, next, speed;
    for(size_t t=0; t<traj.getTimeSteps(); t++)
    {
        double max_speed = 0;
        for(size_t joint_idx=0; joint_idx<traj.size(); joint_idx++)
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

        //Normalize speed to 1.0 and scale to target speed
        for(size_t joint_idx=0; joint_idx<traj.size(); joint_idx++)
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

    //Verify Override:target_velcoity. If it is n ot set, set it to max velcoity of slowest joint. Otherwise checkt, that it fullfills beforementioned constraint
    if(base::isNaN<double>(override_target_velocity)){
        double slowest = 9999999999;
        for(uint i=0; i<nDof; i++){
            if(limits[i].max.speed < slowest)
                slowest = limits[i].max.speed;
        }
        override_target_velocity = slowest;
        LOG_INFO("override_target_velocity was not set. Set it to max velocity of the slowest joint. The new value is %f", override_target_velocity);
    }
    else{
        for(uint i=0; i<nDof; i++){
            if(limits[i].max.speed < override_target_velocity){
                LOG_ERROR("override_target_velocity too high. Joint %s has a max speed of %f. override_target_velocity must be lower than that value.",limits.names[i].c_str(), limits[i].max.speed);
                return false;
            }
        }
    }

    dist_to_upper.resize(nDof);
    dist_to_lower.resize(nDof);

    RML = new ReflexxesAPI( nDof, cycle_time );
    IP_static  = new RMLPositionInputParameters( nDof );
    OP  = new RMLPositionOutputParameters( nDof );
    Flags.SynchronizationBehavior = _sync_behavior.value();
    max_jerk = _max_jerk.get();
    if(max_jerk.size() != limits.size())
    {
        LOG_ERROR("Size of max jerk property is %i, but size of joint limits is %i", max_jerk.size(), limits.size());
        return false;
    }
    for(size_t i=0; i<nDof; i++){
        //Constraints
        IP_static->SetMaxVelocityVectorElement(limits[i].max.speed, i);
        IP_static->SetMaxAccelerationVectorElement(limits[i].max.effort, i);
        IP_static->SetMaxJerkVectorElement(max_jerk[i], i);
    }

#ifdef USING_REFLEXXES_TYPE_IV
    Flags.PositionalLimitsBehavior = _positional_limits_behavior.get();
    LOG_DEBUG("Joint Limits are: ");
    for(uint i = 0; i < nDof; i++)
    {
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
    assert(max_jerk.size() == nDof);
    assert(current_motion_constraints.size() == nDof);
    assert(output_command.size() == nDof);
    assert(current_trajectory.getNumberOfJoints() == nDof);
    if(write_debug_data)
        assert(debug_output_command_unmodified.size() == nDof);

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

void Task::set_active_joints(const std::vector<std::string> &joint_names)
{
    //Go throu all known joints and test if it is also in the joint_names vector.
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
    for(size_t i=0; i<motion_constraints.size(); i++){
        idx = map_joint_name_to_index(motion_constraints.names[i]);
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
        if(!base::isNaN<float>(motion_constraints[i].max.effort))
            IP_active->SetMaxAccelerationVectorElement(motion_constraints[i].max.effort, idx);

        //Only override jerk constraint, if a valid value is set
        if(!base::isNaN<float>(motion_constraints[i].max_jerk))
            IP_active->SetMaxJerkVectorElement(motion_constraints[i].max_jerk, idx);
    }
}

void Task::set_active_motion_constraints_to_default()
{
    set_active_motion_constraints(JointsMotionConstraints());
}

void Task::get_default_motion_constraints(size_t internal_index, JointMotionConstraints& constraints)
{
    constraints.min = limits[internal_index].min;
    constraints.max = limits[internal_index].max;
    constraints.max_jerk = max_jerk[internal_index];
}

void Task::get_default_motion_constraints(const std::string &joint_name, JointMotionConstraints& constraints)
{
    size_t idx = map_joint_name_to_index(joint_name);

    get_default_motion_constraints(idx, constraints);
}

bool Task::make_feasible(ConstrainedJointsTrajectory& sample)
{
    //Check if velocities have been set. If not, guess them
    base::samples::Joints first_sample;
    sample.getJointsAtTimeStep(0, first_sample);
    if(!first_sample.elements[0].hasSpeed()){
        LOG_DEBUG("Trajectory has no velocities set. Will gues them.");
        set_speeds(sample, override_target_velocity);
    }

    //Check if all samples are withing the constraints boundaries and correct if necessary
    bool was_feasible = sample.makeFeasible();
    if(!was_feasible){
        LOG_ERROR_S << "The trajectory was not within its constraints.";

        if(throw_on_infeasible_input)
            throw(std::runtime_error("Input was infeasible. Check input data against position limits."));

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
        get_default_motion_constraints(internal_idx, current_trajectory.motion_constraints[internal_idx][0]);
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
    for(size_t joint_idx=0; joint_idx<sample.getNumberOfJoints(); joint_idx++){
        internal_joint_idx = map_joint_name_to_index(sample.names[joint_idx]);
        for(size_t time_idx=0; time_idx<sample.getTimeSteps(); time_idx++){
            get_default_motion_constraints(internal_joint_idx, current_trajectory.motion_constraints[internal_joint_idx][time_idx]);
            current_trajectory.elements[internal_joint_idx][time_idx] = sample[joint_idx][time_idx];
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
    for(size_t joint_idx=0; joint_idx<sample.getNumberOfJoints(); joint_idx++){
        internal_joint_idx = map_joint_name_to_index(sample.names[joint_idx]);
        for(size_t time_idx=0; time_idx<sample.getTimeSteps(); time_idx++){
            current_trajectory.motion_constraints[internal_joint_idx][time_idx] = sample.motion_constraints[joint_idx][time_idx];
            current_trajectory.elements[internal_joint_idx][time_idx] = sample[joint_idx][time_idx];
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
            IP_active->SetCurrentPositionVectorElement(sample[index_in_sample].position, internal_index);
        }

#ifdef USING_REFLEXXES_TYPE_IV
        //Avoid invalid input here (RML with active Positonal Limits prevention has problems with positional input that is out of limits,
        //which may happen due to noisy position readings)
        //FIXME: What exactly is this 'problem'? Is it a crash or undesired behavior? If the second, what make the behavior undesireable?
        if(Flags.PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
            double inbounds = IP_active->GetCurrentPositionVectorElement(internal_index);
            if(inbounds > limits[internal_index].max.position){
                inbounds = limits[internal_index].max.position;
            }
            if(inbounds < limits[internal_index].min.position){
                inbounds = limits[internal_index].min.position;
            }
            IP_active->SetCurrentPositionVectorElement(inbounds, internal_index);
        }
#endif
        //Only use NewVelocityVector from previous cycle if there has been a previous cycle
        if(override_input_speed && has_rml_been_called_once){
            IP_active->SetCurrentVelocityVectorElement(OP->NewVelocityVector->VecData[internal_index], internal_index);
        }
        else{
            if(input_joint_state[internal_index].hasSpeed())
                IP_active->SetCurrentVelocityVectorElement(input_joint_state[index_in_sample].speed, internal_index);
            else{
                IP_active->SetCurrentVelocityVectorElement(0.0, internal_index);
                if(!override_input_speed){
                    throw(std::runtime_error("override_input_speed was configured to false, but not all joints have a speed measurement."));
                }
            }
        }

        //We can only use 'real' acceleration values from joint status if we treat the effort field as acceleration and don't want to override effort
        if(treat_effort_as_acceleration && !override_input_acceleration){
            //Effort field from joint status is treaded as accerlation and 'real' state should be used. Set it accordingly.
            IP_active->SetCurrentAccelerationVectorElement(input_joint_state[index_in_sample].effort, internal_index);
            if(!input_joint_state[index_in_sample].hasEffort()){
                throw(std::runtime_error("override_input_acceleration was configured to false, but not all joints have a acceleration measurement on effort field."));
            }
        }
        //Otherwise we must check whether there was a reference generated before. If so use it, otherwise assume 0
        else{
            if(!has_rml_been_called_once){
                //No reference acceleration was genereated before. Assume zero.
                IP_active->SetCurrentAccelerationVectorElement(0, internal_index);
            }
            else{
                //Override with reference from previous cycle
                IP_active->SetCurrentAccelerationVectorElement(OP->NewAccelerationVector->VecData[internal_index], internal_index);
            }
        }
    }
}

void Task::reset_for_new_command()
{
    current_step = 0;
    has_target = true;
}

void Task::handle_reflexxes_result_value(const int& result)
{
    switch(result){
    case ReflexxesAPI::RML_WORKING:
        if(state() != FOLLOWING)
            state(FOLLOWING);
        break;
    case ReflexxesAPI::RML_FINAL_STATE_REACHED:
        if(state() != REACHED)
        {
            _via_point_reached.write(current_step++);
            LOG_INFO("Waypoint %d/%d reached", current_step, current_trajectory.getTimeSteps());
            int steps_in_traj = current_trajectory.getTimeSteps();
            if(current_step >= current_trajectory.getTimeSteps())
                state(REACHED);
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
        std::cout<<std::endl;
        error();
        break;
    case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION:
        LOG_ERROR("RML_ERROR_EXECUTION_TIME_CALCULATION");
        IP_active->Echo();
        OP->Echo();
        error();
        break;
    case ReflexxesAPI::RML_ERROR_SYNCHRONIZATION:
        LOG_ERROR("RML_ERROR_SYNCHRONIZATION");
        IP_active->Echo();
        OP->Echo();
        error();
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
        //#ifdef USING_REFLEXXES_TYPE_IV
    case ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS:
        if(state() != IN_LIMITS){
            LOG_WARN("At least one joint is in position joint limits");
            state(IN_LIMITS);

            IP_active->Echo();
            for(size_t i=0; i<nDof; i++){
                LOG_INFO("(Joint %d) Current %.5f, Target %.f  --  Limits [%.5f, %.5f]", i, IP_active->GetCurrentPositionVectorElement(i), IP_active->GetTargetPositionVectorElement(i), IP_active->GetMinPositionVectorElement(i), IP_active->GetMaxPositionVectorElement(i));
            }
            std::cout<<std::endl;
        }
        break;
        //#endif
    }
}

void Task::updateHook()
{
    TaskBase::updateHook();

    //Read from all the input ports. Notice, that we have implicitly a priorization here:
    //If there is data on _position_target, it will be preferred over the other two ports
    if(_trajectory_target.readNewest(input_trajectory_target, false) == RTT::NewData){
        LOG_DEBUG("Received a new target on trajectory input port");
        reset_for_new_command();
        handle_trajectory_target(input_trajectory_target);
    }

    if(_constrained_trajectory_target.read( input_constrained_trajectory_target, false) == RTT::NewData){
        LOG_DEBUG("Received a new target on constrained trajectory input port");
        reset_for_new_command();
        handle_constrained_trajectory_target(input_constrained_trajectory_target);
    }

    if(_position_target.read( input_position_target, false ) == RTT::NewData){
        LOG_DEBUG("Received a new target on trajectory input port");
        reset_for_new_command();
        handle_position_target(input_position_target);
    }


    //If no joint state is avaliable, don't do anything. RML will be uninitialized otherwise
    if(_joint_state.readNewest(input_joint_state) == RTT::NoData){
        if(has_target)
            LOG_ERROR("No data on joint state port");
        if(!state() == NO_JOINT_STATE_INPUT)
            error(NO_JOINT_STATE_INPUT);
        return;
    }
    else{
        assert(output_command.size() == nDof);
        //Set current joint state from sample
        set_current_joint_state(input_joint_state);

        //Pre-initialize output command with current joint state
        for(size_t input_index = 0; input_index<input_joint_state.size(); input_index++){
            size_t internal_index = map_joint_name_to_index(input_joint_state.names[input_index]);
            output_command[internal_index].position = input_joint_state[input_index].position;
            output_command[internal_index].speed = input_joint_state[input_index].speed;
            output_command[internal_index].effort= input_joint_state[input_index].effort;
        }
    }


    //As long as no target was received, do nothing
    if(!has_target)
        return;

    //Prepare data for Reflexxes
    if( current_step < current_trajectory.getTimeSteps() ){
        for( size_t i=0; i<limits.names.size(); i++ )
        {
            IP_active->TargetPositionVector->VecData[i] = current_trajectory[i][current_step].position;
            IP_active->TargetVelocityVector->VecData[i] = current_trajectory[i][current_step].speed;

            //Extract current motion constraints from trajectory and set them to reflexxes input parameters
            current_trajectory.getJointsMotionConstraintsAtSample(current_step, current_motion_constraints);
            set_active_motion_constraints(current_motion_constraints);
        }
    }

    //FIXME: Deleted a bunch of code dealing with 'allow_positive' and 'allow_negative'.
    //       It seems rather hacky, what was it for? I don't think it should be here.

    //Perform control step with reflexxes
    int result = RML->RMLPosition( *IP_active, OP, Flags );
    RMLDoubleVector min_position_vector(nDof);
    RMLDoubleVector max_position_vector(nDof);
    OP->GetPositionalExtrema(&min_position_vector, &max_position_vector);

    double time_until_via_point = OP->GetGreatestExecutionTime();
    _time_until_via_point.write(time_until_via_point);

    if(OP->WillTheTargetPositionBeExceeded()){
         LOG_WARN("The target position will be exceeded");
         for(size_t i=0; i<1; i++){
             LOG_WARN("Joint %d, execution time: %.5f\n     min/max: %.5f / %.5f, \n    target: %.5f, %.5f \n    current: %.5f, %.5f, %.5f\n",
                      i, time_until_via_point,
                      min_position_vector.VecData[i], max_position_vector.VecData[i],
                      IP_active->GetTargetPositionVectorElement(i),
                      IP_active->GetTargetVelocityVectorElement(i),
                      IP_active->GetCurrentPositionVectorElement(i),
                      IP_active->GetCurrentVelocityVectorElement(i),
                      IP_active->GetCurrentAccelerationVectorElement(i));
         }
    }
    has_rml_been_called_once = true;
    handle_reflexxes_result_value(result);

    //Prepare output
    assert(output_command.size() == nDof);
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
        output_command[i].position = OP->GetNewPositionVectorElement(i);
        output_command[i].speed = OP->GetNewVelocityVectorElement(i);
        output_command[i].effort= OP->GetNewAccelerationVectorElement(i);
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
        for(uint i = 0; i < nDof; i++)
        {
            dist_to_upper(i) = limits[i].max.position - IP_active->CurrentPositionVector->VecData[i];
            dist_to_lower(i) = IP_active->CurrentPositionVector->VecData[i] - limits[i].min.position;
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
