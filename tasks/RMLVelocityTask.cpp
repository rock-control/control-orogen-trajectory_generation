/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLVelocityTask.hpp"
#include <base/Logging.hpp>
using namespace trajectory_generation;
using namespace std;

RMLVelocityTask::RMLVelocityTask(std::string const& name)
    : RMLVelocityTaskBase(name)
{
    has_target_ = has_rml_been_called_ = false;
}

RMLVelocityTask::RMLVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLVelocityTaskBase(name, engine)
{
    has_target_ = has_rml_been_called_ = false;
}

RMLVelocityTask::~RMLVelocityTask()
{
}

bool RMLVelocityTask::configureHook()
{
    if (! RMLVelocityTaskBase::configureHook())
        return false;

    override_input_position_ = _override_input_position.value();
    override_input_speed_ = _override_input_speed.value();
    override_input_acceleration_ = _override_input_acceleration.value();
    override_output_acceleration_ = _override_output_acceleration.value();
    limits_ = _limits.value();
    cycle_time_ = _cycle_time.value();
    Vel_Flags_.SynchronizationBehavior = _sync_behavior.value();
    throw_on_infeasible_input_ = _throw_on_infeasible_input.get();
    velocity_timeout_ = _velocity_timeout.get();

    nDOF_ =  limits_.size();

    if(limits_.empty()){
        LOG_ERROR("Size of joint limits has to be > 0!");
        return false;
    }

    for(uint i = 0; i < limits_.size(); i++)
    {
        if(!limits_[i].max.hasSpeed() || limits_[i].max.speed <= 0){
            LOG_ERROR("Limits for joint %i (%s) do not have a valid max speed value", i, limits_.names[i].c_str());
            return false;
        }
        if(!limits_[i].max.hasAcceleration() || limits_[i].max.acceleration <= 0){
            LOG_ERROR("Limits for joint %i (%s) do not have a valid max acceleration value", i, limits_.names[i].c_str());
            return false;
        }
        if(base::isUnset(limits_[i].max_jerk) || limits_[i].max_jerk <= 0){
            LOG_ERROR("Limits for joint %i (%s) do not have a valid max jerk value", i, limits_.names[i].c_str());
            return false;
        }
#ifdef USING_REFLEXXES_TYPE_IV
        if(!limits_[i].max.hasPosition()){
            LOG_ERROR("Limits for joint %i (%s) do not have a valid max position value", i, limits_.names[i].c_str());
            return false;
        }
        if(!limits_[i].min.hasPosition()){
            LOG_ERROR("Limits for joint %i (%s) do not have a valid min position value", i, limits_.names[i].c_str());
            return false;
        }
#endif
    }

    status_.resize(nDOF_);
    output_sample_.resize(nDOF_);
    output_sample_.names = limits_.names;
    command_out_.resize(nDOF_);
    command_out_.names = limits_.names;

    input_params_ = RMLInputParams(nDOF_);
    output_params_ = RMLOutputParams(nDOF_);
    RML_ = new ReflexxesAPI(nDOF_, cycle_time_);
    Vel_IP_ = new RMLVelocityInputParameters(nDOF_);
    Vel_OP_ = new RMLVelocityOutputParameters(nDOF_);

#ifdef USING_REFLEXXES_TYPE_IV
    Vel_Flags_.PositionalLimitsBehavior = _positional_limits_behavior.get();
#endif

    setActiveMotionConstraints(limits_);

    return true;
}

bool RMLVelocityTask::startHook()
{
    if (! RMLVelocityTaskBase::startHook())
        return false;
    has_target_ = has_rml_been_called_ = false;

    for(size_t i = 0; i < nDOF_; i++)
        Vel_IP_->TargetVelocityVector->VecData[i] = 0;
    wrote_velocity_timeout_warning_ = false;

    return true;
}

void RMLVelocityTask::handleStatusInput(const base::samples::Joints &status)
{
    for(size_t i = 0; i < nDOF_; i++)
    {
        size_t joint_idx = 0;
        try{
            joint_idx = status.mapNameToIndex(limits_.names[i]);
        }
        catch(std::exception e){
            continue;
        }

        //Only use NewPositionVector from previous cycle if there has been a previous cycle (has_rml_been_called_ == true)
        if(override_input_position_ && has_rml_been_called_)
            Vel_IP_->CurrentPositionVector->VecData[i] = Vel_OP_->NewPositionVector->VecData[i];
        else{
            if(!status[joint_idx].hasPosition()){
                LOG_ERROR("Position of input joint state of joint %i (%s) is unset", i, limits_.names[i].c_str());
                throw std::invalid_argument("Invalid joint state input");
            }
            Vel_IP_->CurrentPositionVector->VecData[i] = status[joint_idx].position;
        }

        //Avoid invalid input here (RML with active Positonal Limits prevention crashes with positional input that is out of limits, which may happen due to noisy position readings)
#ifdef USING_REFLEXXES_TYPE_IV
        if(Vel_Flags_.PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT)
        {
            double new_position = std::max(std::min(Vel_IP_->MaxPositionVector->VecData[i], Vel_IP_->CurrentPositionVector->VecData[i]), Vel_IP_->MinPositionVector->VecData[i]);
            Vel_IP_->CurrentPositionVector->VecData[i] = new_position;
        }
#endif
        //Only use NewVelocityVector from previous cycle if there has been a previous cycle (has_rml_been_called_ == true)
        if(override_input_speed_ && has_rml_been_called_)
            Vel_IP_->CurrentVelocityVector->VecData[i] = Vel_OP_->NewVelocityVector->VecData[i];
        else{
            if(status_[joint_idx].hasSpeed())
                Vel_IP_->CurrentVelocityVector->VecData[i] = status[joint_idx].speed;
            else{
                if(!override_input_speed_){
                    LOG_ERROR("Override input speed is false and speed of input joint state of joint %i (%s) is unset", i, limits_.names[i].c_str());
                    throw std::invalid_argument("Invalid joint state input");
                }
                Vel_IP_->CurrentVelocityVector->VecData[i] = 0;
            }
        }

#ifndef USING_REFLEXXES_TYPE_IV
        // This is to fix a bug in the Reflexxes type II library. If current and target velocity are equal
        // the output will be nan. TODO: Has this already been fixed in reflexxes?
        if(Vel_IP_->CurrentVelocityVector->VecData[i] == Vel_IP_->TargetVelocityVector->VecData[i])
            Vel_IP_->CurrentVelocityVector->VecData[i] -= 1e-10;
#endif

        // Input acceleration

        //Only use NewAccelerationVector from previous cycle if there has been a previous cycle (has_rml_been_called_ == true)
        if(override_input_acceleration_ && has_rml_been_called_)
            Vel_IP_->CurrentAccelerationVector->VecData[i] = Vel_OP_->NewAccelerationVector->VecData[i];
        else{
            if(status_[joint_idx].hasAcceleration())
                Vel_IP_->CurrentAccelerationVector->VecData[i] = status[joint_idx].acceleration;
            else{
                if(!override_input_acceleration_){
                    LOG_ERROR("Override input acceleration is false and acceleration of input joint state of joint %i (%s) is unset", i, limits_.names[i].c_str());
                    throw std::invalid_argument("Invalid joint state input");
                }
                Vel_IP_->CurrentAccelerationVector->VecData[i] = 0;
            }
        }
    }
}

void RMLVelocityTask::handleCommandInput(const base::commands::Joints &command)
{
    for(size_t i = 0; i < nDOF_; i++)
        Vel_IP_->SelectionVector->VecData[i] = false;

    // Use name mapping to allow partial inputs.
    size_t joint_idx = 0;
    for(size_t i = 0; i < command_in_.size(); i++)
    {
        try{
            joint_idx = limits_.mapNameToIndex(command.names[i]);
        }
        catch(std::exception e){
            continue;
        }

        if(!command_in_[i].hasSpeed()){
            LOG_ERROR("%s: supports only speed control mode, but input command does not provide a speed value", this->getName().c_str());
            throw std::invalid_argument("Invalid control mode");
        }

        //Actively avoid speed limits here:
        Vel_IP_->TargetVelocityVector->VecData[joint_idx] =
                std::max(std::min(input_params_.MaxVelocityVector[joint_idx], (double)command[i].speed), -input_params_.MaxVelocityVector[joint_idx]);

#ifdef  USING_REFLEXXES_TYPE_IV
        if(Vel_Flags_.PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
            double cur_pos = Vel_IP_->CurrentPositionVector->VecData[joint_idx];
            double target_vel = Vel_IP_->TargetVelocityVector->VecData[joint_idx];
            double max_pos = Vel_IP_->MaxPositionVector->VecData[joint_idx];
            double min_pos = Vel_IP_->MinPositionVector->VecData[joint_idx];
            if( (target_vel*cycle_time_ + cur_pos > max_pos) || (target_vel*cycle_time_ + cur_pos < min_pos) )
                Vel_IP_->TargetVelocityVector->VecData[joint_idx] = 0;
        }
#endif

        Vel_IP_->SelectionVector->VecData[joint_idx] = true;
    }
}

void RMLVelocityTask::setActiveMotionConstraints(const trajectory_generation::JointsMotionConstraints& constraints)
{
    double cur_pos, max_pos, min_pos, max_speed, max_acc, max_jerk;
    size_t idx;

    for(size_t i = 0; i < constraints.size(); i++)
    {
        try{
            idx = limits_.mapNameToIndex(constraints.names[i]);
        }
        catch(std::exception e)
        {
            LOG_ERROR("%s: SetMotionConstraints: Joint %s is given in input constraints, but has not been configured. ",
                      this->getName().c_str(), constraints.names[i].c_str());
            throw std::invalid_argument("Invalid constraint input");
        }

#ifdef USING_REFLEXXES_TYPE_IV
        // Set Position limits: If input is nan, reset to initial min/max pos
        if(base::isNaN(constraints[i].max.position))
            max_pos = limits_[idx].max.position;
        else
            max_pos = constraints[i].max.position;

        if(base::isNaN(constraints[i].min.position))
            min_pos = limits_[idx].min.position;
        else
            min_pos = constraints[i].min.position;

        if(max_pos <= min_pos){
            LOG_ERROR("Cannot set max and min pos of joint %i (%s) to %f and %f. Max pos has to be >= min pos!", idx, limits_.names[idx].c_str(), max_pos, min_pos);
            throw std::invalid_argument("Invalid constraint input");
        }

        cur_pos = Vel_IP_->CurrentPositionVector->VecData[idx];
        if(has_rml_been_called_)
        {
            if(cur_pos > max_pos){
                LOG_ERROR("Cannot set max pos of joint %i (%s) to %f, because current pos is %f!", idx, limits_.names[idx].c_str(), max_pos, cur_pos);
                throw std::invalid_argument("Invalid constraint input");
            }

            if(cur_pos < min_pos){
                LOG_ERROR("Cannot set min pos of joint %i (%s) to %f, because current pos is %f!", idx, limits_.names[idx].c_str(), min_pos, cur_pos);
                throw std::invalid_argument("Invalid constraint input");
            }
        }

        Vel_IP_->MaxPositionVector->VecData[idx] = max_pos;
        Vel_IP_->MinPositionVector->VecData[idx] = min_pos;
#endif

        // Set Max Velocity: If input is nan, reset to initial max velocity
        if(base::isNaN(constraints[i].max.speed))
            max_speed = limits_[idx].max.speed;
        else
            max_speed = constraints[i].max.speed;

        if(max_speed <= 0){
            LOG_ERROR("Cannot set max speed of joint %i (%s) to %f. Max speed has to be > 0!", idx, limits_.names[idx].c_str(), max_speed);
            throw std::invalid_argument("Invalid constraint input");
        }
        input_params_.MaxVelocityVector[idx] = max_speed;

        // Set Max Acceleration: If input is nan, reset to initial max acceleration
        if(base::isNaN(constraints[i].max.acceleration))
            max_acc = limits_[idx].max.acceleration;
        else
            max_acc = constraints[i].max.acceleration;

        if(max_acc <= 0){
            LOG_ERROR("Cannot set max acceleration of joint %i (%s) to %f. Max acceleration has to be > 0!", idx, limits_.names[idx].c_str(), max_acc);
            throw std::invalid_argument("Invalid constraint input");
        }
        Vel_IP_->MaxAccelerationVector->VecData[idx] = max_acc;

        // Set Max Jerk: If input is nan, reset to initial max jerk
        if(base::isNaN(constraints[i].max_jerk))
            max_jerk = limits_[idx].max_jerk;
        else
            max_jerk = constraints[i].max_jerk;

        if(max_jerk <= 0){
            LOG_ERROR("Cannot set max jerk of joint %i (%s) to %f. Max jerk has to be > 0!", idx, limits_.names[idx].c_str(), max_jerk);
            throw std::invalid_argument("Invalid constraint input");
        }
        Vel_IP_->MaxJerkVector->VecData[idx] = max_jerk;
    }
}

void RMLVelocityTask::handleRMLInterpolationResult(const int res)
{
    switch(res){
    case ReflexxesAPI::RML_WORKING:
        if(!state() == FOLLOWING)
            state(FOLLOWING);
        break;
    case ReflexxesAPI::RML_FINAL_STATE_REACHED:
        if(!state() == REACHED)
            state(REACHED);
        break;
#ifdef USING_REFLEXXES_TYPE_IV
    case ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS:
        if(!state() == IN_LIMITS)
            state(IN_LIMITS);
        break;
    default:
        LOG_ERROR("Reflexxes returned error %s", Vel_OP_->GetErrorString());
        throw std::runtime_error("Reflexxes runtime error");
#else
    default:
        throw std::runtime_error("Reflexxes runtime error");
#endif
    }
}

void RMLVelocityTask::writeDebug(const RMLVelocityInputParameters* in, const RMLVelocityOutputParameters* out)
{
    base::Time time = base::Time::now();
    base::Time diff = time-prev_time_;
    _actual_cycle_time.write(diff.toSeconds());
    prev_time_ = time;

    for(size_t i = 0; i < nDOF_; i++){
        input_params_.CurrentPositionVector[i] = in->CurrentPositionVector->VecData[i];
        input_params_.CurrentVelocityVector[i] = in->CurrentVelocityVector->VecData[i];
        input_params_.CurrentAccelerationVector[i] = in->CurrentAccelerationVector->VecData[i];
        input_params_.TargetVelocityVector[i] = in->TargetVelocityVector->VecData[i];
        input_params_.MaxAccelerationVector[i] = in->MaxAccelerationVector->VecData[i];
        input_params_.MaxJerkVector[i] = in->MaxJerkVector->VecData[i];
        input_params_.SelectionVector[i] = in->SelectionVector->VecData[i];

        output_params_.ExecutionTimes[i] = out->ExecutionTimes->VecData[i];
        output_params_.NewPositionVector[i] = out->NewPositionVector->VecData[i];
        output_params_.NewVelocityVector[i] = out->NewVelocityVector->VecData[i];
        output_params_.NewAccelerationVector[i] = out->NewAccelerationVector->VecData[i];
        output_params_.PositionValuesAtTargetVelocity[i] = out->PositionValuesAtTargetVelocity->VecData[i];
    }
    input_params_.NumberOfDOFs = in->NumberOfDOFs;
    input_params_.MinimumSynchronizationTime = in->MinimumSynchronizationTime;
    output_params_.ANewCalculationWasPerformed = out->ANewCalculationWasPerformed;
    output_params_.NumberOfDOFs = out->NumberOfDOFs;
    output_params_.DOFWithTheGreatestExecutionTime = out->DOFWithTheGreatestExecutionTime;
    output_params_.SynchronizationTime = out->SynchronizationTime;
    output_params_.TrajectoryIsPhaseSynchronized = out->TrajectoryIsPhaseSynchronized;

#ifdef USING_REFLEXXES_TYPE_IV
    for(size_t i = 0; i < nDOF_; i++)
    {
        input_params_.MinPositionVector[i] = in->MinPositionVector->VecData[i];
        input_params_.MaxPositionVector[i] = in->MaxPositionVector->VecData[i];
    }
#endif

    _rml_input_params.write(input_params_);
    _rml_output_params.write(output_params_);
}

void RMLVelocityTask::writeOutputCommand(const RMLVelocityOutputParameters* output)
{
    for(size_t i = 0; i < command_out_.size(); i++)
    {
        std::string joint_name = command_out_.names[i].c_str();
        command_out_[i].speed = output->NewVelocityVector->VecData[i];
        command_out_[i].acceleration = output->NewAccelerationVector->VecData[i];
    }

    // Acceleration overrides
    for(size_t i = 0; i < override_output_acceleration_.size(); i++)
    {
        uint idx = command_out_.mapNameToIndex(override_output_acceleration_.names[i]);
        command_out_[idx].acceleration = override_output_acceleration_[i].acceleration;
    }

    command_out_.time = base::Time::now();
    _command.write(command_out_);
}

void RMLVelocityTask::writeOutputSample(const base::samples::Joints& status, const RMLVelocityOutputParameters* output)
{
    if(has_rml_been_called_)
    {
        for(size_t i = 0; i < command_out_.size(); i++)
        {
            output_sample_[i].position = output->NewPositionVector->VecData[i];
            output_sample_[i].speed = output->NewVelocityVector->VecData[i];
            output_sample_[i].acceleration = output->NewAccelerationVector->VecData[i];
        }
    }
    else
    {
        for(size_t i = 0; i < nDOF_; i++){
            std::string joint_name = limits_.names[i];
            size_t joint_idx = 0;
            try{
                joint_idx = status.mapNameToIndex(joint_name);
            }
            catch(std::exception e){
                continue;
            }

            output_sample_[i].position = status[joint_idx].position;
            output_sample_[i].speed = status[joint_idx].speed;
            output_sample_[i].acceleration = status[joint_idx].acceleration;
        }
    }
    output_sample_.time = base::Time::now();
    _output_sample.write(output_sample_);
}

void RMLVelocityTask::updateHook()
{
    RMLVelocityTaskBase::updateHook();

    if(_joint_state.read(status_) == RTT::NoData){
        LOG_DEBUG("No data on joint state port");
        return;
    }

    handleStatusInput(status_);

    while(_velocity_target.read(command_in_) == RTT::NewData){
        handleCommandInput(command_in_);
        stamp_ = base::Time::now();
        has_target_ = true;
    }

    while(_constrained_velocity_target.read(constrained_cmd_in_) == RTT::NewData){
        command_in_.names = constrained_cmd_in_.names;
        command_in_.elements = constrained_cmd_in_.elements;
        setActiveMotionConstraints(constrained_cmd_in_.motion_constraints);
        handleCommandInput(command_in_);
        stamp_ = base::Time::now();
        has_target_ = true;
    }


    if(has_target_)
    {
        base::Time cur = base::Time::now();
        double diff = (cur - stamp_).toSeconds();
        if(diff > velocity_timeout_){
            if(!wrote_velocity_timeout_warning_){
                LOG_WARN("Velocity Timeout. No new reference arrived for %f seconds. Setting reference velocity to zero", diff);
                wrote_velocity_timeout_warning_ = true;
            }
            for(uint i = 0; i < nDOF_; i++)
                Vel_IP_->TargetVelocityVector->VecData[i] = 0;
        }
        else
            wrote_velocity_timeout_warning_ = false;

        handleRMLInterpolationResult(RML_->RMLVelocity(*Vel_IP_, Vel_OP_, Vel_Flags_));
        has_rml_been_called_ = true;

        writeOutputCommand(Vel_OP_);
    }

    writeOutputSample(status_, Vel_OP_);
    writeDebug(Vel_IP_, Vel_OP_);
}

void RMLVelocityTask::errorHook()
{
    RMLVelocityTaskBase::errorHook();
}

void RMLVelocityTask::stopHook()
{
    RMLVelocityTaskBase::stopHook();
}

void RMLVelocityTask::cleanupHook()
{
    RMLVelocityTaskBase::cleanupHook();

    delete RML_;
    delete Vel_IP_;
    delete Vel_OP_;

    command_out_.clear();
    status_.clear();
    limits_.clear();
}
