/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLVelocityTask.hpp"
#include <base/logging.h>
using namespace trajectory_generation;
using namespace std;

RMLVelocityTask::RMLVelocityTask(std::string const& name)
    : RMLVelocityTaskBase(name)
{
}

RMLVelocityTask::RMLVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLVelocityTaskBase(name, engine)
{
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
    limits_ = _limits.value();
    cycle_time_ = _cycle_time.value();
    Vel_Flags_.SynchronizationBehavior = _sync_behavior.value();

    nDOF_ =  limits_.size();

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

    return true;
}

void RMLVelocityTask::handleStatusInput(const base::samples::Joints &status)
{
    for(size_t i = 0; i < nDOF_; i++)
    {
        std::string joint_name = limits_.names[i];
        size_t joint_idx = 0;
        try{
            joint_idx = status.mapNameToIndex(joint_name);
        }
        catch(std::exception e){
            continue;
        }

        // Input Position

        //Only use NewPositionVector from previous cycle if there has been a previous cycle (has_rml_been_called_ == true)
        if(override_input_position_ && has_rml_been_called_)
            Vel_IP_->CurrentPositionVector->VecData[i] = Vel_OP_->NewPositionVector->VecData[i];
        else
            Vel_IP_->CurrentPositionVector->VecData[i] = status[joint_idx].position;

        //Avoid invalid input here (RML with active Positonal Limits prevention has problems with positional input that is out of limits,
        //which may happen due to noisy position readings)
#ifdef USING_REFLEXXES_TYPE_IV
        if(Vel_Flags_.PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT)
        {
            double new_position = std::max(std::min(Vel_IP_->MaxPositionVector->VecData[i], Vel_IP_->CurrentPositionVector->VecData[i]), Vel_IP_->MinPositionVector->VecData[i]);
            Vel_IP_->CurrentPositionVector->VecData[i] = new_position;
        }
#endif
        // Input Velocity

        //Only use NewVelocityVector from previous cycle if there has been a previous cycle (has_rml_been_called_ == true)
        if(override_input_speed_ && has_rml_been_called_)
            Vel_IP_->CurrentVelocityVector->VecData[i] = Vel_OP_->NewVelocityVector->VecData[i];
        else
            Vel_IP_->CurrentVelocityVector->VecData[i] = status[joint_idx].speed;

#ifndef USING_REFLEXXES_TYPE_IV
        // This is to fix a bug in the Reflexxes type II library. If current and target velocity are equal
        // the output will be nan.
        if(Vel_IP_->CurrentVelocityVector->VecData[i] == Vel_IP_->TargetVelocityVector->VecData[i])
            Vel_IP_->CurrentVelocityVector->VecData[i] -= 1e-10;
#endif

        // Input acceleration

        //Only use NewAccelerationVector from previous cycle if there has been a previous cycle (has_rml_been_called_ == true)
        if(override_input_acceleration_ && has_rml_been_called_)
            Vel_IP_->CurrentAccelerationVector->VecData[i] = Vel_OP_->NewAccelerationVector->VecData[i];
        else
            Vel_IP_->CurrentAccelerationVector->VecData[i] = status[joint_idx].acceleration;
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

        Vel_IP_->SelectionVector->VecData[joint_idx] = true;
    }
}

void RMLVelocityTask::setActiveMotionConstraints(const trajectory_generation::JointsMotionConstraints& constraints)
{
    for(size_t i = 0; i < constraints.size(); i++)
    {
        size_t idx;
        try{
            idx = limits_.mapNameToIndex(constraints.names[i]);
        }
        catch(std::exception e)
        {
            LOG_WARN("%s: SetMotionConstraints: Joint %s has not been configured. Will ignore this joint",
                      this->getName().c_str(), constraints.names[i].c_str());
            continue;
        }

        // Set Position limits: If input is nan, reset to initial min/max pos

#ifdef USING_REFLEXXES_TYPE_IV
        double max_pos;
        if(base::isNaN(constraints[i].max.position))
            max_pos = limits_[idx].max.position;
        else
            max_pos = constraints[i].max.position;

        double cur_pos = Vel_IP_->CurrentPositionVector->VecData[idx];
        if(cur_pos > max_pos){
            LOG_WARN("Cannot set max pos to %f because current pos is %f", max_pos, cur_pos);
        }
        else{
            Vel_IP_->MaxPositionVector->VecData[idx] = max_pos;
        }

        double min_pos;
        if(base::isNaN(constraints[i].min.position)){
            min_pos = limits_[idx].min.position;
        }
        else{
            min_pos = constraints[i].min.position;
        }

        if(cur_pos < min_pos){
            LOG_WARN("Cannot set min pos to %f because current pos is %f", min_pos, cur_pos);
        }
        else{
            Vel_IP_->MinPositionVector->VecData[idx] = min_pos;
        }
#endif

        // Set Max Velocity: If input is nan, reset to initial max velocity

        double max_vel;
        if(base::isNaN(constraints[i].max.speed)){
            max_vel = limits_[idx].max.speed;
        }
        else{
            max_vel = constraints[i].max.speed;
        }
        input_params_.MaxVelocityVector[idx] = max_vel;


         // Set Max Acceleration: If input is nan, reset to initial max acceleration

        double max_acc;
        if(base::isNaN(constraints[i].max.acceleration)){
            max_acc = limits_[idx].max.acceleration;
        }
        else{
            max_acc = constraints[i].max.acceleration;
        }
        Vel_IP_->MaxAccelerationVector->VecData[idx] = max_acc;

        // Set Max Jerk: If input is nan, reset to initial max jerk

        double max_jerk;
        if(base::isNaN(constraints[i].max_jerk))
            max_jerk = limits_[idx].max_jerk;
        else{
            max_jerk = constraints[i].max_jerk;
        }

        Vel_IP_->MaxJerkVector->VecData[idx] = max_jerk;
    }//for loop
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
