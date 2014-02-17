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
    override_input_effort_ = _override_input_effort.value();

    limits_ = _limits.get();
    cycle_time_ = _cycle_time.get();
    Vel_Flags_.SynchronizationBehavior = _sync_behavior.get();
    std::vector<double> max_jerk = _max_jerk.get();
    max_effort_scale_ = _max_effort_scale.get();
    max_jerk_scale_ = _max_jerk_scale.get();
    timeout_ = _timeout.get();

    if(max_jerk.size() != limits_.size())
    {
        LOG_ERROR("No of joints of interpolator is %i but size of max_jerk is %i", limits_.size(), max_jerk.size());
        return false;
    }

    nDOF_ = limits_.size();

    status_.resize(nDOF_);
    command_out_.resize(nDOF_);
    command_out_.names = limits_.names;
    input_params_ = RMLVelocityInputParams(nDOF_);
    output_params_ = RMLVelocityOutputParams(nDOF_);

    RML_ = new ReflexxesAPI(nDOF_, cycle_time_);
    Vel_IP_ = new RMLVelocityInputParameters(nDOF_);
    Vel_OP_ = new RMLVelocityOutputParameters(nDOF_);

    for(uint i = 0; i < nDOF_; i++)
    {
        if(limits_[i].max.effort <= 0 || limits_[i].max.speed <= 0){
            LOG_ERROR("Max Effort and max speed of limits property must be grater than zero");
            return false;
        }
        command_out_[i].speed = command_out_[i].effort = 0;
        Vel_IP_->MaxAccelerationVector->VecData[i] = limits_[i].max.effort * max_effort_scale_;
        Vel_IP_->MaxJerkVector->VecData[i] = max_jerk[i] * max_jerk_scale_;
        Vel_IP_->SelectionVector->VecData[i] = true;
        Vel_IP_->TargetVelocityVector->VecData[i] = 0;
    }

#ifdef USING_REFLEXXES_TYPE_IV
    Vel_Flags_.PositionalLimitsBehavior = _positional_limits_behavior.get();
    for(uint i = 0; i < nDOF_; i++)
    {
        Vel_IP_->MaxPositionVector->VecData[i] = limits_[i].max.position;
        Vel_IP_->MinPositionVector->VecData[i] = limits_[i].min.position;
    }
#endif

    is_initialized_ = false;

    return true;
}

bool RMLVelocityTask::startHook()
{
    if (! RMLVelocityTaskBase::startHook())
        return false;
    stamp_ = base::Time::now();
    return true;
}

void RMLVelocityTask::updateHook()
{
    base::Time start = base::Time::now();

    RMLVelocityTaskBase::updateHook();

    if(_joint_state.read(status_) == RTT::NoData){
        LOG_DEBUG("No data on joint state port");
        return;
    }

    //
    // Single Velocity Command input
    //
    while(_velocity_target.read(command_in_) == RTT::NewData){

        stamp_ = base::Time::now();
        // Use name mapping to allow partial inputs.
        for(uint i = 0; i < command_in_.size(); i++){

            uint joint_idx = 0;
            try{
                joint_idx = limits_.mapNameToIndex(command_in_.names[i]);
            }
            catch(std::exception e){
                continue;
            }

            if(command_in_[i].getMode() != base::JointState::SPEED){
                LOG_ERROR("Supports only speed control mode, but input has control mode %i", command_in_[i].getMode());
                throw std::invalid_argument("Invalid control mode");
            }

            //Actively avoid speed limits here:
            Vel_IP_->TargetVelocityVector->VecData[joint_idx] =
                    std::max(std::min(limits_[joint_idx].max.speed, command_in_[i].speed), -limits_[joint_idx].max.speed);
        }
    }

    double difftime = (base::Time::now() - stamp_).toSeconds();
    if(difftime > timeout_){
        LOG_DEBUG("Watchdog: Last reference value arrived %f seconds ago, Timeout is %f. Setting reference to zero", difftime, timeout_);
        stamp_ = base::Time::now();
        for(uint i = 0; i < nDOF_; i++)
            Vel_IP_->TargetVelocityVector->VecData[i] = 0;
    }

    //
    // Compute next sample
    //
    for(uint i = 0; i < nDOF_; i++){

        uint joint_idx = 0;
        try{
            joint_idx = status_.mapNameToIndex(limits_.names[i]);
        }
        catch(std::exception e){
            continue;
        }

        //Only use NewPositionVector from previous cycle if there has been a previous cycle (is_initialized_ == true)
        if(override_input_position_ && is_initialized_)
            Vel_IP_->CurrentPositionVector->VecData[i] = Vel_OP_->NewPositionVector->VecData[i];
        else
            Vel_IP_->CurrentPositionVector->VecData[i] = status_[joint_idx].position;

#ifdef USING_REFLEXXES_TYPE_IV
            //Avoid invalid input here (RML with active Positonal Limits prevention has problems with positional input that is out of limits,
            //which may happen due to noisy position readings)
            if(Vel_Flags_.PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT)
                Vel_IP_->CurrentPositionVector->VecData[i] = std::max(std::min(limits_[i].max.position, Vel_IP_->CurrentPositionVector->VecData[i]), limits_[i].min.position);
#endif

        //Only use NewVelocityVector from previous cycle if there has been a previous cycle (is_initialized_ == true)
        if(override_input_speed_ && is_initialized_)
            Vel_IP_->CurrentVelocityVector->VecData[i] = Vel_OP_->NewVelocityVector->VecData[i];
        else
            Vel_IP_->CurrentVelocityVector->VecData[i] = status_[joint_idx].speed;

#ifndef USING_REFLEXXES_TYPE_IV
        // This is to fix a bug in the Reflexxes type II library. If current and target velocity are equal
        // the output will be nan.
        if(Vel_IP_->CurrentVelocityVector->VecData[i] == Vel_IP_->TargetVelocityVector->VecData[i])
            Vel_IP_->CurrentVelocityVector->VecData[i] -= 1e-10;
#endif

        ///Only use NewAccelerationVector from previous cycle if there has been a previous cycle (is_initialized_ == true)
        if(override_input_effort_ && is_initialized_)
            Vel_IP_->CurrentAccelerationVector->VecData[i] = Vel_OP_->NewAccelerationVector->VecData[i];
        else
            Vel_IP_->CurrentAccelerationVector->VecData[i] = status_[joint_idx].effort;
    }

    //
    // Handle Reset commands
    //
    if(_reset.read(reset_command_) == RTT::NewData){
        for(uint i = 0; i < reset_command_.size(); i++){
            uint joint_idx;
            try{
                joint_idx = limits_.mapNameToIndex(reset_command_.names[i]);
            }
            catch(std::exception e){
                continue;
            }
            if(reset_command_[i].hasPosition())
                Vel_IP_->CurrentPositionVector->VecData[joint_idx] = reset_command_[i].position;
            if(reset_command_[i].hasSpeed()){
                Vel_IP_->CurrentVelocityVector->VecData[joint_idx] = reset_command_[i].speed;
                Vel_IP_->TargetVelocityVector->VecData[joint_idx] = reset_command_[i].speed;
            }
            if(reset_command_[i].hasEffort())
                Vel_IP_->CurrentAccelerationVector->VecData[joint_idx] = reset_command_[i].effort;
        }
    }

    uint res = RML_->RMLVelocity(*Vel_IP_, Vel_OP_, Vel_Flags_);
    is_initialized_ = true;

    //
    // Handle interpolation result
    //
    switch(res){
    case ReflexxesAPI::RML_WORKING:
        state(FOLLOWING);
        break;
    case ReflexxesAPI::RML_FINAL_STATE_REACHED:
        state(REACHED);
        break;
#ifdef USING_REFLEXXES_TYPE_IV
    case ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS:
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

    //
    // Write Output
    //
    for(uint i = 0; i < command_out_.size(); i++){
        command_out_[i].speed = Vel_OP_->NewVelocityVector->VecData[i];
        command_out_[i].effort = Vel_OP_->NewAccelerationVector->VecData[i];
    }
    _command.write(command_out_);

    //
    // Write debug Data
    //
    _actual_cycle_time.write((base::Time::now() - start).toSeconds());

    for(uint i = 0; i < nDOF_; i++){
        input_params_.CurrentPositionVector[i] = Vel_IP_->CurrentPositionVector->VecData[i];
        input_params_.CurrentVelocityVector[i] = Vel_IP_->CurrentVelocityVector->VecData[i];
        input_params_.CurrentAccelerationVector[i] = Vel_IP_->CurrentAccelerationVector->VecData[i];
        input_params_.TargetVelocityVector[i] = Vel_IP_->TargetVelocityVector->VecData[i];
#ifdef USING_REFLEXXES_TYPE_IV
        input_params_.MinPositionVector[i] = Vel_IP_->MinPositionVector->VecData[i];
        input_params_.MaxPositionVector[i] = Vel_IP_->MaxPositionVector->VecData[i];
#endif
        input_params_.MaxAccelerationVector[i] = Vel_IP_->MaxAccelerationVector->VecData[i];
        input_params_.MaxJerkVector[i] = Vel_IP_->MaxJerkVector->VecData[i];
        input_params_.SelectionVector[i] = Vel_IP_->SelectionVector->VecData[i];

        output_params_.ExecutionTimes[i] = Vel_OP_->ExecutionTimes->VecData[i];
        output_params_.NewPositionVector[i] = Vel_OP_->NewPositionVector->VecData[i];
        output_params_.NewVelocityVector[i] = Vel_OP_->NewVelocityVector->VecData[i];
        output_params_.NewAccelerationVector[i] = Vel_OP_->NewAccelerationVector->VecData[i];
        output_params_.PositionValuesAtTargetVelocity[i] = Vel_OP_->PositionValuesAtTargetVelocity->VecData[i];
    }
    input_params_.NumberOfDOFs = Vel_IP_->NumberOfDOFs;
    input_params_.MinimumSynchronizationTime = Vel_IP_->MinimumSynchronizationTime;
    output_params_.ANewCalculationWasPerformed = Vel_OP_->ANewCalculationWasPerformed;
    output_params_.NumberOfDOFs = Vel_OP_->NumberOfDOFs;
    output_params_.DOFWithTheGreatestExecutionTime = Vel_OP_->DOFWithTheGreatestExecutionTime;
    output_params_.SynchronizationTime = Vel_OP_->SynchronizationTime;
    output_params_.TrajectoryIsPhaseSynchronized = Vel_OP_->TrajectoryIsPhaseSynchronized;

    _rml_input_params.write(input_params_);
    _rml_output_params.write(output_params_);

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
