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
    treat_effort_as_acceleration_ = _treat_effort_as_acceleration.value();
    override_input_acceleration_ = _override_input_acceleration.value();

    initial_motion_constraints_ = _initial_motion_constraints.get();
    cycle_time_ = _cycle_time.get();
    Vel_Flags_.SynchronizationBehavior = _sync_behavior.get();
    timeout_ = _timeout.get();

    nDOF_ =  initial_motion_constraints_.size();

    status_.resize(nDOF_);
    output_sample_.resize(nDOF_);
    output_sample_.names = initial_motion_constraints_.names;
    command_out_.resize(nDOF_);
    command_out_.names = initial_motion_constraints_.names;
    input_params_ = RMLInputParams(nDOF_);
    output_params_ = RMLOutputParams(nDOF_);
    max_velocity_.resize(nDOF_);

    RML_ = new ReflexxesAPI(nDOF_, cycle_time_);
    Vel_IP_ = new RMLVelocityInputParameters(nDOF_);
    Vel_OP_ = new RMLVelocityOutputParameters(nDOF_);

#ifdef USING_REFLEXXES_TYPE_IV
    Vel_Flags_.PositionalLimitsBehavior = _positional_limits_behavior.get();
#endif

    setActiveMotionConstraints(initial_motion_constraints_);

    return true;
}

bool RMLVelocityTask::startHook()
{
    if (! RMLVelocityTaskBase::startHook())
        return false;
    has_target_ = is_initialized_ = false;

    for(size_t i = 0; i < nDOF_; i++)
        Vel_IP_->TargetVelocityVector->VecData[i] = 0;

    return true;
}

void RMLVelocityTask::handleStatusInput(const base::samples::Joints &status)
{
    for(size_t i = 0; i < nDOF_; i++)
    {
        std::string joint_name = initial_motion_constraints_.names[i];
        size_t joint_idx = 0;
        try{
            joint_idx = status.mapNameToIndex(joint_name);
        }
        catch(std::exception e){
            continue;
        }

        // Input Position

        //Only use NewPositionVector from previous cycle if there has been a previous cycle (is_initialized_ == true)
        if(override_input_position_ && is_initialized_)
            Vel_IP_->CurrentPositionVector->VecData[i] = Vel_OP_->NewPositionVector->VecData[i];
        else
            Vel_IP_->CurrentPositionVector->VecData[i] = status[joint_idx].position;

#ifdef USING_REFLEXXES_TYPE_IV
        //Avoid invalid input here (RML with active Positonal Limits prevention has problems with positional input that is out of limits,
        //which may happen due to noisy position readings)
        if(Vel_Flags_.PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT)
        {
            double new_position = std::max(std::min(initial_motion_constraints_[i].max.position, Vel_IP_->CurrentPositionVector->VecData[i]), initial_motion_constraints_[i].min.position);
            Vel_IP_->CurrentPositionVector->VecData[i] = new_position;
        }
#endif

        // Input Velocity

        //Only use NewVelocityVector from previous cycle if there has been a previous cycle (is_initialized_ == true)
        if(override_input_speed_ && is_initialized_)
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

        //We can only use 'real' acceleration values from joint status if we treat the effort field as acceleration and don't want to override effort. TODO: Way to complicated
        /*if(treat_effort_as_acceleration_ && !override_input_acceleration_)
        {
            //Effort field from joint status is treaded as accerlation and 'real' state should be used. Set it accordingly.
            Vel_IP_->CurrentAccelerationVector->VecData[i] = status[joint_idx].effort;
        }
        //Otherwise we must check whether there was a reference generated before. If so use it, otherwise assume 0
        else
        {*/
            if(!is_initialized_)
            {
                //No reference acceleration was genereated before. Assume zero.
                Vel_IP_->CurrentAccelerationVector->VecData[i] = 0;
            }
            else
            {
                //Override with reference from previous cycle
                Vel_IP_->CurrentAccelerationVector->VecData[i] = Vel_OP_->NewAccelerationVector->VecData[i];
            }
      //  }
    }
}

void RMLVelocityTask::handleCommandInput(const base::commands::Joints &command)
{
    for(size_t i = 0; i < nDOF_; i++)
        Vel_IP_->SelectionVector->VecData[i] = false;

    // Use name mapping to allow partial inputs.
    for(size_t i = 0; i < command_in_.size(); i++)
    {
        size_t joint_idx = 0;
        try{
            joint_idx = initial_motion_constraints_.mapNameToIndex(command.names[i]);
        }
        catch(std::exception e){
            continue;
        }

        if(command_in_[i].getMode() != base::JointState::SPEED){
            LOG_ERROR("RMLVelocity supports only speed control mode, but input has control mode %i", command[i].getMode());
            throw std::invalid_argument("Invalid control mode");
        }

        //Actively avoid speed limits here:
        Vel_IP_->TargetVelocityVector->VecData[joint_idx] = command[i].speed;
                //std::max(std::min(max_velocity_[joint_idx], command[i].speed), -max_velocity_[joint_idx]);

        /*LOG_DEBUG("Joint %i(%s): Speed %f, Max Speed: %f, Min Speed: %f", joint_idx, command.names[i].c_str(),
                  Vel_IP_->TargetVelocityVector->VecData[joint_idx], motion_constraints_[joint_idx].max.speed,
                  motion_constraints_[joint_idx].min.speed);*/

        Vel_IP_->SelectionVector->VecData[joint_idx] = true;
    }
}

void RMLVelocityTask::setActiveMotionConstraints(const trajectory_generation::JointsMotionConstraints& constraints)
{
    LOG_DEBUG("Setting active motion constraints to ... ");

    for(size_t i = 0; i < constraints.size(); i++)
    {
        if( constraints[i].max.effort <= 0 ||  constraints[i].max.speed <= 0){
            LOG_ERROR("Max Effort and max speed of limits property must be grater than zero");
            throw std::invalid_argument("Invalid motion constraints");
        }

        size_t idx;
        try{
            idx = initial_motion_constraints_.mapNameToIndex(constraints.names[i]);
        }
        catch(std::exception e)
        {
            LOG_ERROR("%s: SetMotionConstraints: Joint %s has not been configured in motion constraints",
                      this->getName().c_str(), constraints.names[i].c_str());
            throw std::invalid_argument("Invalid constraint vector");
        }

        if(!base::isUnset(constraints[i].max.speed) && !base::isNaN(constraints[i].max.speed))
            max_velocity_[idx] = constraints[i].max.speed;
        else
            max_velocity_[idx] = initial_motion_constraints_[idx].max.speed;

        if(!base::isUnset(constraints[i].max.effort) && !base::isNaN(constraints[i].max.effort))
            Vel_IP_->MaxAccelerationVector->VecData[idx] = constraints[i].max.effort;
        else{
            if(fabs(Vel_IP_->CurrentAccelerationVector->VecData[idx]) < initial_motion_constraints_[idx].max.effort)
                Vel_IP_->MaxAccelerationVector->VecData[idx] = initial_motion_constraints_[idx].max.effort;
        }

        //TODO: This has to be made mode transparent: It must not be allowed to set a maximum acceleration/jerk that
        //is smaller than the current acceleration/jerk

        if(!base::isUnset(constraints[i].max_jerk) && !base::isNaN(constraints[i].max_jerk))
            Vel_IP_->MaxJerkVector->VecData[idx] = constraints[i].max_jerk;
        else{
            if(fabs(Vel_IP_->CurrentAccelerationVector->VecData[idx]) < initial_motion_constraints_[idx].max.effort)
                Vel_IP_->MaxJerkVector->VecData[idx] = initial_motion_constraints_[idx].max_jerk;
        }


#ifdef USING_REFLEXXES_TYPE_IV
        if(!base::isUnset(constraints[i].max.position) && !base::isNaN(constraints[i].max.position))
            Vel_IP_->MaxPositionVector->VecData[idx] = constraints[i].max.position;
        else
            Vel_IP_->MaxPositionVector->VecData[idx] = initial_motion_constraints_[idx].max.position;

        if(!base::isUnset(constraints[i].min.position) && !base::isNaN(constraints[i].min.position))
            Vel_IP_->MinPositionVector->VecData[idx] = constraints[i].min.position;
        else
            Vel_IP_->MinPositionVector->VecData[idx] = initial_motion_constraints_[idx].min.position;

        /*
        LOG_DEBUG("Joint idx %i(%s): Max Pos: %f, Min pos: %f, Max Vel: %f, Max Acc: %f, Max Jerk: %f",
                  idx, constraints.names[i].c_str(),  Vel_IP_->MaxPositionVector->VecData[idx],
                   Vel_IP_->MinPositionVector->VecData[idx], max_velocity_[idx],
                   Vel_IP_->MaxAccelerationVector->VecData[idx],  Vel_IP_->MaxJerkVector->VecData[idx]);*/

#endif
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

        if(treat_effort_as_acceleration_)
            command_out_[i].effort = output->NewAccelerationVector->VecData[i];
        else
            command_out_[i].effort = base::unknown<float>();

    }
    command_out_.time = base::Time::now();
    _command.write(command_out_);
}

void RMLVelocityTask::writeOutputSample(const base::samples::Joints& status, const RMLVelocityOutputParameters* output)
{
    if(is_initialized_)
    {
        for(size_t i = 0; i < command_out_.size(); i++)
        {
            output_sample_[i].position = output->NewPositionVector->VecData[i];
            output_sample_[i].speed = output->NewVelocityVector->VecData[i];
            output_sample_[i].effort = output->NewAccelerationVector->VecData[i];
        }
    }
    else
    {
        for(size_t i = 0; i < nDOF_; i++){
            std::string joint_name = initial_motion_constraints_.names[i];
            size_t joint_idx = 0;
            try{
                joint_idx = status.mapNameToIndex(joint_name);
            }
            catch(std::exception e){
                continue;
            }

            output_sample_[i].position = status[joint_idx].position;
            output_sample_[i].speed = status[joint_idx].speed;
            output_sample_[i].effort= status[joint_idx].effort;
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

    while(_motion_constraints.read(constraints_from_port_) == RTT::NewData)
        setActiveMotionConstraints(constraints_from_port_);

    while(_velocity_target.read(command_in_) == RTT::NewData){
        handleCommandInput(command_in_);
        if(!has_target_) //init timestamp for velocity watchdog, so that it is not activated right away
            stamp_ = base::Time::now();
        has_target_ = true;
    }

    if(has_target_)
    {
        double difftime = (base::Time::now() - stamp_).toSeconds();
        if(difftime > timeout_)
        {
            stamp_ = base::Time::now();
            for(size_t i = 0; i < nDOF_; i++)
                Vel_IP_->TargetVelocityVector->VecData[i] = 0;
        }

        handleRMLInterpolationResult(RML_->RMLVelocity(*Vel_IP_, Vel_OP_, Vel_Flags_));
        is_initialized_ = true;

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
    initial_motion_constraints_.clear();
}
