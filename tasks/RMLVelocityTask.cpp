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

    limits_ = _limits.get();
    cycle_time_ = _cycle_time.get();
    Vel_Flags_.SynchronizationBehavior = _sync_behavior.get();
    std::vector<double> max_jerk = _max_jerk.get();
    max_acceleration_scale_ = _max_acceleration_scale.get();
    max_jerk_scale_ = _max_jerk_scale.get();
    timeout_ = _timeout.get();

    if(max_jerk.size() != limits_.size())
    {
        LOG_ERROR("No of joints of interpolator is %i but size of max_jerk is %i", limits_.size(), max_jerk.size());
        return false;
    }

    nDOF_ = limits_.size();

    dist_to_upper_.resize(nDOF_);
    dist_to_lower_.resize(nDOF_);

    status_.resize(nDOF_);
    command_out_.resize(nDOF_);
    command_out_.names = limits_.names;
    input_params_ = RMLInputParams(nDOF_);
    output_params_ = RMLOutputParams(nDOF_);

    RML_ = new ReflexxesAPI(nDOF_, cycle_time_);
    Vel_IP_ = new RMLVelocityInputParameters(nDOF_);
    Vel_OP_ = new RMLVelocityOutputParameters(nDOF_);

    for(size_t i = 0; i < nDOF_; i++)
    {
        if(limits_[i].max.effort <= 0 || limits_[i].max.speed <= 0){
            LOG_ERROR("Max Effort and max speed of limits property must be grater than zero");
            return false;
        }
        command_out_[i].speed = command_out_[i].effort = 0;
        Vel_IP_->MaxAccelerationVector->VecData[i] = limits_[i].max.effort * max_acceleration_scale_;
        Vel_IP_->MaxJerkVector->VecData[i] = max_jerk[i] * max_jerk_scale_;
        Vel_IP_->TargetVelocityVector->VecData[i] = 0;
    }

#ifdef USING_REFLEXXES_TYPE_IV
    Vel_Flags_.PositionalLimitsBehavior = _positional_limits_behavior.get();
    for(size_t i = 0; i < nDOF_; i++)
    {
        Vel_IP_->MaxPositionVector->VecData[i] = limits_[i].max.position;
        Vel_IP_->MinPositionVector->VecData[i] = limits_[i].min.position;
    }
#endif

    return true;
}

bool RMLVelocityTask::startHook()
{
    if (! RMLVelocityTaskBase::startHook())
        return false;
    stamp_ = base::Time::now();
    has_target_ = false;
    is_initialized_ = false;
    return true;
}

void RMLVelocityTask::updateHook()
{
    RMLVelocityTaskBase::updateHook();

    if(_joint_state.read(status_) == RTT::NoData){
        LOG_DEBUG("No data on joint state port");
        return;
    }

    //
    // Single Velocity Command input
    //
    while(_velocity_target.read(command_in_) == RTT::NewData){

        for(size_t i = 0; i < nDOF_; i++)
            Vel_IP_->SelectionVector->VecData[i] = false;

        stamp_ = base::Time::now();
        // Use name mapping to allow partial inputs.
        for(size_t i = 0; i < command_in_.size(); i++){

            size_t joint_idx = 0;
            try{
                joint_idx = limits_.mapNameToIndex(command_in_.names[i]);
                LOG_DEBUG("Mapped joint with name '%s' to index %d", command_in_.names[i].c_str(), joint_idx);
            }
            catch(std::exception e){
                LOG_DEBUG("Joint '%s' that is mentioned in command was not configured to be used. Will ignore command for this joint.", command_in_.names[i].c_str());
                continue;
            }

            if(command_in_[i].getMode() != base::JointState::SPEED){
                LOG_ERROR("RMLVelocity supports only speed control mode, but input has control mode %i", command_in_[i].getMode());
                throw std::invalid_argument("Invalid control mode");
            }

            //Actively avoid speed limits here:
            Vel_IP_->TargetVelocityVector->VecData[joint_idx] =
                    std::max(std::min(limits_[joint_idx].max.speed, command_in_[i].speed), -limits_[joint_idx].max.speed);
            LOG_DEBUG("Modified target velocity if joint %s from %f to %f, due to velocity limits.", command_in_.names[i].c_str(), command_in_[i].speed, Vel_IP_->TargetVelocityVector->VecData[joint_idx]);

            Vel_IP_->SelectionVector->VecData[joint_idx] = true;
        }

        //Init timestamp here, if this was the first target. Like this the velocity watchdog will not bark immediately!
        if(!has_target_)
                stamp_ = base::Time::now();

        has_target_ = true;
    }

    if(has_target_){
        double difftime = (base::Time::now() - stamp_).toSeconds();
        if(difftime > timeout_){
            LOG_DEBUG("Watchdog: Last reference value arrived %f seconds ago, Timeout is %f. Setting reference to zero", difftime, timeout_);
            stamp_ = base::Time::now();
            for(size_t i = 0; i < nDOF_; i++)
                Vel_IP_->TargetVelocityVector->VecData[i] = 0;
        }

        //
        // Compute next sample
        //
        for(size_t i = 0; i < nDOF_; i++){
            std::string joint_name = limits_.names[i];
            size_t joint_idx = 0;
            try{
                joint_idx = status_.mapNameToIndex(joint_name);
            }
            catch(std::exception e){
                continue;
            }

            //Only use NewPositionVector from previous cycle if there has been a previous cycle (is_initialized_ == true)
            if(override_input_position_ && is_initialized_){
                Vel_IP_->CurrentPositionVector->VecData[i] = Vel_OP_->NewPositionVector->VecData[i];
                LOG_DEBUG("Override current position of joint %s to %f", joint_name.c_str(), Vel_OP_->NewPositionVector->VecData[i])
            }
            else{
                Vel_IP_->CurrentPositionVector->VecData[i] = status_[joint_idx].position;
                LOG_DEBUG("Set current position of joint %s to %f (else)", joint_name.c_str(), Vel_IP_->CurrentPositionVector->VecData[i])
            }

#ifdef USING_REFLEXXES_TYPE_IV
            //Avoid invalid input here (RML with active Positonal Limits prevention has problems with positional input that is out of limits,
            //which may happen due to noisy position readings)
            if(Vel_Flags_.PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
                double new_position = std::max(std::min(limits_[i].max.position, Vel_IP_->CurrentPositionVector->VecData[i]), limits_[i].min.position);
                LOG_DEBUG("Override current position for jpint %s from %f to %f due to position limits violation.", joint_name.c_str(),
                          Vel_IP_->CurrentPositionVector->VecData[i], new_position);
                Vel_IP_->CurrentPositionVector->VecData[i] = new_position;
            }
#endif

            //Only use NewVelocityVector from previous cycle if there has been a previous cycle (is_initialized_ == true)
            if(override_input_speed_ && is_initialized_){
                Vel_IP_->CurrentVelocityVector->VecData[i] = Vel_OP_->NewVelocityVector->VecData[i];
                LOG_DEBUG("Overide current velocity for joint %s to %f", joint_name.c_str(), Vel_IP_->CurrentVelocityVector->VecData[i]);
            }
            else
                Vel_IP_->CurrentVelocityVector->VecData[i] = status_[joint_idx].speed;

#ifndef USING_REFLEXXES_TYPE_IV
            // This is to fix a bug in the Reflexxes type II library. If current and target velocity are equal
            // the output will be nan.
            if(Vel_IP_->CurrentVelocityVector->VecData[i] == Vel_IP_->TargetVelocityVector->VecData[i])
                Vel_IP_->CurrentVelocityVector->VecData[i] -= 1e-10;
#endif

            //We can only use 'real' acceleration values from joint status if we treat the effort field as acceleration and don't want to override effort
            if(treat_effort_as_acceleration_ && !override_input_acceleration_){
                //Effort field from joint status is treaded as accerlation and 'real' state should be used. Set it accordingly.
                Vel_IP_->CurrentAccelerationVector->VecData[i] = status_[joint_idx].effort;
            }
            //Otherwise we must check whether there was a reference generated before. If so use it, otherwise assume 0
            else{
                if(!is_initialized_){
                    //No reference acceleration was genereated before. Assume zero.
                    Vel_IP_->CurrentAccelerationVector->VecData[i] = 0;
                    LOG_DEBUG("Overide current acceleration for joint %s to %f (first cycle)", joint_name.c_str(), Vel_IP_->CurrentAccelerationVector->VecData[i]);
                }
                else{
                    //Override with reference from previous cycle
                    Vel_IP_->CurrentAccelerationVector->VecData[i] = Vel_OP_->NewAccelerationVector->VecData[i];
                    LOG_DEBUG("Overide current acceleration for joint %s to %f", joint_name.c_str(), Vel_IP_->CurrentAccelerationVector->VecData[i]);
                }
            }

        }

        //
        // Handle Reset commands
        //
        if(_reset.read(reset_command_) == RTT::NewData){
            for(size_t i = 0; i < reset_command_.size(); i++){
                size_t joint_idx;
                std::string joint_name = reset_command_.names[i];
                try{
                    joint_idx = limits_.mapNameToIndex(joint_name);
                }
                catch(std::exception e){
                    continue;
                }
                if(reset_command_[i].hasPosition()){
                    Vel_IP_->CurrentPositionVector->VecData[joint_idx] = reset_command_[i].position;
                    LOG_DEBUG("Reset current position of joint %s to %f", joint_name.c_str(), Vel_IP_->CurrentPositionVector->VecData[joint_idx]);
                }
                if(reset_command_[i].hasSpeed()){
                    Vel_IP_->CurrentVelocityVector->VecData[joint_idx] = reset_command_[i].speed;
                    LOG_DEBUG("Reset current velocity of joint %s to %f", joint_name.c_str(), Vel_IP_->CurrentVelocityVector->VecData[joint_idx]);
                    Vel_IP_->TargetVelocityVector->VecData[joint_idx] = reset_command_[i].speed;
                    LOG_DEBUG("Reset target velocity of joint %s to %f", joint_name.c_str(), Vel_IP_->TargetVelocityVector->VecData[joint_idx]);
                }
                if(treat_effort_as_acceleration_ && reset_command_[i].hasEffort()){
                    Vel_IP_->CurrentAccelerationVector->VecData[joint_idx] = reset_command_[i].effort;
                    LOG_DEBUG("Reset current acceleration of joint %s to %f", joint_name.c_str(), Vel_IP_->CurrentAccelerationVector->VecData[joint_idx]);
                }
                else{
                    //We dont have acceleration information. Assume zero.
                    // FIXME: Is this really better than assuming previous reference? I think so, but not sure.
                    Vel_IP_->CurrentAccelerationVector->VecData[joint_idx] = 0;
                }

            }
        }

        int res = RML_->RMLVelocity(*Vel_IP_, Vel_OP_, Vel_Flags_);
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
        for(size_t i = 0; i < command_out_.size(); i++){
            std::string joint_name = command_out_.names[i].c_str();
            LOG_DEBUG("New velocity for joint %s is %f", joint_name.c_str(), Vel_OP_->NewVelocityVector->VecData[i]);
            LOG_DEBUG("New acceleration for joint %s is %f", joint_name.c_str(), Vel_OP_->NewAccelerationVector->VecData[i])
            command_out_[i].speed = Vel_OP_->NewVelocityVector->VecData[i];
            if(treat_effort_as_acceleration_){
                command_out_[i].effort = Vel_OP_->NewAccelerationVector->VecData[i];
            }
            else{
                command_out_[i].effort = base::unknown<float>();
            }
        }
        command_out_.time = base::Time::now();
        _command.write(command_out_);
    }


    //
    // Write debug data
    //

    base::Time time = base::Time::now();
    base::Time diff = time-prev_time_;
    _actual_cycle_time.write(diff.toSeconds());
    prev_time_ = time;

    for(size_t i = 0; i < nDOF_; i++){
        input_params_.CurrentPositionVector[i] = Vel_IP_->CurrentPositionVector->VecData[i];
        input_params_.CurrentVelocityVector[i] = Vel_IP_->CurrentVelocityVector->VecData[i];
        input_params_.CurrentAccelerationVector[i] = Vel_IP_->CurrentAccelerationVector->VecData[i];
        input_params_.TargetVelocityVector[i] = Vel_IP_->TargetVelocityVector->VecData[i];
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

#ifdef USING_REFLEXXES_TYPE_IV
    for(size_t i = 0; i < nDOF_; i++)
    {
        input_params_.MinPositionVector[i] = Vel_IP_->MinPositionVector->VecData[i];
        input_params_.MaxPositionVector[i] = Vel_IP_->MaxPositionVector->VecData[i];
        dist_to_upper_(i) = limits_[i].max.position - Vel_IP_->CurrentPositionVector->VecData[i];
        dist_to_lower_(i) = Vel_IP_->CurrentPositionVector->VecData[i] - limits_[i].min.position;
    }
    _dist_lower.write(dist_to_lower_);
    _dist_upper.write(dist_to_upper_);
#endif

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
