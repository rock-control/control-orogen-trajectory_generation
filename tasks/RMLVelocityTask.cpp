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
    status_.resize(limits_.size());
    cycle_time_ = _cycle_time.get();
    command_out_.resize(limits_.size());
    command_out_.names = limits_.names;
    input_params_ = RMLVelocityInputParams(limits_.size());
    output_params_ = RMLVelocityOutputParams(limits_.size());

    RML_ = new ReflexxesAPI(limits_.size(), cycle_time_);
    Vel_IP_ = new RMLVelocityInputParameters(limits_.size());
    Vel_OP_ = new RMLVelocityOutputParameters(limits_.size());
    Vel_Flags_.SynchronizationBehavior = _sync_behavior.get();

    for(uint i = 0; i < limits_.size(); i++){
        //TODO
        //Vel_IP_->MaxPositionVector->VecData[i] = limits_[i].max.position;
        //Vel_IP_->MinPositionVector->VecData[i] = limits_[i].min.position;
        if(limits_[i].max.effort <= 0){
            LOG_ERROR("Max Effort must be grater than zero");
            return false;
        }
        command_out_[i].speed = command_out_[i].effort = 0;
        Vel_IP_->MaxAccelerationVector->VecData[i] = limits_[i].max.effort;
        Vel_IP_->MaxJerkVector->VecData[i] = 1.0;
        Vel_IP_->SelectionVector->VecData[i] = true;
        LOG_DEBUG("Joint: %s, Max Effort: %f, Max Jerk: %f", limits_.names[i].c_str(), Vel_IP_->MaxAccelerationVector->VecData[i], Vel_IP_->MaxJerkVector->VecData[i]);
    }

    is_initialized_ = false;
    has_target_ = false;

    return true;
}

bool RMLVelocityTask::startHook()
{
    if (! RMLVelocityTaskBase::startHook())
        return false;
    return true;
}

void RMLVelocityTask::updateHook()
{
    base::Time start = base::Time::now();

    RMLVelocityTaskBase::updateHook();

    if(_joint_state.read(status_) == RTT::NoData){
        LOG_DEBUG("No data on joint status port");
        return;
    }

    if(status_.size() != limits_.size()){
        LOG_ERROR("Status should have size %i bus has size %i", limits_.size(), status_.size());
        throw std::invalid_argument("Invalid status size");
    }

    //
    // Single Velocity Command input
    //
    while(_velocity_target.read(command_in_) == RTT::NewData){

        for(uint i = 0; i < command_in_.size(); i++)
        {
            uint joint_idx = 0;
            try{joint_idx = limits_.mapNameToIndex(command_in_.names[i]);}
            catch(std::exception e){
                continue;
            }
            if(command_in_[i].getMode() != base::JointState::SPEED)
            {
                LOG_ERROR("Supports only speed control mode, but input has control mode %i", command_in_[i].getMode());
                throw std::invalid_argument("Invalid control mode");
            }
            Vel_IP_->TargetVelocityVector->VecData[joint_idx] = command_in_[i].speed;
        }
        has_target_ = true;
    }

    //
    // Compute next samples
    //
    for(uint i = 0; i < limits_.size(); i++){
        if(override_input_position_ && is_initialized_)
            Vel_IP_->CurrentPositionVector->VecData[i] = Vel_OP_->NewPositionVector->VecData[i];
        else
            Vel_IP_->CurrentPositionVector->VecData[i] = status_[i].position;

        if(override_input_speed_ && is_initialized_)
            Vel_IP_->CurrentVelocityVector->VecData[i] = Vel_OP_->NewVelocityVector->VecData[i];
        else
            Vel_IP_->CurrentVelocityVector->VecData[i] = status_[i].speed;

        if(override_input_effort_ && is_initialized_)
            Vel_IP_->CurrentAccelerationVector->VecData[i] = Vel_OP_->NewAccelerationVector->VecData[i];
        else
            Vel_IP_->CurrentAccelerationVector->VecData[i] = status_[i].effort;
    }

    if(has_target_){
        uint res = RML_->RMLVelocity(*Vel_IP_, Vel_OP_, Vel_Flags_);

        for(uint i = 0; i < limits_.size(); i++){
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

        _rml_input_params.write(input_params_);
        _rml_output_params.write(output_params_);

        is_initialized_ = true;

        switch(res){
        case ReflexxesAPI::RML_WORKING:
            state(FOLLOWING);
            break;
        case ReflexxesAPI::RML_FINAL_STATE_REACHED:
            state(REACHED);
            break;
        case ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES:
            LOG_ERROR("RML_ERROR_INVALID_INPUT_VALUES");
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
        default:
            LOG_ERROR("Reflexxes returned error %i", res);
            throw std::runtime_error("Reflexxes runtime error");
            break;
        }

        for(uint i = 0; i < command_out_.size(); i++){
            command_out_[i].speed = Vel_OP_->NewVelocityVector->VecData[i];
            command_out_[i].effort = Vel_OP_->NewAccelerationVector->VecData[i];
        }
    }

    _cmd.write(command_out_);
    base::Time diff = base::Time::now() - start;
    _actual_cycle_time.write(diff.toSeconds());
    if(diff.toSeconds() > cycle_time_){
        LOG_WARN("Computation Time for one cycle exceeded desired cycle time");
        LOG_WARN("Cycle Time: %f", cycle_time_);
        LOG_WARN("Computation Time: %f", diff.toSeconds());
        LOG_WARN("...........................");
    }
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
