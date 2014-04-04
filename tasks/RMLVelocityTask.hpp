/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLVELOCITYTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLVELOCITYTASK_TASK_HPP

#include "trajectory_generation/RMLVelocityTaskBase.hpp"

#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <base/commands/joints.h>
#include <trajectory_generation/trajectory_generationTypes.hpp>


namespace trajectory_generation {

class RMLVelocityTask : public RMLVelocityTaskBase
{
    friend class RMLVelocityTaskBase;
protected:

    ReflexxesAPI *RML_;
    RMLVelocityInputParameters  *Vel_IP_;
    RMLVelocityOutputParameters *Vel_OP_;
    RMLVelocityFlags Vel_Flags_;

    bool override_input_position_;   //Set output position as input for next cycle
    bool override_input_speed_;      //Set output speed as input for next cycle
    bool treat_effort_as_acceleration_; //Use the effort field from JointState type for acceleration.
    bool override_input_acceleration_;     //Set output acceleration as input for next cycle. When treat_effort_as_acceleration == false, input acceleration is always overriden. In this case, acceleration is always assumed to zero at first sample.

    base::JointLimits limits_;
    base::samples::Joints status_;
    double cycle_time_;
    base::commands::Joints command_out_;
    base::commands::Joints command_in_;
    bool is_initialized_;
    size_t nDOF_;
    double max_acceleration_scale_, max_jerk_scale_;
    base::samples::Joints output_sample_;
    base::VectorXd dist_to_upper_, dist_to_lower_;
    std::vector<std::string> dont_allow_positive_, dont_allow_negative_;

    RMLInputParams input_params_;
    RMLOutputParams output_params_;

    bool has_target_;
    base::Time stamp_;
    double timeout_;
    base::Time prev_time_;

public:
    RMLVelocityTask(std::string const& name = "trajectory_generation::RMLVelocityTask");
    RMLVelocityTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RMLVelocityTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

