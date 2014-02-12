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
    bool override_input_effort_;     //Set output effort as input for next cycle

    base::JointLimits limits_;
    base::samples::Joints status_;
    double cycle_time_;
    base::commands::Joints command_out_;
    base::commands::Joints command_in_;
    bool is_initialized_;
    uint nDOF_;
    double max_effort_scale_, max_jerk_scale_;
    base::samples::Joints reset_command_;

    RMLVelocityInputParams input_params_;
    RMLVelocityOutputParams output_params_;

    base::Time stamp_;
    double timeout_;

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

