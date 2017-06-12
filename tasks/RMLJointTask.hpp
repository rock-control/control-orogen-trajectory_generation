/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLJOINTTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLJOINTTASK_TASK_HPP

#include "trajectory_generation/RMLJointTaskBase.hpp"
#include <base/commands/Joints.hpp>

namespace trajectory_generation{

class RMLJointTask : public RMLJointTaskBase
{
    friend class RMLJointTaskBase;

protected:
    base::samples::Joints joint_state;    /** From input port: Current joint state. Will only be used for initializing RML */
    base::samples::Joints current_sample; /** From input port: Current joint interpolator status (position/speed/acceleration)*/
    ConstrainedJointsCmd target;          /** From input port: Target joint position or speed.  */
    base::commands::Joints command;       /** To output port: Commanded joint position or speed.  */

    /** Update the motion constraints of a particular element*/
    void updateMotionConstraints(const MotionConstraint& constraint,
                                 const size_t idx,
                                 RMLInputParameters* new_input_parameters) = 0;

    /** Read the current state from port and return position and flow status*/
    virtual RTT::FlowStatus getCurrentPosition(std::vector<double> &current_position);

    /** Read target vector and return the flow status*/
    virtual RTT::FlowStatus getTarget(TargetVector& target_vector);

    /** Read a new target from port. If available, update the RML input parameters. Also return the flow state of the port. */
    virtual void updateTarget(const TargetVector& target_vector,
                              RMLInputParameters* new_input_parameters) = 0;

    /** Perform one step of online trajectory generation (call the RML algorithm with the given parameters). Return the RML result value*/
    virtual ReflexxesResultValue performOTG(RMLInputParameters* new_input_parameters,
                                            RMLOutputParameters* new_output_parameters,
                                            RMLFlags *rml_flags) = 0;

    /** Write the generated trajectory to port*/
    virtual void writeCommand(const RMLOutputParameters& new_output_parameters) = 0;

    /** Call echo() method for rml input and output parameters*/
    virtual void printParams(const RMLInputParameters& in, const RMLOutputParameters& out) = 0;

    /** Convert from RMLInputParameters to orogen type*/
    virtual const ReflexxesInputParameters& fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out) = 0;

    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out) = 0;

public:
    RMLJointTask(std::string const& name = "trajectory_generation::RMLJointTask") : RMLJointTaskBase(name){}
    RMLJointTask(std::string const& name, RTT::ExecutionEngine* engine) : RMLJointTaskBase(name, engine){}
    ~RMLJointTask(){}
    bool configureHook();
    bool startHook(){return RMLJointTaskBase::startHook();}
    void updateHook(){RMLJointTaskBase::updateHook();}
    void errorHook(){RMLJointTaskBase::errorHook();}
    void stopHook(){RMLJointTaskBase::stopHook();}
    void cleanupHook();
};
}

#endif

