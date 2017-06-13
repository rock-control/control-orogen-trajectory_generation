/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLCARTESIANVELOCITYTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLCARTESIANVELOCITYTASK_TASK_HPP

#include "trajectory_generation/RMLCartesianVelocityTaskBase.hpp"

namespace trajectory_generation{

class RMLCartesianVelocityTask : public RMLCartesianVelocityTaskBase
{
    friend class RMLCartesianVelocityTaskBase;

    base::samples::RigidBodyState cartesian_state; /** From input port: Current Cartesian state. Will only be used for initializing RML */
    base::samples::RigidBodyState current_sample;  /** From input port: Current Cartesian interpolator status (position/speed/acceleration)*/
    base::samples::RigidBodyState target;          /** From input port: Target Cartesian position or speed.  */
    base::samples::RigidBodyState command;         /** To output port: Commanded Cartesian position or speed.  */

    double no_reference_timeout;
    base::Time time_of_last_reference;
    bool convert_to_position;

protected:
    /** Update the motion constraints of a particular element*/
    virtual void updateMotionConstraints(const MotionConstraint& constraint,
                                         const size_t idx,
                                         RMLInputParameters* new_input_parameters);

    /** Read the current state from port and return position and flow status*/
    virtual RTT::FlowStatus updateCurrentState(RMLInputParameters* new_input_parameters);

    /** Update the RML input parameters with the new target */
    virtual RTT::FlowStatus updateTarget(RMLInputParameters* new_input_parameters);

    /** Perform one step of online trajectory generation (call the RML algorithm with the given parameters). Return the RML result value*/
    virtual ReflexxesResultValue performOTG(RMLInputParameters* new_input_parameters,
                                            RMLOutputParameters* new_output_parameters,
                                            RMLFlags *rml_flags);

    /** Write the generated trajectory to port*/
    virtual void writeCommand(const RMLOutputParameters& new_output_parameters);

    /** Call echo() method for rml input and output parameters*/
    virtual void printParams(const RMLInputParameters& in, const RMLOutputParameters& out);

    /** Convert from RMLInputParameters to orogen type*/
    virtual const ReflexxesInputParameters& convertRMLInputParams(const RMLInputParameters &in, ReflexxesInputParameters& out);

    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& convertRMLOutputParams(const RMLOutputParameters &in, ReflexxesOutputParameters& out);

public:
    RMLCartesianVelocityTask(std::string const& name = "trajectory_generation::RMLCartesianVelocityTask") : RMLCartesianVelocityTaskBase(name){}
    RMLCartesianVelocityTask(std::string const& name, RTT::ExecutionEngine* engine) : RMLCartesianVelocityTaskBase(name, engine){}
    ~RMLCartesianVelocityTask(){}
    bool configureHook();
    bool startHook(){return RMLCartesianVelocityTaskBase::startHook();}
    void updateHook(){RMLCartesianVelocityTaskBase::updateHook();}
    void errorHook(){RMLCartesianVelocityTaskBase::errorHook();}
    void stopHook(){RMLCartesianVelocityTaskBase::stopHook();}
    void cleanupHook(){RMLCartesianVelocityTaskBase::cleanupHook();}
};
}

#endif

