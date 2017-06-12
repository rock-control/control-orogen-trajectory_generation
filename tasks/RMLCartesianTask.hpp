/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLCARTESIANTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLCARTESIANTASK_TASK_HPP

#include "trajectory_generation/RMLCartesianTaskBase.hpp"
#include <base/samples/RigidBodyState.hpp>

namespace trajectory_generation{

class RMLCartesianTask : public RMLCartesianTaskBase
{
    friend class RMLCartesianTaskBase;

protected:
    base::samples::RigidBodyState cartesian_state; /** From input port: Current Cartesian state. Will only be used for initializing RML */
    base::samples::RigidBodyState current_sample;  /** From input port: Current Cartesian interpolator status (position/speed/acceleration)*/
    base::samples::RigidBodyState target;          /** From input port: Target Cartesian position or speed.  */
    base::samples::RigidBodyState command;         /** To output port: Commanded Cartesian position or speed.  */

    /** Update the motion constraints of a particular element*/
    virtual void updateMotionConstraints(const MotionConstraint& constraint,
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
                                            RMLFlags *rml_flag) = 0;

    /** Write the generated trajectory to port*/
    virtual void writeCommand(const RMLOutputParameters& new_output_parameters) = 0;

    /** Call echo() method for rml input and output parameters*/
    virtual void printParams(const RMLInputParameters& in, const RMLOutputParameters& out) = 0;

    /** Convert from RMLInputParameters to orogen type*/
    virtual const ReflexxesInputParameters& fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out) = 0;

    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out) = 0;

    /** Common conversion method for orientation values*/
    base::Vector3d toEuler(const base::Orientation& orientation);

    /** Common conversion method for orientation values*/
    base::Orientation fromEuler(const base::Vector3d& euler);

public:
    RMLCartesianTask(std::string const& name = "trajectory_generation::RMLCartesianTask") : RMLCartesianTaskBase(name){}
    RMLCartesianTask(std::string const& name, RTT::ExecutionEngine* engine) : RMLCartesianTaskBase(name, engine){}
    ~RMLCartesianTask(){}
    bool configureHook();
    bool startHook(){return RMLCartesianTaskBase::startHook();}
    void updateHook(){RMLCartesianTaskBase::updateHook();}
    void errorHook(){RMLCartesianTaskBase::errorHook();}
    void stopHook(){RMLCartesianTaskBase::stopHook();}
    void cleanupHook(){RMLCartesianTaskBase::cleanupHook();}
};
}

#endif

