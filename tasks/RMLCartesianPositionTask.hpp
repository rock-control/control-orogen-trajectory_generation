/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLCARTESIANPOSITIONTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLCARTESIANPOSITIONTASK_TASK_HPP

#include "trajectory_generation/RMLCartesianPositionTaskBase.hpp"

namespace trajectory_generation{

class RMLCartesianPositionTask : public RMLCartesianPositionTaskBase
{
    friend class RMLCartesianPositionTaskBase;

    base::samples::RigidBodyState cartesian_state; /** From input port: Current Cartesian state. Will only be used for initializing RML */
    base::samples::RigidBodyState current_sample;  /** From input port: Current Cartesian interpolator status (position/speed/acceleration)*/
    base::samples::RigidBodyState target;          /** From input port: Target Cartesian position or speed.  */
    base::samples::RigidBodyState command;         /** To output port: Commanded Cartesian position or speed.  */

protected:
    /** Update the motion constraints of a particular element*/
    virtual void updateMotionConstraints(const MotionConstraint& constraint,
                                         const size_t idx,
                                         RMLInputParameters* new_input_parameters);

    /** Read the current state from port and return position and flow status*/
    virtual bool updateCurrentState(RMLInputParameters* new_input_parameters);

    /** Update the RML input parameters with the new target */
    virtual bool updateTarget(RMLInputParameters* new_input_parameters);

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
    RMLCartesianPositionTask(std::string const& name = "trajectory_generation::RMLCartesianPositionTask") : RMLCartesianPositionTaskBase(name){}
    RMLCartesianPositionTask(std::string const& name, RTT::ExecutionEngine* engine) : RMLCartesianPositionTaskBase(name, engine){}
    ~RMLCartesianPositionTask(){}
    bool configureHook();
    bool startHook(){return RMLCartesianPositionTaskBase::startHook();}
    void updateHook(){RMLCartesianPositionTaskBase::updateHook();}
    void errorHook(){RMLCartesianPositionTaskBase::errorHook();}
    void stopHook(){RMLCartesianPositionTaskBase::stopHook();}
    void cleanupHook(){RMLCartesianPositionTaskBase::cleanupHook();}
};
}

#endif

