/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLCARTESIANPOSITIONTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLCARTESIANPOSITIONTASK_TASK_HPP

#include "trajectory_generation/RMLCartesianPositionTaskBase.hpp"

namespace trajectory_generation{

/** Position based implementation of a Cartesian RML task. The given target has to have valid position entries. Velocity entries are optional.*/
class RMLCartesianPositionTask : public RMLCartesianPositionTaskBase
{
    friend class RMLCartesianPositionTaskBase;
protected:    
    /** Update the motion constraints of a particular element*/
    virtual void updateMotionConstraints(const MotionConstraint& constraint,
                                         const size_t idx,
                                         RMLInputParameters* new_input_parameters);

    /** Perform one step of online trajectory generation (call the RML algorithm with the given parameters). Return the RML result value*/
    virtual ReflexxesResultValue performOTG(RMLInputParameters* new_input_parameters,
                                            RMLOutputParameters* new_output_parameters,
                                            RMLFlags *rml_flags);

    /** Write the generated trajectory to port*/
    virtual void writeCommand(const RMLOutputParameters& new_output_parameters);

    /** Call echo() method for rml input and output parameters*/
    virtual void printParams(const RMLInputParameters& in, const RMLOutputParameters& out);

    /** Update the RML input parameters with the new target */
    virtual void updateTarget(const TargetVector& target_vector,
                              RMLInputParameters* new_input_parameters);

    /** Convert from RMLInputParameters to orogen type*/
    virtual const ReflexxesInputParameters& fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out);

    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out);


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

