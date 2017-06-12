/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLCARTESIANVELOCITYTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLCARTESIANVELOCITYTASK_TASK_HPP

#include "trajectory_generation/RMLCartesianVelocityTaskBase.hpp"

namespace trajectory_generation{

/** Velocity based implementation of a Cartesian RML task. The given target has to have valid velocity entries. */
class RMLCartesianVelocityTask : public RMLCartesianVelocityTaskBase
{
    friend class RMLCartesianVelocityTaskBase;

    double no_reference_timeout;
    base::Time time_of_last_reference;
    bool convert_to_position;
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
    virtual void  updateTarget(const TargetVector& target_vector,
                               RMLInputParameters* new_input_parameters);


    /** Convert from RMLInputParameters to orogen type*/
    virtual const ReflexxesInputParameters& fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out);

    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out);

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

