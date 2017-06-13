/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLPOSITION_TASK_HPP
#define TRAJECTORY_GENERATION_RMLPOSITION_TASK_HPP

#include "trajectory_generation/RMLPositionBase.hpp"

namespace trajectory_generation{
class RMLPosition : public RMLPositionBase
{
    friend class RMLPositionBase;
protected:
    /** Update the motion constraints of a particular element*/
    virtual void updateMotionConstraints(const MotionConstraint& constraint,
                                         const size_t idx,
                                         RMLInputParameters* new_input_parameters);

    /** Read the current state from port and return position and flow status*/
    virtual RTT::FlowStatus getCurrentState(CurrentStateData& current_state) = 0;

    /** Read target vector and return the flow status*/
    virtual RTT::FlowStatus getTarget(TargetData& target_vector) = 0;

    /** Update the RML input parameters with the new target */
    virtual void updateTarget(const TargetData& target_vector,
                              RMLInputParameters* new_input_parameters);

    /** Perform one step of online trajectory generation (call the RML algorithm with the given parameters). Return the RML result value*/
    virtual ReflexxesResultValue performOTG(RMLInputParameters* new_input_parameters,
                                            RMLOutputParameters* new_output_parameters,
                                            RMLFlags *rml_flags);

    /** Call echo() method for rml input and output parameters*/
    virtual void printParams(const RMLInputParameters& in, const RMLOutputParameters& out);

    /** Convert from RMLInputParameters to orogen type*/
    virtual const ReflexxesInputParameters& fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out);

    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out);

    /** Velocity watchdog: Throw if time_last_reference is bigger than timeout */
    void checkVelocityTimeout(const base::Time time_last_reference, const double timeout);

public:
    RMLPosition(std::string const& name = "trajectory_generation::RMLPosition") : RMLPositionBase(name){}
    RMLPosition(std::string const& name, RTT::ExecutionEngine* engine) : RMLPositionBase(name, engine){}
    ~RMLPosition(){}
    bool configureHook();
    bool startHook(){return RMLPositionBase::startHook();}
    void updateHook(){RMLPositionBase::updateHook();}
    void errorHook(){RMLPositionBase::errorHook();}
    void stopHook(){RMLPositionBase::stopHook();}
    void cleanupHook(){RMLPositionBase::cleanupHook();}
};
}

#endif

