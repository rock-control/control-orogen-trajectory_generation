/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLVELOCITY_TASK_HPP
#define TRAJECTORY_GENERATION_RMLVELOCITY_TASK_HPP

#include "trajectory_generation/RMLVelocityBase.hpp"

namespace trajectory_generation{
class RMLVelocity : public RMLVelocityBase
{
    friend class RMLVelocityBase;

protected:    
    double no_reference_timeout;
    base::Time time_of_last_reference;
    bool convert_to_position;

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
    RMLVelocity(std::string const& name = "trajectory_generation::RMLVelocity") : RMLVelocityBase(name){}
    RMLVelocity(std::string const& name, RTT::ExecutionEngine* engine) : RMLVelocityBase(name, engine){}
    ~RMLVelocity(){}
    bool configureHook();
    bool startHook(){return RMLVelocityBase::startHook();}
    void updateHook(){RMLVelocityBase::updateHook();}
    void errorHook(){RMLVelocityBase::errorHook();}
    void stopHook(){RMLVelocityBase::stopHook();}
    void cleanupHook(){RMLVelocityBase::cleanupHook();}
};
}

#endif

