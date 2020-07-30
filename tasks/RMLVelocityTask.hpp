/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLVELOCITYTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLVELOCITYTASK_TASK_HPP

#include "trajectory_generation/RMLVelocityTaskBase.hpp"

namespace trajectory_generation{

class RMLVelocityTask : public RMLVelocityTaskBase
{
    friend class RMLVelocityTaskBase;

    base::samples::Joints joint_state;    /** From input port: Current joint state. Will only be used for initializing RML */
    base::samples::Joints current_sample; /** From input port: Current joint interpolator status (position/speed/acceleration)*/
    ConstrainedJointsCmd target;          /** From input port: Target joint position or speed.  */
    base::commands::Joints command;       /** To output port: Commanded joint position or speed.  */

    double no_reference_timeout;
    base::Time time_of_last_reference;
    bool convert_to_position;
    base::VectorXd max_pos_diff;

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

    /** Correct the given RMLOutputParameters if the difference between actual joint position and interpolator position is bigger than max_diff*/
    void correctInterpolatorState(RMLInputParameters *in,
                                  RMLOutputParameters *out,
                                  const base::samples::Joints &act,
                                  const base::VectorXd& max_diff);
public:
    RMLVelocityTask(std::string const& name = "trajectory_generation::RMLVelocityTask") : RMLVelocityTaskBase(name){}
    RMLVelocityTask(std::string const& name, RTT::ExecutionEngine* engine) : RMLVelocityTaskBase(name, engine){}
    ~RMLVelocityTask(){}
    bool configureHook();
    bool startHook(){return RMLVelocityTaskBase::startHook();}
    void updateHook(){RMLVelocityTaskBase::updateHook();}
    void errorHook(){RMLVelocityTaskBase::errorHook();}
    void stopHook(){RMLVelocityTaskBase::stopHook();}
    void cleanupHook(){RMLVelocityTaskBase::cleanupHook();}
};
}

#endif

