/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLCARTESIANVELOCITYTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLCARTESIANVELOCITYTASK_TASK_HPP

#include "trajectory_generation/RMLCartesianVelocityTaskBase.hpp"

namespace trajectory_generation{

/** Velocity based implementation of a Cartesian RML task. The given target has to have valid velocity entries. */
class RMLCartesianVelocityTask : public RMLCartesianVelocityTaskBase
{
    friend class RMLCartesianVelocityTaskBase;
protected:
    /** Update the motion constraints of a particular joint*/
    virtual void updateMotionConstraints(const MotionConstraint& constraint,
                                         const size_t idx,
                                         RMLInputParameters* new_input_parameters);

    /** Perform one step of online trajectory generation (call the RML algorithm with the given parameters). Return the RML result value*/
    virtual ReflexxesResultValue performOTG(RMLInputParameters* new_input_parameters,
                                            RMLOutputParameters* new_output_parameters,
                                            RMLFlags *rml_flag);

    /** Write the generated trajectory to port*/
    virtual void writeCommand(const RMLOutputParameters& new_output_parameters);

    /** Call echo() method for rml input and output parameters*/
    virtual void printParams();

    /** Update the current state of a particular joint*/
    virtual void updateCurrentState(const base::JointState &state,
                                    const size_t idx,
                                    RMLInputParameters* new_input_parameters);

    /** Update the target of a particular joint*/
    virtual void updateTarget(const base::JointState &cmd,
                              const size_t idx,
                              RMLInputParameters* new_input_parameters);

    /** Convert from RMLInputParameters to orogen type*/
    virtual const ReflexxesInputParameters& fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out);

    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out);


public:
    /** TaskContext constructor for RMLCartesianVelocityTask
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
    RMLCartesianVelocityTask(std::string const& name = "trajectory_generation::RMLCartesianVelocityTask");

    /** TaskContext constructor for RMLCartesianVelocityTask
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         *
         */
    RMLCartesianVelocityTask(std::string const& name, RTT::ExecutionEngine* engine);

    /** Default deconstructor of RMLCartesianVelocityTask
         */
    ~RMLCartesianVelocityTask();

    /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
    bool configureHook();

    /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
    bool startHook();

    /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
    void updateHook();

    /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
    void errorHook();

    /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
    void stopHook();

    /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
    void cleanupHook();
};
}

#endif

