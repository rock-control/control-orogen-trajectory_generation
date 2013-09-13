/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_TESTPLANT_TASK_HPP
#define TRAJECTORY_GENERATION_TESTPLANT_TASK_HPP

#include "trajectory_generation/TestPlantBase.hpp"
#include "base/commands/Joints.hpp"
#include "base/Logging.hpp"
#include "base/Time.hpp"

namespace trajectory_generation {

class FakeJoint{
protected:
    base::JointLimitRange j_range;
    base::JointState j_state;
    base::JointState j_setpoint;
    base::JointState::MODE mode;
    base::Time t_prev;
    std::string name;
    bool first_cycle;

    template<typename T>
    void trunc_to_bounds(T& v, const T& min, const T& max){
        if(v<min)
            v = min;
        if(v>max)
            v = max;
    }

public:
    inline FakeJoint(const base::JointLimitRange& j_range,
                     const base::JointState& initial_state)
    {
        reset(initial_state, j_range);
    }

    inline void reset(const base::JointState& initial_state,
                      const base::JointLimitRange& j_range)
    {
        j_state = initial_state;
        j_setpoint = initial_state;
        this->j_range = j_range;
        first_cycle = true;
    }

    inline void step_dt(const base::Time& dt){
        if(mode == base::JointState::POSITION)
        {
            j_state.position = j_setpoint.position;
            j_state.speed = j_setpoint.speed;
            j_state.effort = j_setpoint.effort;

            trunc_to_bounds(j_setpoint.position,
                            j_range.min.position,
                            j_range.max.position);
            trunc_to_bounds(j_setpoint.speed,
                            j_range.min.speed,
                            j_range.max.speed);
            trunc_to_bounds(j_setpoint.effort,
                            j_range.min.effort,
                            j_range.max.effort);
        }
        else if(mode == base::JointState::SPEED)
        {
            j_state.speed = j_setpoint.speed;
            j_state.effort = j_setpoint.effort;

            trunc_to_bounds(j_setpoint.speed,
                            j_range.min.speed,
                            j_range.max.speed);
            trunc_to_bounds(j_setpoint.effort,
                            j_range.min.effort,
                            j_range.max.effort);

            j_state.position += j_setpoint.speed*dt.toSeconds();

            trunc_to_bounds(j_setpoint.position,
                            j_range.min.position,
                            j_range.max.position);
        }
        else if(mode == base::JointState::EFFORT)
        {
            j_state.effort = j_setpoint.effort;

            trunc_to_bounds(j_setpoint.effort,
                            j_range.min.effort,
                            j_range.max.effort);

            j_state.speed += j_setpoint.effort*dt.toSeconds();
            j_state.position += j_setpoint.speed*dt.toSeconds();

            trunc_to_bounds(j_setpoint.position,
                            j_range.min.position,
                            j_range.max.position);
            trunc_to_bounds(j_setpoint.speed,
                            j_range.min.speed,
                            j_range.max.speed);
        }
        t_prev = base::Time::now();
    }

    inline void step(base::Time time=base::Time::now()){
        if(first_cycle){
            t_prev = base::Time::now();
        }

        base::Time dt = time - t_prev;
        step_dt(dt);
    }

    inline void state(base::JointState& j_state){
        j_state = this->j_state;
    }

    inline void cmd(base::JointState& setpoint){
        if(mode != setpoint.getMode()){
            LOG_INFO("Switching ctrl mode for joint %s from %d to %d.", name.c_str(),
                     mode, setpoint.getMode() );
            mode = setpoint.getMode();
        }
        j_setpoint = setpoint;
    }

    inline const std::string& get_name(){return name;}

};

/*! \class TestPlant
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','trajectory_generation::TestPlant')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
class TestPlant : public TestPlantBase
{
    friend class TestPlantBase;
protected:
    double noise_mean;
    double noise_variance;
    base::commands::Joints j_cmd;
    base::samples::Joints j_state;
    base::JointLimits limits;
    std::map<std::string, base::JointState::MODE> j_modes;
    bool use_fixed_simulation_step_time;
    base::Time simulation_step_time;

    typedef std::map<std::string, FakeJoint> JointMap;
    JointMap joints;


public:
    /** TaskContext constructor for TestPlant
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
    TestPlant(std::string const& name = "trajectory_generation::TestPlant");

    /** TaskContext constructor for TestPlant
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         *
         */
    TestPlant(std::string const& name, RTT::ExecutionEngine* engine);

    /** Default deconstructor of TestPlant
         */
    ~TestPlant();

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

