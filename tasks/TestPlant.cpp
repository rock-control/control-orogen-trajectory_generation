/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "TestPlant.hpp"

using namespace trajectory_generation;

TestPlant::TestPlant(std::string const& name)
    : TestPlantBase(name), noise_mean(0), noise_variance(0)
{
}

TestPlant::TestPlant(std::string const& name, RTT::ExecutionEngine* engine)
    : TestPlantBase(name, engine), noise_mean(0), noise_variance(0)
{
}

TestPlant::~TestPlant()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See TestPlant.hpp for more detailed
// documentation about them.

bool TestPlant::configureHook()
{
    if (! TestPlantBase::configureHook())
        return false;
    base::JointLimits limits = _limits.get();
    noise_mean = _noise_mean.get();
    noise_variance = _noise_variance.get();

    j_cmd.resize(limits.size());
    j_state.resize(limits.size());

    j_cmd.names = limits.names;
    j_state.names = limits.names;

    simulation_step_time = base::Time::fromSeconds(_simulation_step_time.get());
    use_fixed_simulation_step_time = _use_fixed_simulation_step_time.get();

    for(uint i=0; i<limits.size(); i++){
        base::JointState initial_state;
        initial_state.position = std::max(0., limits[i].min.position);
        initial_state.speed = 0;
        initial_state.effort = 0;
        initial_state.raw = 0;
        joints.insert( std::make_pair(limits.names[i],
                                      FakeJoint(limits.elements[i],
                                                initial_state,
                                                limits.names[i])));
    }
    return true;
}
bool TestPlant::startHook()
{
    if (! TestPlantBase::startHook())
        return false;
    return true;
}
void TestPlant::updateHook()
{
    TestPlantBase::updateHook();

    //Simulation step for joints
    base::Time cur_time = base::Time::now();
    for(JointMap::iterator it = joints.begin(); it!=joints.end();
        ++it)
    {
        LOG_DEBUG("Running update for Joint %s", it->first.c_str());

        if(use_fixed_simulation_step_time){
            //Simulate
            LOG_DEBUG("Performing simulation step with fixed step time %d (seconds)", simulation_step_time.toSeconds());
            it->second.step_dt(simulation_step_time);
            LOG_DEBUG("Simulation step done");
        }
        else{
            LOG_DEBUG("Performing simulation step with flexible step time. Current time stamp %d (seconds)", cur_time.toSeconds());
            it->second.step(cur_time);
            LOG_DEBUG("Simulation step done");
        }
        //Set new state for output
        LOG_DEBUG("Updating joint state. Iterator name: %s, Joint name: %s.", it->first.c_str(), it->second.get_name().c_str());
        it->second.get_state(j_state[it->second.get_name()]);
        LOG_DEBUG("Joint state update done");
    }

    //Process setpoint input
    while( _cmd.read( j_cmd, false ) == RTT::NewData )
    {
        LOG_DEBUG("New command sample read");
        std::string jname;
        for(uint i=0; i<j_cmd.size(); i++){
            jname = j_cmd.names[i];
            JointMap::iterator it = joints.find(jname);
            assert(it != joints.end());
            it->second.cmd(j_cmd.elements[i]);
        }
    }

    //Write output
    j_state.time = base::Time::now();
    _joint_state.write(j_state);

}
void TestPlant::errorHook()
{
    TestPlantBase::errorHook();
}
void TestPlant::stopHook()
{
    TestPlantBase::stopHook();
}
void TestPlant::cleanupHook()
{
    TestPlantBase::cleanupHook();
}
