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

        joints.insert( std::make_pair(limits.names[i],
                                      FakeJoint(limits.elements[i],
                                                base::JointState::Position(0))));
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
        if(use_fixed_simulation_step_time){
            //Simulate
            it->second.step_dt(simulation_step_time);
        }
        else{
            it->second.step(cur_time);
        }
        //Set new state for output
        it->second.state(j_state[it->second.get_name()]);
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
