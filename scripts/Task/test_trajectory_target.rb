require 'orocos'

Orocos.initialize
Orocos.conf.load_dir('.')

def range (min, max)
    rand() * (max-min) + min
end

Orocos.run "trajectory_generation::Task" => "interpolator" do

    interpolator = Orocos::TaskContext.get "interpolator"
    Orocos.conf.apply(interpolator, ["default", "reactive"])
    interpolator.configure
    interpolator.start
  
    initial_joint_state = Types::Base::Samples::Joints.new
    initial_joint_state.names = interpolator.limits.names

    interpolator.limits.elements.each do |elem|
       state = Types::Base::JointState.new
       state.position = state.speed = state.acceleration = 0
       initial_joint_state.elements << state
    end
    sleep(1)

    target = Types::Base::JointsTrajectory.new
    target.names = interpolator.limits.names

    # Drive a trajectory from 0 .. 1 and back to zero (20 steps)
    interpolator.limits.elements.each do |elem|
        traj = Types::Base::JointTrajectory.new
        state = Types::Base::JointState.new
        state.position = 0
        for i in 0..10
            state.position = 0.1*i
            traj << state
        end
        for i in 0..10
            state.position = (1-0.1*i)
            traj << state
        end
        target.elements << traj
    end

    initial_joint_state.time = Types::Base::Time.now
    interpolator.trajectory_target.write(target)
    interpolator.joint_state.write(initial_joint_state)

    puts "Press ENTER to stop!"
    STDIN.readline
end
