require 'orocos'

Orocos.initialize
Orocos.conf.load_dir('.')

def range (min, max)
    rand() * (max-min) + min
end

Orocos.run "trajectory_generation::Task" => "interpolator" do

    interpolator = Orocos::TaskContext.get "interpolator"
    Orocos.conf.apply(interpolator, ["default"])
    interpolator.configure
    interpolator.start
  
    initial_joint_state = Types::Base::Samples::Joints.new
    interpolator.limits.names.each do |name|
       initial_joint_state.names << name
    end
    interpolator.limits.elements.each do |elem|
       state = Types::Base::JointState.new
       state.position = (elem.max.position + elem.min.position)/2
       state.speed = 0
       state.acceleration = 0
       initial_joint_state.elements << state
    end

    initial_joint_state.time = Types::Base::Time.now
    interpolator.joint_state.write(initial_joint_state)

    sleep(1)

    target = Types::Base::JointsTrajectory.new
    interpolator.limits.names.each do |name|
        target.names << name
    end
    interpolator.limits.elements.each do |elem|
        traj = Types::Base::JointTrajectory.new
        state = Types::Base::JointState.new
        state.position = 0
        for i in 0..10
            state.position = 0.1*i#range(elem.min.position, elem.max.position)
            traj << state
        end
        state = Types::Base::JointState.new
        state.position = 0
        traj << state
        target.elements << traj
    end

    pp target
    interpolator.trajectory_target.write(target)

    puts "Press ENTER to stop!"
    STDIN.readline
end
