require 'orocos'
require 'pry'

Orocos.initialize
Orocos.conf.load_dir('.')

def range (min, max)
    rand() * (max-min) + min
end

Orocos.run "trajectory_generation::Task" => "interpolator" do

    interpolator = Orocos::TaskContext.get "interpolator"
    Orocos.conf.apply(interpolator, ["default", "smooth"])
    interpolator.configure
    interpolator.start
  
    initial_joint_state = Types::Base::Samples::Joints.new
    initial_joint_state.names = interpolator.limits.names

    interpolator.limits.elements.each do |elem|
       state = Types::Base::JointState.new
       state.position = state.speed = state.acceleration = 0
       initial_joint_state.elements << state
    end

    initial_joint_state.time = Types::Base::Time.now
    interpolator.joint_state.write(initial_joint_state)

    sleep(1)

    target = Types::TrajectoryGeneration::ConstrainedJointsCmd.new
    target.names = interpolator.limits.names

    interpolator.limits.elements.each do |elem|
        state = Types::Base::JointState.new
        state.position = (elem.max.position + elem.min.position)/2
        target.elements << state
    end

    target.motion_constraints = interpolator.limits

    # Drive to 20 points and change speed, acceleration and jerk of the motion for every point
    for i in 0..20 
       target.elements.each do |elem|
          elem.position = range(-1,1) 
       end
       target.motion_constraints.elements.each do |elem|
          elem.max_jerk = elem.max_jerk * 1.5
          elem.max.acceleration = elem.max_jerk * 1.5
          elem.max.speed = elem.max_jerk * 1.5
       end
       interpolator.constrained_position_target.write(target)
       interpolator.joint_state.write(initial_joint_state)
       sleep(3)
    end
    
    
    puts "Press ENTER to stop!"
    STDIN.readline
end
