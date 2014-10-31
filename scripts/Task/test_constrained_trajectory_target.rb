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
    initial_joint_state.time = Types::Base::Time.now
    sleep(1)

    target = Types::TrajectoryGeneration::ConstrainedJointsTrajectory.new
    target.names = interpolator.limits.names

    # Drive a trajectory from 0 .. 1 and back to 0 and change speed, acceleration and jerk for every point
    interpolator.limits.elements.each do |elem|
        traj = Types::Base::JointTrajectory.new
        waypoint = Types::Base::JointState.new
        for i in 0..10
           waypoint.position = i*0.1
           traj << waypoint
        end
        for i in 0..10
           waypoint.position = (1-i*0.1)
           traj << waypoint
        end
        target.elements << traj
    end
    
    interpolator.limits.elements.each do |elem|
       motion_constraints = Types::TrajectoryGeneration::JointMotionConstraintsSeries.new
       for i in 0..10
         elem.max.speed = elem.max.speed*1.2
         elem.max.acceleration = elem.max.acceleration*1.2
         elem.max_jerk = elem.max_jerk*1.2
         motion_constraints << elem
       end
       for i in 0..10
         elem.max.speed = elem.max.speed/1.2
         elem.max.acceleration = elem.max.acceleration/1.2
         elem.max_jerk = elem.max_jerk/1.2
         motion_constraints << elem
       end
       target.motion_constraints << motion_constraints
    end

    interpolator.constrained_trajectory_target.write(target)
    interpolator.joint_state.write(initial_joint_state)
    
    puts "Press ENTER to stop!"
    STDIN.readline
end
