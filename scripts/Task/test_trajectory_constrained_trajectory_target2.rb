require 'orocos'
require 'pry'
require 'vizkit'

include Orocos

Orocos.initialize
Orocos.conf.load_dir('./config')

def empty_constraint
    constr=Types::TrajectoryGeneration::JointMotionConstraints.new
    constr.min = Types::Base::JointState.new
    constr.max = Types::Base::JointState.new
    constr.max_jerk = Float::NAN
    constr
end

def empty_state
    state = Types::Base::JointState.new
    state
end

def get_minimum_jerk_to_cover_distance_at_time(distance, time)
    jerk = (6*distance)/(time**3)
    jerk
end

def get_velocity_at_time_assuming_constant_acceleration(jerk, time, start_velocity=0)
    velocity = start_velocity + (jerk*time**2.0)/2.0
    velocity
end

def get_time_to_accelerate_to_velocity(veclocity, jerk)
    time = Math.sqrt(velocity/(2.0*jerk))
    time
end

def get_acceleration_at_time_assuming_constant_acceleration(jerk, time)
    acceleration = jerk * time
    acceleration
end

def get_velocity_accelerated_to_in_time_with_jerk(time, jerk)
    velocity = (jerk*(time**2.0)/2)
    return velocity
end

def get_time_to_accelerate_to(acceleration, jerk)
    time = acceleration/jerk
    time
end


def get_target_and_motion_costraints(start_position, end_position, time)
    distance = end_position-start_position
    jerk = get_minimum_jerk_to_cover_distance_at_time(distance, time)
    
    acceleration = get_acceleration_at_time_assuming_constant_acceleration(jerk, time/2.0)
    
    velocity = get_velocity_accelerated_to_in_time_with_jerk(jerk, time)
    
#    acceleration = get_acceleration_at_time_assuming_constant_acceleration(jerk, time)
    
    constr = empty_constraint
    constr.max.speed = 9000
    constr.max.effort = 9000
    constr.max_jerk = jerk*2
    
    state = empty_state
    state.position = end_position
    state.speed = velocity
    state.effort = acceleration
    
    return state, constr
end

Orocos.run 'trajectory_generation::Task' => 'interpolator', 'trajectory_generation::TestPlant' => 'driver' do
    
   interpolator = Orocos.name_service.get 'interpolator'
   driver = Orocos.name_service.get 'driver'
   Orocos.conf.apply(interpolator, ['default'])
   pp interpolator.limits
   puts "Warum sind die min Werte 0 und nicht nan? Types::Base::JointState.new inittialisiert ich mit nan. Dieses verhalten sollte sich nicht aendern."

   driver.limits = interpolator.limits
   driver.simulation_step_time = interpolator.cycle_time
   driver.configure
   driver.start
  
   driver.joint_state.connect_to interpolator.joint_state
   driver.cmd.connect_to interpolator.cmd
   
   writer = interpolator.constrained_trajectory_target.writer
   traj = Types::TrajectoryGeneration::ConstrainedJointsTrajectory.new
   single_traj = Types::Base::JointTrajectory.new
   constraints=Types::TrajectoryGeneration::JointMotionConstraintsSeries.new
   traj.names << "J_Shoulder1_l"
   state, constr = get_target_and_motion_costraints(0, 0.7, 1.2)
   constr.max_jerk = 8
   constr.max.speed = 3
   state.speed = 2.5
   #state.speed = 1
   #constr.max_jerk = state.speed*9
   pp state
   binding.pry
   
   single_traj << state
   constraints << constr
   
   state.position = 1.0
   state.speed = 0
   single_traj << state
   constr.max_jerk = 90
   constraints << constr

   traj.elements << single_traj
   traj.motion_constraints << constraints
   
   interpolator.configure
   interpolator.start
  
   sleep(1)
   writer.write(traj)

   Vizkit.exec
    
end
