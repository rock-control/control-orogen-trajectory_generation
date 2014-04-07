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

Orocos.run 'trajectory_generation::Task' => 'interpolator', :wait => true 'trajectory_generation::TestPlant' => 'driver', :wait=>true do
    
   interpolator = Orocos.name_service.get_provides 'trajectory_generation::Task'
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
   traj.names << "J_Shoulder1_l"
   state = Types::Base::JointState.new
   state.position = 0.5
   state.speed = 0
   single_traj << state
   state.position = -0.0
   state.speed = 0
   single_traj << state
   state.position = 0.2
   state.speed = 0
   single_traj << state
   state.position = -0.02
   state.speed = 0
   single_traj << state
   state.position = 1.0
   state.speed = 0
   single_traj << state
   state.position = 0.2
   state.speed = 0
   single_traj << state
   traj.elements << single_traj
   
   constraints=Types::TrajectoryGeneration::JointMotionConstraintsSeries.new
   constr = empty_constraint
   constr.max_jerk = 8
   constr.max.effort = 6
   constr.max.speed = 2
   constraints << constr
   
   constr = empty_constraint
   constr.max_jerk = 0.2
   constraints << constr
   
   constr = empty_constraint
   constr.max_jerk = 50
   constr.max.speed = 0.05
   constraints << constr
   
   constr = empty_constraint
   constr.max_jerk = 80
   constr.max.effort = 60
   constr.max.speed = 30
   constraints << constr
   
   constr = empty_constraint
   constraints << constr
   
   constr = empty_constraint
   constraints << constr
   
   traj.motion_constraints << constraints
   
   interpolator.configure
   interpolator.start
   
   

   sleep(1)
   writer.write(traj)

   Vizkit.exec
    
end
