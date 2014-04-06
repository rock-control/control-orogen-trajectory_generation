require 'orocos'
require 'pry'
require 'vizkit'

include Orocos

Orocos.initialize
Orocos.conf.load_dir('./config')

Orocos.run 'trajectory_generation::TestPlant' => 'driver' do
    
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

   writer = interpolator.trajectory_target.writer
   traj = Types::Base::JointsTrajectory.new
   single_traj = Types::Base::JointTrajectory.new
   traj.names << "J_Shoulder1_l"
   state = Types::Base::JointState.new
   state.position = 0.5
   single_traj << state
   state.position = -0.0
   single_traj << state
   state.position = 0.2
   single_traj << state
   state.position = -0.02
   single_traj << state
   state.position = 1.0
   single_traj << state
   state.position = 0.2
   single_traj << state
   traj.elements << single_traj
    
   interpolator.configure
   interpolator.start

   sleep(1)
   writer.write(traj)

   Vizkit.exec
    
end
