require 'orocos'
require 'pry'
require 'vizkit'

include Orocos

Orocos.initialize
Orocos.conf.load_dir('./config')

if !ARGV[0]
   puts "No driver task context name given" 
   puts "Usage: ruby task_trajectory.rb <Task_Context_Name>"
   exit(0)
end

driver_task = ARGV[0]

Orocos.run 'trajectory_generation::Task' => 'interpolator' do
    
   interpolator = Orocos.name_service.get 'interpolator'
   driver = Orocos.name_service.get driver_task
   Orocos.conf.apply(interpolator, ['default'])
   
   driver.joint_state.connect_to interpolator.joint_state
   driver.command.connect_to interpolator.cmd

   writer = interpolator.trajectory_target.writer
   traj = Types::Base::JointsTrajectory.new
   single_traj = Types::Base::JointTrajectory.new
   traj.names << "J_Foot"
   state = Types::Base::JointState.new
   state.position = 0.5
   single_traj << state
   state.position = -2.0
   single_traj << state
   state.position = 2.5
   single_traj << state
   state.position = 4.0
   single_traj << state
   state.position = 5.0
   single_traj << state
   state.position = 6.0
   single_traj << state
   traj.elements << single_traj
    
   interpolator.configure
   interpolator.start

   sleep(1)
   writer.write(traj)

   Vizkit.exec
    
end
