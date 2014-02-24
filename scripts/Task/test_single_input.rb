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

   interpolator.configure
   interpolator.start

   Vizkit.exec
    
end
