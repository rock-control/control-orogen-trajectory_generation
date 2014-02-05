require 'orocos'
require 'vizkit'

include Orocos

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run 'trajectory_generation::RMLVelocityTask' => 'interpolator' do
   
   Orocos.log_all 
  
   robot_model = Orocos.name_service.get 'robot_model'
   interpolator = Orocos.name_service.get 'interpolator'
   driver = Orocos.name_service.get 'driver'

   Orocos.conf.apply(interpolator, ['default'])
   interpolator.limits = robot_model.GetJointLimits()

   driver.joint_status.connect_to interpolator.joint_state
   interpolator.cmd.connect_to driver.command

   interpolator.configure
   interpolator.start
  
   Vizkit.exec
    
end
