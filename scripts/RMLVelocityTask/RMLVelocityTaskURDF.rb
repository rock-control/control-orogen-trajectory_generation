require 'orocos'
require 'vizkit'

include Orocos

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run 'trajectory_generation::RMLVelocityTask' => 'interpolator' do
   
   robot_model = Orocos.name_service.get 'robot_model'
   interpolator = Orocos.name_service.get 'interpolator'
   driver = Orocos.name_service.get 'driver'

   Orocos.conf.apply(interpolator, ['default'])
   limits = robot_model.GetJointLimits()
   interpolator.limits = limits

   driver.joint_state.connect_to interpolator.joint_state
   interpolator.command.connect_to driver.command

   max_jerk = []
   for i in 0..(limits.elements.length-1)
       max_jerk << 1    
   end
   interpolator.max_jerk = max_jerk

   interpolator.configure
   interpolator.start
  
   Vizkit.exec
    
end
