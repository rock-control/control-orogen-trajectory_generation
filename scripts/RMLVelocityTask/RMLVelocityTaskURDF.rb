require 'orocos'
require 'vizkit'

include Orocos

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run 'trajectory_generation::RMLVelocityTask' => 'interpolator',
           'joint_control::JointLimitCheckerTask' => 'limit_checker' do
   
   robot_model = Orocos.name_service.get 'robot_model'
   interpolator = Orocos.name_service.get 'interpolator'
   limit_checker = Orocos.name_service.get 'limit_checker'
   driver = Orocos.name_service.get 'driver'

   Orocos.conf.apply(interpolator, ['default'])
   interpolator.limits = robot_model.GetJointLimits()
   limit_checker.joint_limits = robot_model.GetJointLimits()

   driver.joint_status.connect_to interpolator.joint_state
   driver.joint_status.connect_to limit_checker.joint_status
   limit_checker.command.connect_to interpolator.cmd
   limit_checker.output_command.connect_to driver.command

   interpolator.configure
   interpolator.start
   limit_checker.configure
   limit_checker.start
  
   Vizkit.exec
    
end
