require 'orocos'
require 'vizkit'

include Orocos

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run 'trajectory_generation::RMLVelocityTask' => 'interpolator' do
    
   interpolator = Orocos.name_service.get 'interpolator'
   driver = Orocos.name_service.get 'driver'

   Orocos.conf.apply(interpolator, ['default'])

   driver.joint_state.connect_to interpolator.joint_state
   driver.command.connect_to interpolator.command

   interpolator.configure
   interpolator.start

   Vizkit.exec
    
end
