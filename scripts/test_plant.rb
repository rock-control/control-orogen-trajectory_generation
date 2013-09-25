require 'pry'
require 'orocos'

Orocos.initialize
Orocos.conf.load_dir('./test_data')

task = Orocos.name_service.get 'orogen_default_trajectory_generation__TestPlant'
Orocos.conf.apply(task, ['aila'])
task.configure
task.start
binding.pry
