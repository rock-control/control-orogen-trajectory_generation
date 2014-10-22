require 'orocos'
require 'orocos/async'

Orocos.initialize
Orocos.conf.load_dir('.')

Orocos.run "trajectory_generation::Task" => "interpolator" do

    interpolator = Orocos::TaskContext.get "interpolator"
    proxy = Orocos::Async.get "interpolator"
    Orocos.conf.apply(interpolator, ["default"])
    interpolator.configure
    interpolator.start
  
    initial_joint_state = Types::Base::Samples::Joints.new
    interpolator.limits.names.each do |name|
       state = Types::Base::JointState.new
       state.position = 0
       state.speed = 0
       state.acceleration = 0
       initial_joint_state.elements << state
       initial_joint_state.names << name
    end

    interpolator.joint_state.write(initial_joint_state)

    sleep(1)

    target = Types::Base::Commands::Joints.new
    state = Types::Base::JointState.new
    state.position = 1.0
    target.elements << state
    target.names << "Joint_one"

    interpolator.position_target.write(target)
    
    
    puts "Press ENTER to stop!"
    STDIN.readline
end
