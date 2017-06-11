require 'orocos'
require 'readline'

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run "trajectory_generation::RMLPositionTask" => "interpolator" do
  
    interpolator = Orocos::TaskContext.get "interpolator"
    Orocos.conf.apply(interpolator, ["default"], true)
    interpolator.configure
    interpolator.start

    joint_state = Types::Base::Samples::Joints.new
    joint_state.names = interpolator.motion_constraints.names
    joint_state.names.each do 
        state = Types::Base::JointState.new
        state.position = 0
        joint_state.elements << state
    end
    joint_state.time = Types::Base::Time.now

    Readline.readline("Press Enter to start")

    puts "Writing joint state"
    joint_state_writer = interpolator.joint_state.writer 
    joint_state_writer.write(joint_state)

    Readline.readline("Press Enter to target")

    target = Types::Base::Samples::Joints.new
    target.names = interpolator.motion_constraints.names
    cmd = Types::Base::JointState.new
    cmd.position = 2.0
    cmd.speed = 0.0
    target.elements << cmd
    cmd = Types::Base::JointState.new
    cmd.position  = 1.0
    cmd.speed = 0.0
    target.elements << cmd

    puts "Writing target"
    target_writer = interpolator.target.writer 
    target_writer.write(target)

    Readline.readline("Press Enter to stop")
end
