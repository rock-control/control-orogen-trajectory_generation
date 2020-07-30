require 'orocos'
require 'readline'

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run "trajectory_generation::RMLCartesianVelocityTask" => "interpolator" do

    interpolator = Orocos::TaskContext.get "interpolator"
    Orocos.conf.apply(interpolator, ["default"], true)
    interpolator.configure
    interpolator.start

    cartesian_state                  = Types.base.samples.CartesianState.new
    cartesian_state.pose.position    = Types.base.Vector3d.new(0,0,0)
    cartesian_state.pose.orientation = Types.base.Quaterniond.from_euler(Types.base.Vector3d.new(-Math::PI,-Math::PI/2,Math::PI/2),2,1,0)
    cartesian_state.time             = Types.base.Time.now
    cartesian_state.source_frame     = "current_state_tip"
    cartesian_state.target_frame     = "current_state_root"

    Readline.readline("Press Enter to start")

    cartesian_state_writer = interpolator.cartesian_state.writer
    cartesian_state_writer.write(cartesian_state)

    Readline.readline("Press Enter to send target")

    target                  = Types.base.samples.CartesianState.new
    target.twist.linear         = Types.base.Vector3d.new(1,2,3)
    target.twist.angular = Types.base.Vector3d.new(Math::PI,Math::PI/2,-Math::PI/2)
    target.time             = Types.base.Time.now
    target.source_frame      = "target_tip"
    target.target_frame      = "target_root"

    target_writer = interpolator.target.writer
    target_writer.write(target)

    command_reader = interpolator.command.reader
    while true
        command = command_reader.read
        if command
            puts "Target twist.linear: "    + target.twist.linear[0].to_s  + " " + target.twist.linear[1].to_s  + " " + target.twist.linear[2].to_s
            puts "Commanded twist.linear: " + command.twist.linear[0].to_s + " " + command.twist.linear[1].to_s + " " + command.twist.linear[2].to_s
            puts "Target angular twist.linear: "    + target.twist.angular[0].to_s  + " " + target.twist.angular[1].to_s  + " " + target.twist.angular[2].to_s
            puts "Commanded angular twist.linear: " + command.twist.angular[0].to_s + " " + command.twist.angular[1].to_s + " " + command.twist.angular[2].to_s
            puts "---------------------------------------------------"
        end
        sleep 0.01
    end
end
