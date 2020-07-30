require 'orocos'
require 'readline'

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run "trajectory_generation::RMLCartesianPositionTask" => "interpolator" do

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
    target.pose.position    = Types.base.Vector3d.new(1,2,3)
    target.pose.orientation = Types.base.Quaterniond.from_euler(Types.base.Vector3d.new(Math::PI,Math::PI/2,-Math::PI/2),2,1,0)
    target.time             = Types.base.Time.now
    target.source_frame     = "target_tip"
    target.target_frame     = "target_root"

    target_writer = interpolator.target.writer
    target_writer.write(target)

    command_reader = interpolator.command.reader
    while true
        command = command_reader.read
        if command
            puts "Target position: "    + target.pose.position[0].to_s  + " " + target.pose.position[1].to_s  + " " + target.pose.position[2].to_s
            puts "Commanded position: " + command.pose.position[0].to_s + " " + command.pose.position[1].to_s + " " + command.pose.position[2].to_s
            euler = target.pose.orientation.to_euler
            puts "Target orientation: "    + euler[0].to_s  + " " + euler[1].to_s  + " " + euler[2].to_s
            euler = command.pose.orientation.to_euler
            puts "Commanded orientation: " + euler[0].to_s  + " " + euler[1].to_s  + " " + euler[2].to_s
            puts "---------------------------------------------------"
        end
        sleep 0.01
    end

    Readline.readline("Press Enter to stop")
end
