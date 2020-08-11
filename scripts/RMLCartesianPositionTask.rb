require 'orocos'
require 'readline'

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run "trajectory_generation::RMLCartesianPositionTask" => "interpolator", :output=>nil do

    interpolator = Orocos::TaskContext.get "interpolator"
    Orocos.conf.apply(interpolator, ["default"], true)
    interpolator.configure
    interpolator.start

    cartesian_state                  = Types.base.samples.RigidBodyStateSE3.new
    cartesian_state.pose.position    = Types.base.Vector3d.new(0,0,0)
    cartesian_state.pose.orientation = Types.base.Quaterniond.from_euler(Types.base.Vector3d.new(-Math::PI,-Math::PI/2,Math::PI/2),2,1,0)
    cartesian_state.time             = Types.base.Time.now
    cartesian_state.frame_id        = "current_state_root"

    Readline.readline("Press Enter to start")

    cartesian_state_writer = interpolator.cartesian_state.writer
    cartesian_state_writer.write(cartesian_state)

    Readline.readline("Press Enter to send target")

    target             = Types.base.samples.RigidBodyState.new
    target.position    = Types.base.Vector3d.new(1,2,3)
    target.orientation = Types.base.Quaterniond.from_euler(Types.base.Vector3d.new(Math::PI,Math::PI/2,-Math::PI/2),2,1,0)
    target.time        = Types.base.Time.now
    target.sourceFrame = "target_tip"
    target.targetFrame = "target_root"

    target_writer = interpolator.target.writer
    target_writer.write(target)
    command_reader = interpolator.command.reader
    start_time = Types.base.Time::now
    sleep 5
    while true
        command = command_reader.read
        if command
            #puts "Target position: "    + target.position[0].to_s  + " " + target.position[1].to_s  + " " + target.position[2].to_s
            #puts "Commanded position: " + command.pose.position[0].to_s + " " + command.pose.position[1].to_s + " " + command.pose.position[2].to_s
            #euler = target.orientation.to_euler
            #puts "Target orientation: "    + euler[0].to_s  + " " + euler[1].to_s  + " " + euler[2].to_s
            #euler = command.pose.orientation.to_euler
            #puts "Commanded orientation: " + euler[0].to_s  + " " + euler[1].to_s  + " " + euler[2].to_s
            #puts "---------------------------------------------------"
        end
        #cur_time = Types.base.Time::now
        #if cur_time - start_time > 5
        #    puts "Setting new Target"
        #    target.position = Types.base.Vector3d.new(rand(),rand()*2,rand()*3)
        #    target.orientation = Types.base.Quaterniond.from_euler(Types.base.Vector3d.new(rand(),rand(),rand()),2,1,0)
        #    target.time = cur_time
        #    target_writer.write target
        #    start_time = cur_time
        #end
        sleep 0.01
    end

    Readline.readline("Press Enter to stop")
end
