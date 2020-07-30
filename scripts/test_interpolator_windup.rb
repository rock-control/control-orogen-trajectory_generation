require 'orocos'
require 'readline'
require 'csv'

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run "trajectory_generation::RMLVelocityTask" => "interpolator" do

    interpolator = Orocos::TaskContext.get "interpolator"
    Orocos.conf.apply(interpolator, ["default", "windup"], true)
    interpolator.configure
    interpolator.start

    joint_state = Types::Base::Samples::Joints.new
    joint_state.names = interpolator.motion_constraints.names
    joint_state.names.each do
        state = Types::Base::JointState.new
        state.position = 0.0
        joint_state.elements << state
    end
    joint_state.time = Types::Base::Time.now

    Readline.readline("Press Enter to start")

    joint_state_writer = interpolator.joint_state.writer
    joint_state_writer.write(joint_state)

    Readline.readline("Press Enter to send target")

    target = Types::Base::Samples::Joints.new
    target.names = interpolator.motion_constraints.names
    cmd = Types::Base::JointState.new
    cmd.speed = 0.3
    target.elements << cmd
    cmd = Types::Base::JointState.new
    cmd.speed = 0.5
    target.elements << cmd

    target_writer = interpolator.target.writer
    target_writer.write(target)

    command_reader = interpolator.command.reader
    start = Types.base.Time.now
    CSV.open("test.csv", "w", {:col_sep => "\t"}) do |csv|
        while true
            command = command_reader.read
            if command
                #print "Target velocity: "
                arr = [(Types.base.Time.now - start).to_s + " "]
                target.elements.each { |e|  arr << e.speed.to_s + " "}
                #print "\nCommanded velocity: "
                command.elements.each { |e| arr << e.speed.to_s + " "}
                #print "\nCommanded position: "
                command.elements.each { |e| arr << e.position.to_s + " "}
                #arr << "\n"
                csv << arr
                #puts "\n---------------------------------------------------"
                joint_state = command
            end
            sleep 0.01
            if (Types.base.Time.now - start) > 5
                 joint_state_writer.write(joint_state)
            end
            if (Types.base.Time.now - start) > 10
                break
           end
        end
    end
end
