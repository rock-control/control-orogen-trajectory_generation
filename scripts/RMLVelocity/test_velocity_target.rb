require 'orocos'

Orocos.initialize
Orocos.conf.load_dir('.')

def range (min, max)
    rand() * (max-min) + min
end

Orocos.run "trajectory_generation::RMLVelocityTask" => "interpolator" do

    interpolator = Orocos::TaskContext.get "interpolator"
    Orocos.conf.apply(interpolator, ["default"])
    interpolator.configure
    interpolator.start
  
    initial_joint_state = Types::Base::Samples::Joints.new
    initial_joint_state.names = interpolator.limits.names

    interpolator.limits.elements.each do |elem|
       state = Types::Base::JointState.new
       state.position = state.speed = state.acceleration = 0
       initial_joint_state.elements << state
    end

    initial_joint_state.time = Types::Base::Time.now
    interpolator.joint_state.write(initial_joint_state)

    sleep(1)

    target = Types::Base::Commands::Joints.new
    target.names = interpolator.limits.names

    interpolator.limits.elements.each do |elem|
        state = Types::Base::JointState.new
        target.elements << state
    end

    # Drive to 20 random points
    joint_state = Types::Base::Samples::Joints.new
    reader = interpolator.output_sample.reader
    for i in 0..20
       target.elements.each do |elem|
          elem.speed = range(-1,1)
       end
       interpolator.velocity_target.write(target)
       for j in 0..300
           reader.read(joint_state)
           interpolator.joint_state.write(joint_state)
           sleep(0.01)
       end
    end
    
    
    puts "Press ENTER to stop!"
    STDIN.readline
end
