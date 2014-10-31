require 'orocos'

Orocos.initialize
Orocos.conf.load_dir('.')

def range (min, max)
    rand() * (max-min) + min
end

Orocos.run "trajectory_generation::Task" => "interpolator" do

    interpolator = Orocos::TaskContext.get "interpolator"
    Orocos.conf.apply(interpolator, ["default", "smooth"])
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
        state.position = (elem.max.position + elem.min.position)/2
        target.elements << state
    end

    # Drive to 20 random points
    for i in 0..20
       target.elements.each do |elem|
          elem.position = range(-1,1)
       end
       interpolator.position_target.write(target)
       interpolator.joint_state.write(initial_joint_state)
       sleep(3)
    end
    
    
    puts "Press ENTER to stop!"
    STDIN.readline
end
