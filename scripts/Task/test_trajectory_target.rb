require 'rock/bundle'
require 'pry'
require 'orocos/async'

Orocos.initialize
Orocos.conf.load_dir('.')

def range (min, max)
    rand() * (max-min) + min
end

joint_names = ["J_Elbow_l" , "J_Elbow_r" , "J_Foot" , "J_Forearm_l" , "J_Forearm_r" , "J_Hip" , "J_Knees" , "J_Shoulder1_l" , "J_Shoulder1_r" , "J_Shoulder2_l" , "J_Shoulder2_r" , "J_UpperArm_l" , "J_UpperArm_r" , "J_Waist" , "J_Wrist1_l" , "J_Wrist1_r" , "J_Wrist2_l" , "J_Wrist2_r"]

points = [[-1.24314 , 1.26523 , 0 , -0.466476 , 0.484792 , 2.16 , 2.291 , 0.8016 , -0.822251 , -1.28651 , 1.26416 , -1.05 , 1.04 , 0 , -0.370337 , -0.378411 , -0.247321 , -0.25931],
[-1.33436 , 1.35441 , 0 , -0.459556 , 0.480003 , 2.16 , 2.291 , 0.87492 , -0.89566 , -1.25956 , 1.23553 , -1.05 , 1.04 , 0 , -0.39929 , -0.40742 , -0.242391 , -0.257901],
[-1.40711 , 1.42568 , 0 , -0.453433 , 0.45542 , 2.16 , 2.291 , 0.938083 , -0.966975 , -1.23747 , 1.22495 , -1.05 , 1.06 , 0 , -0.423106 , -0.418138 , -0.24325 , -0.254398],
[-1.45544 , 1.47309 , 0 , -0.449019 , 0.452464 , 2.16 , 2.291 , 0.982881 , -1.01225 , -1.22249 , 1.20926 , -1.05 , 1.06 , 0 , -0.439308 , -0.434071 , -0.246693 , -0.259656],
[-1.47397 , 1.49128 , 0 , -0.44724 , 0.430894 , 2.16 , 2.291 , 1.00076 , -1.03852 , -1.21668 , 1.21716 , -1.05 , 1.08 , 0 , -0.445607 , -0.426007 , -0.248716 , -0.254982],
[-1.45543 , 1.47308 , 0 , -0.448996 , 0.452489 , 2.16 , 2.291 , 0.982883 , -1.01225 , -1.22251 , 1.20925 , -1.05 , 1.06 , 0 , -0.439316 , -0.434058 , -0.246687 , -0.259669],
[-1.4071 , 1.42566 , 0 , -0.453389 , 0.455466 , 2.16 , 2.291 , 0.938085 , -0.966971 , -1.23751 , 1.22492 , -1.05 , 1.06 , 0 , -0.423124 , -0.41811 , -0.243239 , -0.254424],
[-1.33435 , 1.35438 , 0 , -0.459495 , 0.480067 , 2.16 , 2.291 , 0.874923 , -0.895655 , -1.25962 , 1.23549 , -1.05 , 1.04 , 0 , -0.399319 , -0.407375 , -0.242379 , -0.257938],
[-1.24312 , 1.26519 , 0 , -0.466403 , 0.48487 , 2.16 , 2.291 , 0.801604 , -0.822243 , -1.28659 , 1.2641 , -1.05 , 1.04 , 0 , -0.37038 , -0.378348 , -0.247308 , -0.259355],
[-1.14178 , 1.16643 , 0 , -0.473257 , 0.448963 , 2.16 , 2.291 , 0.726255 , -0.759356 , -1.31563 , 1.31599 , -1.05 , 1.08 , 0 , -0.339323 , -0.32588 , -0.259064 , -0.255807],
[-1.02788 , 1.05592 , 0 , -0.329487 , 0.393011 , 2.16 , 2.291 , 0.68223 , -0.694934 , -1.41839 , 1.37673 , -1.2 , 1.14 , 0 , -0.230265 , -0.264737 , -0.245119 , -0.258303],
[-1.77711 , 1.78965 , 0 , -0.266587 , 0.224871 , 2.16 , 2.591 , 1.08126 , -1.13721 , -1.24675 , 1.28751 , -1.2 , 1.26 , 0 , -0.413098 , -0.354689 , -0.245314 , -0.256398],
[-1.72824 , 1.74148 , 0 , -0.272844 , 0.330222 , 2.16 , 2.591 , 1.00511 , -1.02008 , -1.25964 , 1.20093 , -1.2 , 1.16 , 0 , -0.399223 , -0.442629 , -0.214908 , -0.255167],
[-1.67318 , 1.68723 , 0 , -0.424992 , 0.416088 , 2.16 , 2.591 , 0.867123 , -0.908782 , -1.15147 , 1.15002 , -1.05 , 1.08 , 0 , -0.516341 , -0.494678 , -0.244732 , -0.258631],
[-1.61189 , 1.62689 , 0 , -0.432618 , 0.46179 , 2.16 , 2.591 , 0.794969 , -0.816755 , -1.172 , 1.14106 , -1.05 , 1.04 , 0 , -0.493938 , -0.503875 , -0.225047 , -0.252951],
[-1.54422 , 1.56033 , 0 , -0.440227 , 0.507708 , 2.16 , 2.591 , 0.723289 , -0.726124 , -1.19415 , 1.13758 , -1.05 , 1 , 0 , -0.469858 , -0.507467 , -0.211236 , -0.252821],
[-1.46987 , 1.48727 , 0 , -0.447746 , 0.55375 , 2.16 , 2.591 , 0.651771 , -0.636929 , -1.21787 , 1.13945 , -1.05 , 0.96 , 0 , -0.444163 , -0.505556 , -0.20329 , -0.257819 ]]

Orocos.run "trajectory_generation::Task" => "interpolator" do

    dispatcher = Orocos::TaskContext.get "aila_dispatcher"
    interpolator = Orocos::TaskContext.get "aila_interpolation_position"
    robot_interface = Orocos::TaskContext.get "aila_interpolation_position"
    #Orocos.conf.apply(interpolator, ["default", "sherpa_manipulator"])
    #interpolator.configure
    #interpolator.start
  
    initial_joint_state = Types::Base::Samples::Joints.new
    initial_joint_state.names = interpolator.limits.names

    interpolator.limits.elements.each do |elem|
       state = Types::Base::JointState.new
       state.position = state.speed = state.acceleration = 0
       initial_joint_state.elements << state
    end
    sleep(1)

    target = Types::Base::JointsTrajectory.new
    target.names = joint_names
     
    points_inv = []
    for i in 0..(points[0].size()-1)
       row = []
       for j in 0..(points.size()-1)
           row << points[j][i]
       end
       points_inv << row
    end

    points.each do |pt|
       it = 0
       pt.each do |pos|
          idx = interpolator.limits.names.index(joint_names[it])
          range = interpolator.limits.elements[idx]
          if pos >= range.max.position
             puts "Joint " + joint_names[it] + " exceeds max pos! Max Pos is " + range.max.position.to_s + " and target is " + pos.to_s
          elsif pos <= range.min.position
             puts "Joint " + joint_names[it] + " exceeds min pos! Min Pos is " + range.min.position.to_s + " and target is " + pos.to_s
          end
          it = it + 1
       end
    end
   
    points_inv.each do |joint_traj|
        
        traj = Types::Base::JointTrajectory.new
        joint_traj.each do |point|
           state = Types::Base::JointState.new
           state.position = point
           traj << state
        end
        target.elements	 << traj
    end

    arr = []
    points.each do |elem|
       pt = Types::Base::Commands::Joints.new
       elem.each do |e|
          state = Types::Base::JointState.new
          state.position = e
          pt.elements << state
       end
       pt.names = joint_names
       arr << pt
    end
    
    initial_joint_state.time = Types::Base::Time.now
    interpolator.trajectory_target.write(target)
    interpolator.joint_state.write(initial_joint_state)

    #arr.each do |e|
    #   dispatcher.body_command.write(e)
    #   sleep(1)   
    #end


    Orocos.log_all

    puts "Press ENTER to stop!"
    STDIN.readline
end
