require 'rock/bundle'
require 'pry'
require 'orocos/async'

Orocos.initialize
Orocos.conf.load_dir('.')

def range (min, max)
    rand() * (max-min) + min
end

joint_names = ["J_Elbow_r", "J_Forearm_r", "J_Shoulder1_r", "J_Shoulder2_r", "J_UpperArm_r", "J_Wrist1_r", "J_Wrist2_r"]

points = [[0.00012504 , 0.000270657 , -4.07121e-05 , -0.000110363 , -4.27669e-05 , -0.000117393 , -0.000119131],
[0.0760103 , -0.227874 , 0.102925 , 0.0350644 , 0.193709 , -0.0188065 , 0.0192143],
[0.142408 , -0.427505 , 0.19302 , 0.065844 , 0.363243 , -0.0351577 , 0.0361328],
[0.18983 , -0.570109 , 0.257375 , 0.0878334 , 0.48434 , -0.0468328 , 0.0482218],
[0.20879 , -0.62717 , 0.28312 , 0.0966375 , 0.532782 , -0.0514941 , 0.0530663],
[0.189799 , -0.570176 , 0.257385 , 0.087861 , 0.484351 , -0.0468035 , 0.0482516],
[0.142345 , -0.427641 , 0.19304 , 0.0658992 , 0.363264 , -0.035099 , 0.0361924],
[0.0759165 , -0.228077 , 0.102955 , 0.0351472 , 0.193741 , -0.0187185 , 0.0193036],
[0 , 0 , 0 , 0 , 0 , 0 , 0 , ]]

Orocos.run "trajectory_generation::Task" => "interpolator" do

    interpolator = Orocos::TaskContext.get "aila_interpolation_position"
    #Orocos.conf.apply(interpolator, ["default", "trajectory_following"])
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
   
    points_inv.each do |joint_traj|
        traj = Types::Base::JointTrajectory.new
        joint_traj.each do |point|
           state = Types::Base::JointState.new
           state.position = point
           traj << state
        end
        target.elements	 << traj
    end
    
    initial_joint_state.time = Types::Base::Time.now
    interpolator.trajectory_target.write(target)
    interpolator.joint_state.write(initial_joint_state)

    puts "Press ENTER to stop!"
    STDIN.readline
end
