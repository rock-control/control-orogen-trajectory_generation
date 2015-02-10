require 'rock/bundle'
require 'pry'
require 'orocos/async'

Orocos.initialize
Orocos.conf.load_dir('.')

def range (min, max)
    rand() * (max-min) + min
end

joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5", "arm_joint_6"]

points = [[0.0001339584087047365, -0.00010074474741910817, -0.00030223422600267916, -0.0, 0.0007317373833439964, -0.0],
[0.04362281915058206, 0.014474166756500942, 0.05982011534124712, 0.0675441621750215, 0.006938049750045283, -0.07284134726528091],
[0.08525684923322316, 0.02612895957778144, 0.11672195663963716, 0.12953393350031014, 0.010736108427606764, -0.13969269776838256],
[0.12318121799739176, 0.031943515033782836, 0.16718278140030765, 0.18041492312613305, 0.00971765972688864, -0.19456405474712568],
[0.15554109478385167, 0.028997714441865583, 0.20798208135439886, 0.2146327402027574, 0.0014744499587511012, -0.231465421439331],
[0.18048164893336666, 0.014371439119390126, 0.23589934823305098, 0.2266329938804503, -0.016401774565945652, -0.24440680108281923],
[0.20086527184688965, -0.011360330568513747, 0.2547820426212476, 0.22041432630059382, -0.043860939048883954, -0.23770043135188804],
[0.21955435492537367, -0.047622614256716246, 0.26847762510397216, 0.19997537960456996, -0.08085296869174614, -0.21565854992083505],
[0.2394112895697717, -0.09384043158008759, 0.2808335562662083, 0.1693147959337608, -0.12732778869621456, -0.18259339446395786],
[0.26329846718103667, -0.14943880217349798, 0.29569729669293954, 0.13243121742954844, -0.18323532426397154, -0.14281720265555414],
[0.2902083270174518, -0.21207258771943768, 0.312441894068042, 0.09088039280028365, -0.24621739650419494, -0.09800773358350548],
[0.31913330833730047, -0.2793966499003969, 0.3304403960753921, 0.04621807075431723, -0.31391582652606265, -0.049842746335693563],
[0.3490658503988659, -0.3490658503988659, 0.3490658503988659, 0.0, -0.3839724354387525, 0.0]]

Orocos.run "trajectory_generation::Task" => "interpolator" do

    interpolator = Orocos::TaskContext.get "interpolator"
    Orocos.conf.apply(interpolator, ["default", "sherpa_manipulator"])
    interpolator.configure
    interpolator.start
  
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
    
    initial_joint_state.time = Types::Base::Time.now
    interpolator.trajectory_target.write(target)
    interpolator.joint_state.write(initial_joint_state)

    puts "Press ENTER to stop!"
    STDIN.readline
end
