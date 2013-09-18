require 'pocolog'
include Pocolog
require 'pry'

def write_joints_stream(stream, out_stream)
  first_sample = true
  stream.samples.each do |realtime, logical, sample|
    if first_sample
      out_stream << "# This files contains joint state data for the following joints:\n"
      out_stream << "# "
      sample.names.each do |name|
        out_stream << "'" << name << "' "
      end
      out_stream << "\n"
      out_stream << "#\n"
      out_stream << "# For each joint four data filed where logged: 'position', 'speed', 'effort', 'raw'.\n"
      out_stream << "# The data below is ordered as follows:\n"
      out_stream << "#    * One line per sample\n"
      out_stream << "#    * Per sample:\n"
      out_stream << "#        time_in_seconds joint1 joint2 joint3\n"
      out_stream << "#    * Per joint:\n"
      out_stream << "#        position speed effort raw\n"
      out_stream << "\n"      
      
      first_sample = false
    end

    line = ""   
    line += logical.to_f.to_s + "    "
    sample.elements.each do |el|
      line += "#{el.position} #{el.speed} #{el.effort} #{el.raw}    "
    end
    out_stream << line << "\n"
  end
end

stream_cmd = "/orogen_default_trajectory_generation__Task.cmd"
stream_state = "/orogen_default_trajectory_generation__TestPlant.joint_state"
logfile = 'joint_state.log'

file = Logfiles.new File.open(logfile)

############################################

data_stream = file.stream(stream_cmd)
ofile = File.open("cmd.csv", 'w')

puts "Extracting command stream... "
write_joints_stream(data_stream, ofile)
puts "... done"
ofile.close

#############################################

data_stream = file.stream(stream_state)
ofile = File.open("state.csv", 'w')

puts "Extracting state stream... "
write_joints_stream(data_stream, ofile)
puts "... done"
ofile.close

############################################

file.close
