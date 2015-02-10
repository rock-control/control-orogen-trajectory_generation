#ifndef BASE_CONSTRAINEDJOINTSTRAJECTORY_HPP
#define BASE_CONSTRAINEDJOINTSTRAJECTORY_HPP

#include <vector>
#include <base/JointsTrajectory.hpp>
#include <base/JointLimitRange.hpp>
#include <base/commands/Joints.hpp>
#include <base/Time.hpp>

namespace trajectory_generation {

struct JointMotionConstraints
        : public  base::JointLimitRange
{
    JointMotionConstraints(){
        max_jerk = base::unset<float>();
    }

    float max_jerk;
};

typedef base::NamedVector<JointMotionConstraints> JointsMotionConstraints;
typedef std::vector<JointMotionConstraints> JointMotionConstraintsSeries;

struct ConstrainedJointsCmd
        : public base::commands::Joints
{

    /**
     * @brief Array of motion constraints the motion of each joint underlies
     *
     * This vector needs to be the same dimensions as the base type. That is,
     * for every joint, a motion constraint is given.
     */
    JointsMotionConstraints motion_constraints;
};


struct ConstrainedJointsTrajectory 
        : public base::JointsTrajectory
{
    /**
     * @brief Array of motion constraints the motion to the corresponding sample
     *        of each joint underlies
     *
     * This vector needs to be the same dimensions as the trajectory. That is,
     * for every sample of every joint, a motion constriant is given.
     */
    std::vector<JointMotionConstraintsSeries> motion_constraints;
    
    /**
     * @brief Extracts the motion constraints for all joints at a given
     *        time step.
     *
     * @param motion_constraints extracted motion_constraints will be stored
     *        here
     *
     * @throws InvalidTimeStep if the given time_step does not exist in the
     *         trajectory.
     */
    void getJointsMotionConstraintsAtSample(size_t time_step,
                                            JointsMotionConstraints& motion_constraints){
        if(time_step > getTimeSteps()){
            throw(base::JointsTrajectory::InvalidTimeStep(time_step));
        }
        
        motion_constraints.resize(names.size());
        motion_constraints.names = names;
        
        for(size_t joint_index=0; joint_index<getNumberOfJoints(); joint_index++){
            motion_constraints.elements[joint_index] = this->motion_constraints[joint_index][time_step];
        }
    }

    /** @return true if the structure is valid
     */
    bool isValid() const
    {
        if(!JointsTrajectory::isValid())
            return false;

        if(motion_constraints.size() != elements.size())
            return false;

        if(motion_constraints[0].size() != elements[0].size())
            return false;

        return true;
    }

    /**
     * @brief Feasibility check of a constrained trajectory.
     *
     * A ContrainedTrajectory is feasible, if all via points
     * fullfill the given constraints.
     *
     * TODO: Currently only static limits are checked. For timed
     *       trajectories, the constraints could also be checked with respect
     *       to time stamps.
     *
     * @return true if traejctory is feasible
     */
    bool isFeasible() const
    {
        for(size_t joint_idx=0; joint_idx<elements.size(); joint_idx++){
            for(size_t time_idx=0; time_idx<getTimeSteps(); time_idx++){
                base::JointState sample = elements[joint_idx][time_idx];
                JointMotionConstraints constraint = motion_constraints[joint_idx][time_idx];
                if(!constraint.isValid(sample))
                    return false;
            }
        }
        return true;
    }

    /**
     * @brief Perform feasibility check of a constrained trajectory modify
     *        samples if necessary to make them feasible.
     *
     * see isFeasible()
     *
     * @return true if was feasible.
     * @return false if trajectory was modified.
     */
    bool makeFeasible(std::stringstream &err){
        bool all_feasible = true;

        err.str("");

        for(size_t joint_idx=0; joint_idx<elements.size(); joint_idx++){
            for(size_t time_idx=0; time_idx<getTimeSteps(); time_idx++){
                base::JointState& sample = elements[joint_idx][time_idx];
                JointMotionConstraints& constraint = motion_constraints[joint_idx][time_idx];

                //Check position and make feasible
                if(constraint.max.hasPosition() && sample.position > constraint.max.position){
                    err << "Joint " << joint_idx << " (" << names[joint_idx] << ") exceeds limits: Target position: "
                        << sample.position << ", Max position: " << constraint.max.position << std::endl;
                    sample.position = constraint.max.position - 1e-5; //Subtract small value here, to avoid RML throwing error msgs because of exceeding joint limits
                    sample.speed = sample.acceleration = 0; //Set speed and acc to zero, otherwise the trajectory might be driven into the limits anyways
                    all_feasible = false;
                }
                if(constraint.min.hasPosition() && sample.position < constraint.min.position){
                    err << "Joint " << joint_idx << " (" << names[joint_idx] << ") exceeds limits: Target position: "
                        << sample.position << ", Min position: " << constraint.min.position << std::endl;
                    sample.position = constraint.min.position + 1e-5; //Add small value here, to avoid RML throwing error msgs because of exceeding joint limits
                    sample.speed = sample.acceleration = 0; //Set speed and acc to zero, otherwise the trajectory might be driven into the limits anyways
                    all_feasible = false;
                }

                //Check speed and make feasible
                if(constraint.max.hasSpeed() && fabs(sample.speed) > constraint.max.speed){
                    sample.speed = constraint.max.speed;
                    err << "Joint " << joint_idx << " (" << names[joint_idx] << ") exceeds limits: Target speed: "
                        << sample.speed<< ", Max Speed: " << constraint.max.speed << std::endl;
                    all_feasible = false;
                }

                //Check acceleration and make feasible
                if(constraint.max.hasAcceleration() && fabs(sample.acceleration) > constraint.max.acceleration){
                    sample.acceleration = constraint.max.acceleration;
                    err << "Joint " << joint_idx << " (" << names[joint_idx] << ") exceeds limits: Target acceleration: "
                        << sample.acceleration<< ", Max acceleration: " << constraint.max.acceleration << std::endl;
                    all_feasible = false;
                }
            }
        }
        return all_feasible;
    }
    
    void resize(int num_joints, int num_samples){
        JointsTrajectory::resize(num_joints, num_samples);

        motion_constraints.resize(num_joints);
        for(size_t i=0; i<motion_constraints.size(); i++){
            motion_constraints[i].resize(num_samples);
        }
    }
    
    void resize(int num_joints){
        JointsTrajectory::resize(num_joints);
        motion_constraints.resize(num_joints);
    }
}; 

}

#endif

