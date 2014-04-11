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
    bool makeFeasible(){
        bool all_feasible = true;

        for(size_t joint_idx=0; joint_idx<elements.size(); joint_idx++){
            for(size_t time_idx=0; time_idx<getTimeSteps(); time_idx++){
                base::JointState& sample = elements[joint_idx][time_idx];
                JointMotionConstraints& constraint = motion_constraints[joint_idx][time_idx];

                //Check position and make feasible
                if(constraint.max.hasPosition() && sample.position > constraint.max.position){
                    sample.position = constraint.max.position;
                    all_feasible = false;
                }
                if(constraint.min.hasPosition() && sample.position < constraint.min.position){
                    sample.position = constraint.min.position;
                    all_feasible = false;
                }

                //Check speed and make feasible
                if(constraint.max.hasSpeed() && sample.speed > constraint.max.speed){
                    sample.speed = constraint.max.speed;
                    all_feasible = false;
                }
                /*
                //Constriants for speed and effort might be defined only by its max value which value is
                //then valid for both, positive and megative direction.
                if(constraint.min.hasSpeed()){
                    if(sample.speed < constraint.min.speed){
                        sample.speed = constraint.min.speed;
                        all_feasible = false;
                    }
                }
                else{
                    if(constraint.max.hasSpeed() && sample.speed < -constraint.max.speed){
                        sample.speed = -constraint.max.speed;
                        all_feasible = false;
                    }
                }
                */

                //Check effort and make feasible
                if(constraint.max.hasEffort() && sample.effort > constraint.max.effort){
                    sample.effort = constraint.max.effort;
                    all_feasible = false;
                }
                /*
                //Constriants for speed and effort might be defined only by its max value which value is
                //then valid for both, positive and megative direction.
                if(constraint.min.hasEffort()){
                    if(sample.effort < constraint.min.effort){
                        sample.effort = constraint.min.effort;
                        all_feasible = false;
                    }
                }
                else{
                    if(constraint.max.hasEffort() && sample.effort < -constraint.max.effort){
                        sample.effort = -constraint.max.effort;
                        all_feasible = false;
                    }
                }
                */

                //Check raw and make feasible
                if(constraint.max.hasRaw() && sample.raw > constraint.max.raw){
                    sample.raw = constraint.max.raw;
                    all_feasible = false;
                }
                /*
                //Constriants for speed and effort might be defined only by its max value which value is
                //then valid for both, positive and megative direction.
                if(constraint.min.hasRaw()){
                    if(sample.raw < constraint.min.raw){
                        sample.raw = constraint.min.raw;
                        all_feasible = false;
                    }
                }
                else{
                    if(constraint.max.hasRaw() && sample.raw < -constraint.max.raw){
                        sample.raw = -constraint.max.raw;
                        all_feasible = false;
                    }
                }
                */
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

