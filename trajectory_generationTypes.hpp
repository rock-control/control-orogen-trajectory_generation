#ifndef trajectory_generation_TYPES_HPP
#define trajectory_generation_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <vector>

namespace trajectory_generation {
    struct RMLInputParams{   
         RMLInputParams(){}
         RMLInputParams(int nDOF){
              NumberOfDOFs = nDOF;
              CurrentPositionVector.resize(nDOF);
              CurrentVelocityVector.resize(nDOF);
              CurrentAccelerationVector.resize(nDOF);
              TargetPositionVector.resize(nDOF);
              TargetVelocityVector.resize(nDOF);
              MinPositionVector.resize(nDOF);
              MaxPositionVector.resize(nDOF);
              MaxVelocityVector.resize(nDOF);
              MaxAccelerationVector.resize(nDOF);
              MaxJerkVector.resize(nDOF);
              SelectionVector.resize(nDOF);
              MinimumSynchronizationTime = 0;
         }
         int NumberOfDOFs;
         std::vector<double> CurrentPositionVector;
         std::vector<double> CurrentVelocityVector;
         std::vector<double> CurrentAccelerationVector;
         std::vector<double> TargetVelocityVector;
         std::vector<double> TargetPositionVector;
         std::vector<double> MinPositionVector;
         std::vector<double> MaxPositionVector;
         std::vector<double> MaxVelocityVector;
         std::vector<double> MaxAccelerationVector;
         std::vector<double> MaxJerkVector;
         std::vector<int> SelectionVector;
         double MinimumSynchronizationTime;
    };

    struct RMLOutputParams{
         RMLOutputParams(){}
         RMLOutputParams(int nDOF){
              NumberOfDOFs = nDOF;
              ExecutionTimes.resize(nDOF);
              NewPositionVector.resize(nDOF);
              NewVelocityVector.resize(nDOF);
              NewAccelerationVector.resize(nDOF);
              PositionValuesAtTargetVelocity.resize(nDOF);
              ANewCalculationWasPerformed = 0;
              DOFWithTheGreatestExecutionTime = -1;
              SynchronizationTime = 0;
              TrajectoryIsPhaseSynchronized = 0;
         }   
         int NumberOfDOFs;
         int ANewCalculationWasPerformed;
         int DOFWithTheGreatestExecutionTime;
         std::vector<double> ExecutionTimes;
         double SynchronizationTime;
         int TrajectoryIsPhaseSynchronized;
         
         std::vector<double> NewPositionVector;
         std::vector<double> NewVelocityVector;
         std::vector<double> NewAccelerationVector;
         std::vector<double> PositionValuesAtTargetVelocity;
    };
}

#endif

