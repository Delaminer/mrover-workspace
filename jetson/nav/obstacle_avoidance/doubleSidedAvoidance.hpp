#ifndef DOUBLE_SIDED_AVOIDANCE_HPP
#define DOUBLE_SIDED_AVOIDANCE_HPP

#include "obstacleAvoidanceStateMachine.hpp"

// Create an avoidance system for two sided detection
class DoubleSidedAvoidance : public ObstacleAvoidanceStateMachine
{
public:
    DoubleSidedAvoidance( StateMachine* roverStateMachine, Rover* rover, const rapidjson::Document& roverConfig, 
                            double pathWidth_in, double bearingPathWidth_in );

    ~DoubleSidedAvoidance();

    NavState executeTurnAroundObs( Rover* rover, const rapidjson::Document& roverConfig );


    NavState executeDriveAroundObs( Rover* rover, const rapidjson::Document& roverConfig );


    Odometry createAvoidancePoint( Rover* rover, const double distance );

private:
    // Helper function: Determine which angle is closer to 0
    void DetermineSmallerAngle( double leftAngle, double rightAngle, bool &direction, double &smallerAngle);
    // The width of the rover's path, for determining if it can drive past an obstacle
    double pathWidth;
    // The width of the rover's path, in degrees. This determines the change in bearing in which
    // the rover can drive through (if they detect a hole, we know if we can fit through)
    double bearingPathWidth;
};

#endif // DOUBLE_SIDED_AVOIDANCE_HPP