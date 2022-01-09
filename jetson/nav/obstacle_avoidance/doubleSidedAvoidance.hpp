#ifndef DOUBLE_SIDED_AVOIDANCE_HPP
#define DOUBLE_SIDED_AVOIDANCE_HPP

#include "obstacleAvoidanceStateMachine.hpp"

// Create an avoidance system for two sided detection
class DoubleSidedAvoidance : public ObstacleAvoidanceStateMachine
{
public:
    DoubleSidedAvoidance( StateMachine* roverStateMachine, Rover* rover, const rapidjson::Document& roverConfig );

    ~DoubleSidedAvoidance();

    NavState executeTurnAroundObs( Rover* rover, const rapidjson::Document& roverConfig );


    NavState executeDriveAroundObs( Rover* rover, const rapidjson::Document& roverConfig );


    Odometry createAvoidancePoint( Rover* rover, const double bearing, const double distance );
    Odometry createAvoidancePoint( Rover* rover, const double distance );

private:
    // Helper function: Determine which angle is closer to 0
    void DetermineSmallerAngle( double leftAngle, double rightAngle, bool &direction, double &smallerAngle);
};

#endif // DOUBLE_SIDED_AVOIDANCE_HPP