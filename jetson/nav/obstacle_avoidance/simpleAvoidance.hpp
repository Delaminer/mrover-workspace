#ifndef SIMPLE_AVOIDANCE_HPP
#define SIMPLE_AVOIDANCE_HPP

#include "obstacleAvoidanceStateMachine.hpp"

// This class implements the logic for the simple obstacle avoidance algorithm.
// If an obstacle is seen, create an avoidance point using trigonometry with the angle turned and
// distance from obstacle.
class SimpleAvoidance : public ObstacleAvoidanceStateMachine
{
public:
    SimpleAvoidance( StateMachine* roverStateMachine, Rover* rover, const rapidjson::Document& roverConfig, 
                            double pathWidth_in, double bearingPathWidth_in );

    ~SimpleAvoidance();

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

#endif //SIMPLE_AVOIDANCE_HPP
