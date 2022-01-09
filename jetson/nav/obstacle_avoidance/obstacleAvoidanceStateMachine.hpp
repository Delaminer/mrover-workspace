#ifndef OBSTACLE_AVOIDANCE_STATE_MACHINE_HPP
#define OBSTACLE_AVOIDANCE_STATE_MACHINE_HPP

#include "rover.hpp"

class StateMachine;

// This class is the representation of different 
// obstacle avoidance algorithms
enum class ObstacleAvoidanceAlgorithm
{
    SimpleAvoidance,
    DoubleSidedAvoidance
};

// This class is the base class for the logic of the obstacle avoidance state machine 
class ObstacleAvoidanceStateMachine 
{
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    ObstacleAvoidanceStateMachine( StateMachine* stateMachine_, Rover* rover, const rapidjson::Document& roverConfig );

    virtual ~ObstacleAvoidanceStateMachine() {}

    void updateObstacleAngle( double leftBearing, double rightBearing );

    void updateObstacleDistance( double distance );

    void updateObstacleElements( double leftBearing, double rightBearing, double distance );  

    NavState run();

    bool isTargetDetected();

    virtual Odometry createAvoidancePoint( Rover* rover, const double distance ) = 0;

    virtual NavState executeTurnAroundObs( Rover* rover, const rapidjson::Document& roverConfig ) = 0;


    virtual NavState executeDriveAroundObs( Rover* rover, const rapidjson::Document& roverConfig ) = 0;


protected:
    /*************************************************************************/
    /* Protected Member Variables */
    /*************************************************************************/
    
    // Pointer to rover State Machine to access member functions
    StateMachine* roverStateMachine;

    // Odometry point used when avoiding obstacles.
    Odometry mObstacleAvoidancePoint;

    // // Initial angle to go around obstacle upon detection.
    double mOriginalObstacleAngle;

    // Initial angle of obstacle's left side
    double mOriginalObstacleLeftAngle;

    // Initial angle of obstacle's right side
    double mOriginalObstacleRightAngle;

    // Initial angle to go around obstacle upon detection.
    double mOriginalObstacleDistance;

    // bool for consecutive obstacle detections
    bool mJustDetectedObstacle;

    // Last obstacle angle for consecutive angles
    double mLastObstacleAngle;

    // Last obstacle left angle for consecutive angles
    double mLastObstacleLeftAngle;
    // Last obstacle right angle for consecutive angles
    double mLastObstacleRightAngle;

    // Pointer to rover object
    Rover* mRover;

private:
    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/

    // Reference to config variables
    const rapidjson::Document& mRoverConfig;

};

// Creates an ObstacleAvoidanceStateMachine object based on the inputted obstacle 
// avoidance algorithm. This allows for an an ease of transition between obstacle 
// avoidance algorithms
ObstacleAvoidanceStateMachine* ObstacleAvoiderFactory( StateMachine* roverStateMachine,
                                                       ObstacleAvoidanceAlgorithm algorithm, Rover* rover, const rapidjson::Document& roverConfig );

#endif //OBSTACLE_AVOIDANCE_STATE_MACHINE_HPP
