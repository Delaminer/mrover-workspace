#include "obstacleAvoidanceStateMachine.hpp"

#include "utilities.hpp"
#include "stateMachine.hpp"
#include "simpleAvoidance.hpp"
#include "doubleSidedAvoidance.hpp"
#include <cmath>
#include <iostream>

// Constructs an ObstacleAvoidanceStateMachine object with roverStateMachine, mRoverConfig, and mRover
ObstacleAvoidanceStateMachine::ObstacleAvoidanceStateMachine( StateMachine* stateMachine_, Rover* rover, const rapidjson::Document& roverConfig )
    : roverStateMachine( stateMachine_ )
    , mJustDetectedObstacle( false )
    , mRover( rover ) 
    , mRoverConfig( roverConfig ) {}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void ObstacleAvoidanceStateMachine::updateObstacleAngle( double leftBearing, double rightBearing )
{
    // Some code still uses only one angle, so still update it
    mOriginalObstacleAngle = leftBearing;

    //Update each sides of the angle
    mOriginalObstacleLeftAngle = leftBearing;
    mOriginalObstacleRightAngle = rightBearing;
}

// Allows outside objects to set the original obstacle distance
// This will allow the variable to be set before the rover turns
void ObstacleAvoidanceStateMachine::updateObstacleDistance( double distance )
{
    mOriginalObstacleDistance = distance;
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void ObstacleAvoidanceStateMachine::updateObstacleElements( double leftBearing, 
                                                        double rightBearing, double distance )
{
    updateObstacleAngle( leftBearing, rightBearing );
    updateObstacleDistance( distance );
}

// Runs the avoidance state machine through one iteration. This will be called by StateMachine
// when NavState is in an obstacle avoidance state. This will call the corresponding function based
// on the current state and return the next NavState
NavState ObstacleAvoidanceStateMachine::run()
{
    switch ( mRover->roverStatus().currentState() )
    {
        case NavState::TurnAroundObs:
        case NavState::SearchTurnAroundObs:
        {
            return executeTurnAroundObs( mRover, mRoverConfig );
        }

        case NavState::DriveAroundObs:
        case NavState::SearchDriveAroundObs:
        {

            return executeDriveAroundObs( mRover, mRoverConfig );

        }

        default:
        {
            cerr << "Entered unknown NavState in obstacleAvoidanceStateMachine" << endl;
            return NavState::Unknown;
        }
    } // switch
}

// Checks that both rover is in search state and that target is detected
bool ObstacleAvoidanceStateMachine::isTargetDetected ()
{
    return ( mRover->roverStatus().currentState() == NavState::SearchTurnAroundObs &&
             mRover->roverStatus().target().distance >= 0 );
}

// The obstacle avoidance factory allows for the creation of obstacle avoidance objects and
// an ease of transition between obstacle avoidance algorithms
ObstacleAvoidanceStateMachine* ObstacleAvoiderFactory ( StateMachine* roverStateMachine,
                                                        ObstacleAvoidanceAlgorithm algorithm, Rover* rover, const rapidjson::Document& roverConfig )
{
    ObstacleAvoidanceStateMachine* avoid = nullptr;
    switch ( algorithm )
    {
        case ObstacleAvoidanceAlgorithm::SimpleAvoidance:
        case ObstacleAvoidanceAlgorithm::DoubleSidedAvoidance: { //curly brackets are needed for variable initialization
            // Get path width from the config file
            double pathWidth = roverConfig["roverMeasurements"]["width"].GetDouble();
            // Calculate bearing version of width using math (law of cosines)
            double visionDistance = roverConfig["computerVision"]["visionDistance"].GetDouble();
            double bearingPathWidth = acos(1.0 - (pathWidth * pathWidth / (2 * visionDistance * visionDistance)));
            // Convert to degrees
            bearingPathWidth *= 180 / 3.141592;
            avoid = new SimpleAvoidance( roverStateMachine, rover, roverConfig, pathWidth, bearingPathWidth );
            break;
        }
        default:
            std::cerr << "Unkown Search Type. Defaulting to original\n";
            avoid = new SimpleAvoidance( roverStateMachine, rover, roverConfig, 0, 0 );
            break;
    } // switch
    return avoid;
} // ObstacleAvoiderFactory

