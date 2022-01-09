#include "doubleSidedAvoidance.hpp"

#include "stateMachine.hpp"
#include "utilities.hpp"

#include <iostream>
#include <cmath>

// Create an avoidance system for two sided detection

void DoubleSidedAvoidance::DetermineSmallerAngle(double leftAngle, double rightAngle, bool &direction, double &smallerAngle) {
    
    // Determine which side to turn to - the left or the right, 
    // depending on which side the obstacle is closest too.
    // true is right, false is left

    //Which direction do we turn, left (false) or right (true)
    direction = true;
    // How many degrees the closest turn is (keeps direction relative to straightforward)
    smallerAngle = 0;
    
    // There are three cases for the left and right angles:
    // Both bearings are to the left (negative): turn right, use right angle's
    // Both bearings are to the right (positive): turn left, use left angle's
    // Bearings are split (one positive, one negative): turn and use the smallest absolute value
    if (leftAngle > 0) {
        // Turn left
        direction = false; // left
        smallerAngle = leftAngle;
    }
    else if (rightAngle < 0) {
        // Turn right
        direction = true; // right
        smallerAngle = rightAngle;
    }
    else {
        // Turn to the angle closest to zero
        if (fabs(leftAngle) < fabs(rightAngle)) {
            // Left angle is smaller, turn there (left)
            direction = false; // left
            smallerAngle = leftAngle;
        }
        else {
            // Right angle is smaller, turn there (right)
            direction = true; // right
            smallerAngle = leftAngle;
        }
    }
}

// Constructs a DoubleSidedAvoidance object with the input roverStateMachine, rover, and roverConfig.
// DoubleSidedAvoidance is abstracted from ObstacleAvoidanceStateMachine object so it creates an
// ObstacleAvoidanceStateMachine object with the roverStateMachine, rover, and roverConfig. 
// The DoubleSidedAvoidance object will execute the logic for the avoiding using the data from the two edges of an object
DoubleSidedAvoidance::DoubleSidedAvoidance( StateMachine* roverStateMachine, Rover* rover, const rapidjson::Document& roverConfig )
    : ObstacleAvoidanceStateMachine( roverStateMachine, rover, roverConfig ) {}

// Destructs the DoubleSidedAvoidance object.
DoubleSidedAvoidance::~DoubleSidedAvoidance() {}

// Turn away from obstacle until it is no longer detected.
// If in search state and target is both detected and reachable, return NavState TurnToTarget.
// ASSUMPTION: There is no rock that is more than 8 meters (pathWidth * 2) in diameter
NavState DoubleSidedAvoidance::executeTurnAroundObs( Rover* rover,
                                                const rapidjson::Document& roverConfig )
{
    if( isTargetDetected () && isTargetReachable( rover, roverConfig ) )
    {
        return NavState::TurnToTarget;
    }

    if( !isObstacleDetected( rover ) )
    {
        // Determine which direction to turn (and corresponding angle)
        bool turnDirection;
        double turnHeading;
        DetermineSmallerAngle( mOriginalObstacleLeftAngle, mOriginalObstacleRightAngle, turnDirection, turnHeading );
        double distanceAroundObs = mOriginalObstacleDistance /
                                   cos( fabs( degreeToRadian( turnHeading ) ) );
        mObstacleAvoidancePoint = createAvoidancePoint( rover, turnHeading, distanceAroundObs );
        if( rover->roverStatus().currentState() == NavState::TurnAroundObs )
        {
            return NavState::DriveAroundObs;
        }
        mJustDetectedObstacle = false;
        return NavState::SearchDriveAroundObs;
    }
    // An obstacle was detected

    // Get the bearing of the obstacle
    double obstacleLeftBearing = rover->roverStatus().obstacle().bearing;
    double obstacleRightBearing = rover->roverStatus().obstacle().rightBearing;
    cout << "Received bearings (" << obstacleLeftBearing << ", " << obstacleRightBearing << ")" << endl;
    // Determine which smaller angle to use
    bool direction; //left=false, right=true
    double bearing;
    DetermineSmallerAngle(obstacleLeftBearing, obstacleRightBearing, direction, bearing);
    
    // Test:
    direction = false;
    bearing = obstacleLeftBearing;
    double oldBearing = direction ? mLastObstacleRightAngle : mLastObstacleLeftAngle;



    // If an obstacle was detected last frame and it moved from one side of the rover to the other, 
    // restore the direction of the angle to the original
    if( mJustDetectedObstacle &&
        ( bearing < 0 ? oldBearing >= 0 : oldBearing < 0 ) ) {
        bearing *= -1;
    }

    // The bearing we want to have is our current bearing plus the bearing of the obstacle
    double desiredBearing = mod( rover->roverStatus().odometry().bearing_deg + bearing, 360 );
    // Update "previous frame" variables
    mJustDetectedObstacle = true;
    mLastObstacleLeftAngle = obstacleLeftBearing;
    mLastObstacleRightAngle = obstacleRightBearing;

    rover->turn( desiredBearing );
    return rover->roverStatus().currentState();
} // executeTurnAroundObs()

// Drives to dummy waypoint. Once arrived, rover will drive to original waypoint
// ( original waypoint is the waypoint before obstacle avoidance was triggered )
NavState DoubleSidedAvoidance::executeDriveAroundObs( Rover* rover, const rapidjson::Document& roverConfig )
{
    if( isObstacleDetected( rover )  && isObstacleInThreshold( rover, roverConfig ) )

    {
        if( rover->roverStatus().currentState() == NavState::DriveAroundObs )
        {
            return NavState::TurnAroundObs;
        }
        return NavState::SearchTurnAroundObs;
    }

    DriveStatus driveStatus = rover->drive( mObstacleAvoidancePoint );
    if( driveStatus == DriveStatus::Arrived )
    {
        if( rover->roverStatus().currentState() == NavState::DriveAroundObs )
        {
            return NavState::Turn;
        }
        return NavState::SearchTurn;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return rover->roverStatus().currentState();
    }
    if( rover->roverStatus().currentState() == NavState::DriveAroundObs )
    {
        return NavState::TurnAroundObs;
    }
    return NavState::SearchTurnAroundObs;
} // executeDriveAroundObs()

// Create the odometry point used to drive around an obstacle
Odometry DoubleSidedAvoidance::createAvoidancePoint( Rover* rover, 
                                                const double bearing, const double distance )
{
    Odometry avoidancePoint = rover->roverStatus().odometry();
    double totalLatitudeMinutes = avoidancePoint.latitude_min +
        cos( degreeToRadian( avoidancePoint.bearing_deg + bearing ) ) * distance * LAT_METER_IN_MINUTES;
    double totalLongitudeMinutes = avoidancePoint.longitude_min +
        sin( degreeToRadian( avoidancePoint.bearing_deg + bearing ) ) * distance * rover->longMeterInMinutes();
    avoidancePoint.latitude_deg += totalLatitudeMinutes / 60;
    avoidancePoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes) / 60 ) * 60 );
    avoidancePoint.longitude_deg += totalLongitudeMinutes / 60;
    avoidancePoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

    return avoidancePoint;

} // createAvoidancePoint()


// Create the odometry point used to drive around an obstacle
Odometry DoubleSidedAvoidance::createAvoidancePoint( Rover* rover, const double distance )
{
    cout << "Bad use of CreateAvoidancePoint in DoubleSidedAvoidance! Bearing is needed!" << endl;
    Odometry avoidancePoint = rover->roverStatus().odometry();
    double totalLatitudeMinutes = avoidancePoint.latitude_min +
        cos( degreeToRadian( avoidancePoint.bearing_deg ) ) * distance * LAT_METER_IN_MINUTES;
    double totalLongitudeMinutes = avoidancePoint.longitude_min +
        sin( degreeToRadian( avoidancePoint.bearing_deg ) ) * distance * rover->longMeterInMinutes();
    avoidancePoint.latitude_deg += totalLatitudeMinutes / 60;
    avoidancePoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes) / 60 ) * 60 );
    avoidancePoint.longitude_deg += totalLongitudeMinutes / 60;
    avoidancePoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

    return avoidancePoint;

} // createAvoidancePoint()
