/**
 * Hey there! Welcome to the Snowbots Software Challenge!
 *
 * As a prospective team member, we'd like to see you demonstrate your skills
 * by programming our brand spankin' new Mars rover.
 * The goal is to make the rover drive to a set of waypoints, read in from a
 * text file. We'll provide a lot of the rough framework you'll need, including
 * a _swanky_ simulator, we just need you to fill in a couple of functions 
 * below..........
 *
 */

#include <iostream>
#include <stack>
#include <vector>
#include <cmath>
#include <math.h>
#include <fstream>

/** Constants **/
/** (you probably don't need to care about these, and should *NOT* change them) **/
// Length of the robot (used for kinematic model in simulator)
static const double ROBOT_LENGTH = 0.2;
// The time step that the simulator takes on each iteration
static const double SIM_TIME_STEP = 0.05;
// The tolerance we want to come within for each waypoint
static const double WAYPOINT_TOLERANCE = 0.2;

/** You CAN change these constants **/
// The name of the file to read the waypoints in from
static const char* WAYPOINTS_FILE_NAME = "waypoints.txt";
// The maximum number of iterations the simulator will run (presuming 
// "waypoints" is never empty)
static const double MAX_SIM_ITERS = 10;


// A 2D Point In Space (measured in meters)
struct Point {
    double x;
    double y;
};

// A message giving the status of the robot
struct StatusMsg {
    // The current position of the robot
    Point position;

    // The current heading of the robot
    double heading;
};

// A command we can send to the robot to control it
struct MovementMsg {
    // An angle in radians, within [-pi/3, pi/3], with -pi/3 being turn right, 
    // pi/3 being turn left
    double steering_angle;

    // How fast the robot should move, in [-1, 1] with -1 being reverse,
    // and 1 being forward. The value is in meters/second.
    double throttle;
};

// Test Macro
#define TEST(COND) \
    if (!( COND )) { \
        std::cout << "WARNING: " << #COND << " is FALSE" << std::endl; \
    }

// Pre-declarations for functions
double distanceBetweenPoints(Point p1, Point p2);
double angleBetweenPoints(Point p1, Point p2);
MovementMsg computeNewMovement(StatusMsg status);
void runTests();

// Waypoints to navigate to
std::stack<Point> waypoints;

int main(int argc, char** argv) {
    /** RUN TESTS **/
    runTests();

    /** SETUP THE WAYPOINTS **/

    // Read the waypoints in from a file
    std::ifstream infile(WAYPOINTS_FILE_NAME);
    double x, y;
    while (infile >> x >> y){
        waypoints.push({x,y});
    }

    // Flip the waypoints stack, since we want the first waypoint at the top
    auto waypoints_copy = waypoints;
    waypoints = {};
    while (!waypoints_copy.empty()) {
        Point waypoint = waypoints_copy.top();
        waypoints_copy.pop();
        waypoints.push(waypoint);
    }

    /** RUN THE SIMULATOR **/

    // This is our crude "simulator". You don't need to touch this, though
    // it might help to at least somewhat understand what we're doing.
    StatusMsg curr_robot_status{ (Point){0,0}, 0 };
    int curr_sim_iter = 0;

    std::cout << "x, y, heading" << std::endl << "-------------" << std::endl;
    while (!waypoints.empty() && curr_sim_iter < MAX_SIM_ITERS) {
        curr_sim_iter++;

        // Print Robot State
        std::cout << curr_robot_status.position.x << ", "
            << curr_robot_status.position.y << ", " 
            << curr_robot_status.heading << std::endl;

        // Update Robot State
        MovementMsg mv_msg = computeNewMovement(curr_robot_status);

        // You don't need to understand this, but we're using a simple
        // "tricycle" ackermann model here to update the robot state
        double d_theta = (mv_msg.throttle / ROBOT_LENGTH) * 
            std::tan(mv_msg.steering_angle);
        double d_x = mv_msg.throttle * 
            std::cos(curr_robot_status.heading + (d_theta * SIM_TIME_STEP)/2);
        double d_y = mv_msg.throttle * 
            std::sin(curr_robot_status.heading + (d_theta * SIM_TIME_STEP)/2);
        
        curr_robot_status.position.x += d_x * SIM_TIME_STEP;
        curr_robot_status.position.y += d_y * SIM_TIME_STEP;
        curr_robot_status.heading += d_theta * SIM_TIME_STEP;
    }
}

/* ------- YOUR CODE GOES BELOW HERE ------- */

void runTests() {
    // HINT: Add tests for your functions here!!
    
    TEST(distanceBetweenPoints({0,0}, {10, 0}) == 10);

    TEST(angleBetweenPoints({0,0}, {10, 10}) > M_PI/4-0.0001);
    TEST(angleBetweenPoints({0,0}, {10, 10}) < M_PI/4+0.0001);

    std::cout << "-------------" << std::endl;
}

/**
 * Calculates the distance between two given points
 * 
 * @param p1 the first point
 * @param p2 the second point
 * 
 * @return the absolute distance between p1 and p2
 */
double distanceBetweenPoints(Point p1, Point p2) {
    // TODO #1
}

/**
 * Calculates the angle between the two given points
 * 
 * The angle is measured assuming +X is 0 degrees, +Y is +90 degrees
 *
 * @param p1 the first point
 * @param p2 the second point
 *
 * @return the angle between between p1 and p2
 */
double angleBetweenPoints(Point p1, Point p2) {
    // TODO #2
}

/** 
 * Calculate a new MovementMsg based on the given StatusMsg
 *
 * This MovementMsg should move the Robot towards the next waypoint.
 * If we're close enough (ie. within "WAYPOINT_TOLERANCE") to the waypoint, 
 * this will update "waypoints" so that we start moving towards the next one.
 *
 * @param status The current status of the robot
 *
 * @return A MovementMsg moving the robot towards the next waypoint
 */
MovementMsg computeNewMovement(StatusMsg status) {
    // TODO #3
    // NOTE: The "simulator" will stop once "waypoints" is empty
    // HINT: You are allowed to change the waypoints file for testing (by
    //       modifying "WAYPOINTS_FILE_NAME" at the top of this file) (but we
    //       will be testing with our own secret waypoints!)
    // HINT: You probably want to use the functions you filled in above....
    // HINT: If you want to visualize what your robot is doing, you probably
    //       want to take the points that the simulator prints and put them 
    //       into a scatter plot in excel (or equivalent). You may also want
    //       to change "MAX_SIM_ITERS" at the top of this file to limit the 
    //       number of simulator iterations at first
    return MovementMsg({3.14159/4,1});
}
