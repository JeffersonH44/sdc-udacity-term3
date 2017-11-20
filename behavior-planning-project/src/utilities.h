#ifndef UTILITIES
#define UTILITIES

#include <string>
#include <cmath>

using namespace std;

// utility constants

enum SENSOR_FUSION {
    S = 5,
    D = 6,
    VX = 3,
    VY = 4
};

namespace LOCALIZATION {
    static string X = "x";
    static string Y = "y";
    static string S = "s";
    static string D = "d";
    static string YAW = "yaw";
    static string SPEED = "speed";
    static string PREV_PATH_X = "previous_path_x";
    static string PREV_PATH_Y = "previous_path_y";
    static string ENDING_S = "end_path_s";
    static string ENDING_D = "end_path_d";
    static string SENSOR_FUSION_DATA = "sensor_fusion";
};

namespace LIMITS {
    double MAX_VELOCITY = 49.5;
    double ACCELERATION = 0.2;
    double SAFE_DISTANCE = 25.0;
}

// utility functions

double euclideanDistance(double x, double y) {
    return sqrt(x * x + y * y);
}

bool onSameLane(float car_d, int lane) {
    double low_lane_bound = (2 + 4 * lane - 2);
    double high_lane_bound = (2 + 4 * lane + 2);

    return low_lane_bound < car_d  && car_d < high_lane_bound;
}

bool isSafe(double s, double other_s) {
    return abs(s - other_s) < LIMITS::SAFE_DISTANCE;
}

#endif