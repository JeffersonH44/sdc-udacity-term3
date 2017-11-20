#ifndef UTILITIES
#define UTILITIES

#include <string>
#include <cmath>
#include <vector>

using namespace std;

// utility constants

enum SENSOR_FUSION {
    S = 5,
    D = 6,
    VX = 3,
    VY = 4
};

namespace LOCALIZATION {
    const string X = "x";
    const string Y = "y";
    const string S = "s";
    const string D = "d";
    const string YAW = "yaw";
    const string SPEED = "speed";
    const string PREV_PATH_X = "previous_path_x";
    const string PREV_PATH_Y = "previous_path_y";
    const string ENDING_S = "end_path_s";
    const string ENDING_D = "end_path_d";
    const string SENSOR_FUSION_DATA = "sensor_fusion";
};

// parameters for the model 
namespace LIMITS {
    const double MAX_VELOCITY = 49.5;
    const double ACCELERATION = 0.2;
    const double BRAKE = 0.15;
    const double SAFE_DISTANCE = 35.0;
    const double PREDICT_DISTANCE = 25.0;

    const int PREDICT_POINTS = 50;
    const int PREDICTION_HORIZON = 3;
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

bool isNotSafe(double s, double other_s) {
    return abs(s - other_s) < LIMITS::SAFE_DISTANCE;
}

vector<double> changeCoordinates(double x, double y, double yaw) {
    return {
        x * cos(yaw) - y * sin(yaw),
        x * sin(yaw) + y * cos(yaw)
    };
}

#endif