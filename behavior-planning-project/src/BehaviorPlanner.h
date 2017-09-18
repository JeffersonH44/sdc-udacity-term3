//
// Created by jefferson on 18.09.17.
//

#ifndef SDC_UDACITY_TERM3_BEHAVIORPLANNER_H
#define SDC_UDACITY_TERM3_BEHAVIORPLANNER_H

#include <utility>
#include <cmath>

using namespace std;

/**
 * @class
 * Behavior planner that work only on the highway
 */
class BehaviorPlanner {
public:
    BehaviorPlanner(int lanes, int startingLane, double laneWidth, double maxVelocity, double safeDistance);
    pair<int, double> predict(double carS, double carSpeed, int pathSize, auto sensorFusion);
private:
    enum SensorFusion {
        ID = 0,
        X = 1,
        Y = 2,
        VX = 3,
        VY = 4,
        S = 5,
        D = 6
    };
    static double getVelocity(double vx, double vy);
    bool isSafeDistance(double carS, double otherS);
    bool inCurrentLane(double carD, double otherD);


    int lanes;
    int currentLane;
    double laneWidth;
    double halfLaneWidth;
    double maxVelocity;
    double safeDistance;
};


#endif //SDC_UDACITY_TERM3_BEHAVIORPLANNER_H
