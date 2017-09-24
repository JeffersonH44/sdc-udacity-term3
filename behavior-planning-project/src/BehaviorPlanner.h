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

pair<int, double> BehaviorPlanner::predict(double carS, double carSpeed, int pathSize, auto sensorFusion) {
    bool isTooClose = false;
    bool isLeftAvailable = true;
    bool isRightAvailable = true;

    for (int i = 0; i < sensorFusion.size(); ++i) {
        double otherVX = sensorFusion[i][SensorFusion::VX];
        double otherVY = sensorFusion[i][SensorFusion::VY];

        float otherD = sensorFusion[i][SensorFusion::D];
        double otherSpeed = getVelocity(otherVX, otherVY);
        double otherS = sensorFusion[i][SensorFusion::S];
        double predictedS = pathSize * 0.02 * otherSpeed; // predict future position of the car

        double carD = halfLaneWidth + laneWidth * currentLane;
        isTooClose = isSafeDistance(carS, otherS) && inCurrentLane(carD, otherD) && otherS > carS;


        int leftLane = currentLane - 1;
        int rightLane = currentLane + 1;
        double carLeftD = halfLaneWidth + laneWidth * leftLane;
        double carRightD = halfLaneWidth + laneWidth * rightLane;
        isLeftAvailable = leftLane >= 0 && isLeftAvailable && inCurrentLane(carLeftD, otherD) && isSafeDistance(carS, otherS);
        isRightAvailable = rightLane >= 0 && isLeftAvailable && inCurrentLane(carLeftD, otherD) && isSafeDistance(carS, otherS);
    }

    if(isTooClose) {
        carSpeed -= 0.25;
        if(isLeftAvailable) {
            currentLane -= 1;
        } else if(isRightAvailable) {
            currentLane += 1;
        }
    } else if(carSpeed < maxVelocity) {
        carSpeed += 0.25;
    }

    return make_pair(currentLane, carSpeed);
}

#endif //SDC_UDACITY_TERM3_BEHAVIORPLANNER_H
