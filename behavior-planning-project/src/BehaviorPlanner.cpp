//
// Created by jefferson on 18.09.17.
//

#include "BehaviorPlanner.h"

BehaviorPlanner::BehaviorPlanner(int lanes, int startingLane, double laneWidth,  double maxVelocity, double safeDistance) :
        lanes(lanes), currentLane(startingLane), laneWidth(laneWidth), halfLaneWidth(laneWidth * 0.5),
        maxVelocity(maxVelocity), safeDistance(safeDistance) {
}

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

double BehaviorPlanner::getVelocity(double vx, double vy) {
    return sqrt(vx * vx + vy * vy);
}

bool BehaviorPlanner::isSafeDistance(double carS, double otherS) {
    return abs(otherS - carS)  < safeDistance;
}

bool BehaviorPlanner::inCurrentLane(double carD, double otherD) {
    return otherD < carD + halfLaneWidth && carD - halfLaneWidth < otherD;
}
