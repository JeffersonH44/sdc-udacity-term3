//
// Created by jefferson on 18.09.17.
//

#include "BehaviorPlanner.h"

BehaviorPlanner::BehaviorPlanner(int lanes, int startingLane, double laneWidth,  double maxVelocity, double safeDistance) :
        lanes(lanes), currentLane(startingLane), laneWidth(laneWidth), halfLaneWidth(laneWidth * 0.5),
        maxVelocity(maxVelocity), safeDistance(safeDistance) {
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
