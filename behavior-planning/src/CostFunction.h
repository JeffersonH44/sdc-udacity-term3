//
// Created by jefferson on 9/09/17.
//

#ifndef SDC_UDACITY_TERM3_COSTFUNCTION_H
#define SDC_UDACITY_TERM3_COSTFUNCTION_H

#include "Vehicle.h"
#include <map>
#include <vector>

struct dataHelper {
    int proposedLine;
    double avgSpeed;
    double maxAccel;
    double RMSAcceleration;
    int closestApproach;
    int distanceToGoalS;
    int distanceToGoalLine;
    int collides;
};

namespace CostFunction {
    const int PREDICTION_LANE = 0;
    const int PREDICTION_S = 1;
    const int PLANNING_HORIZON = 2;
    const int NOT_COLLISION = -1;

    dataHelper getHelperData(Vehicle vehicle, vector<Snapshot> trajectory, map<int, vector<vector<int> > > predictions);
    map<int, vector<vector<int> > > filterPredictions(map<int, vector<vector<int> > > predictions, int lane);
    bool checkCollision(Snapshot snapshot, int prevS, int currentS);
}


#endif //SDC_UDACITY_TERM3_COSTFUNCTION_H
