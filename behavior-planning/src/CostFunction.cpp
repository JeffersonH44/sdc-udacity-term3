//
// Created by jefferson on 9/09/17.
//

#include "CostFunction.h"
#include <algorithm>

bool absComparator(int i, int j) {
    return abs(i) < abs(j);
}

namespace CostFunction {

    bool checkCollision(Snapshot snapshot, int prevS, int currentS) {
        int s = snapshot.s;
        int v = snapshot.v;
        int targetV = currentS - prevS;
        if (prevS < s) // if predicted trajectory for other vehicle is faster
            if (currentS >= s)
                return true;
            else
                return false;

        if (prevS > s) // if predicted trajectory for other vehicle is slower
            if (currentS <= s)
                return true;
            else
                return false;

        if (prevS == s)
            if (targetV > v)
                return false;
            else
                return true;

        return true; // error case
    }

    map<int, vector<vector<int> > > filterPredictions(map<int, vector<vector<int> > > predictions, int lane) {
        map<int, vector<vector<int> > > filtered;
        auto it = predictions.begin();
        while(it != predictions.end()) {
            if(it->first != -1 && it->second[0][PREDICTION_LANE] == lane) {
                filtered[it->first] = it->second;
            }
            it++;
        }

        return filtered;
    }

    dataHelper
    getHelperData(Vehicle vehicle, vector<Snapshot> trajectory, map<int, vector<vector<int> > > predictions) {
        Snapshot current = trajectory[0], first = trajectory[1], last = trajectory[trajectory.size() - 1];
        dataHelper data;
        data.distanceToGoalS = vehicle.goal_s - last.s;
        data.distanceToGoalLine = abs(vehicle.goal_lane - last.lane);
        double dt = static_cast<double>(trajectory.size());
        data.proposedLine = first.lane;
        data.avgSpeed = (last.s - current.s) / dt;

        vector<int> accels;
        int closestApproach = 999999;
        bool collides = NOT_COLLISION;
        // last snapshot ???
        auto filtered = filterPredictions(predictions, data.proposedLine);
        for(int i = 1, j = 2; i < PLANNING_HORIZON + 1; i++, j++) {
            Snapshot currentSnapshot = trajectory[j];
            accels.push_back(currentSnapshot.a);

            for(auto it = filtered.begin(); it != filtered.end(); ++it) {
                auto state = it->second[i];
                auto lastState = it->second[i - 1];
                bool collision = checkCollision(currentSnapshot, lastState[PREDICTION_S], state[PREDICTION_S]);
                if(collision) {
                    collides = i;
                }
                int dist = abs(state[PREDICTION_S] - currentSnapshot.s);
                if(dist < closestApproach) {
                    closestApproach = dist;
                }
            }
        }

        data.maxAccel = *std::min_element(accels.begin(), accels.end(), absComparator);
        double sum = 0.0;
        for(double a : accels) {
            sum += a * a;
        }
        sum /= accels.size();
        data.RMSAcceleration = sum;
    }
}