
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "utilities.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x, y};
}

int main()
{
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line))
	{
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	// some initial constants given the simulator
	double reference_velocity = 0.0;
	int lane = 1;
	double max_velocity = LIMITS::MAX_VELOCITY;

	h.onMessage([&reference_velocity, &lane, &max_velocity, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
																																			 uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{

			auto s = hasData(data);

			if (s != "")
			{
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry")
				{
					// j[1] is the data JSON object

					// Main car's localization Data
					double car_x = j[1][LOCALIZATION::X];
					double car_y = j[1][LOCALIZATION::Y];
					double car_s = j[1][LOCALIZATION::S];
					double car_d = j[1][LOCALIZATION::D];
					double car_yaw = j[1][LOCALIZATION::YAW];
					double car_speed = j[1][LOCALIZATION::SPEED];

					// Previous path data given to the Planner
					auto previous_path_x = j[1][LOCALIZATION::PREV_PATH_X];
					auto previous_path_y = j[1][LOCALIZATION::PREV_PATH_Y];
					// Previous path's end s and d values
					double end_path_s = j[1][LOCALIZATION::ENDING_S];
					double end_path_d = j[1][LOCALIZATION::ENDING_D];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1][LOCALIZATION::SENSOR_FUSION_DATA];

					int previous_size = previous_path_x.size();

					if (previous_size > 0)
					{
						car_s = end_path_s;
					}

					bool is_too_close = false;
					bool is_left_available = true;
					bool is_right_available = true;

					for (int i = 0; i < sensor_fusion.size(); ++i)
					{
						// get Frenet coordinates
						double s = sensor_fusion[i][SENSOR_FUSION::S];
						float d = sensor_fusion[i][SENSOR_FUSION::D];

						// get velocity
						double vx = sensor_fusion[i][SENSOR_FUSION::VX];
						double vy = sensor_fusion[i][SENSOR_FUSION::VY];

						double speed = euclideanDistance(vx, vy);
						double check_car_s = s + ((double)previous_size * 0.02 * speed);

						// if car is in our lane
						if (onSameLane(d, lane) && (check_car_s > car_s) && isNotSafe(car_s, check_car_s))
						{
							// max velocity in case of a car in front
							max_velocity = 30.0;
							is_too_close = true;
						}

						// prepare for lane change, check availabilities
						int left_lane = lane - 1;
						int right_lane = lane + 1;

						if (left_lane >= 0 && is_left_available && onSameLane(d, left_lane))
						{
							is_left_available = !isNotSafe(car_s, check_car_s);
						}

						if (right_lane <= 2 && is_right_available && onSameLane(d, right_lane))
						{
							is_right_available = !isNotSafe(car_s, check_car_s);
						}
					}
					
					cout << "max vel " << max_velocity << endl;

					if (is_too_close)
					{
						// slow down only if it's above of the established max limit
						if (reference_velocity > max_velocity) {
							reference_velocity -= LIMITS::BRAKE;
						}

						if (lane > 0 && is_left_available)
						{
							lane -= 1;
							max_velocity = LIMITS::MAX_VELOCITY;
						}
						else if (lane < 2 && is_right_available)
						{
							lane += 1;
							max_velocity = LIMITS::MAX_VELOCITY;
						}
					}
					else if (reference_velocity < max_velocity)
					{
						reference_velocity += LIMITS::ACCELERATION;
					}
					else
					{
						max_velocity = LIMITS::MAX_VELOCITY;
					}

					// create a list of widely spaced (x, y) waypoints, evenly spaced at LIMITS::PREDICT_DISTANCE meters
					vector<double> points_x;
					vector<double> points_y;

					// reference values of the car current position
					double reference_x = car_x;
					double reference_y = car_y;
					double reference_yaw;

					if (previous_size < 2)
					{
						// if there's no previous points, just calculate two poinst with the car's tangent previous point
						double prev_car_x = car_x - cos(car_yaw);
						double prev_car_y = car_y - sin(car_yaw);
						reference_yaw = deg2rad(car_yaw);

						points_x.push_back(prev_car_x);
						points_x.push_back(car_x);

						points_y.push_back(prev_car_y);
						points_y.push_back(car_y);
					}
					else
					{
						// use the previous path's end point as starting reference for predictions
						reference_x = previous_path_x[previous_size - 1];
						reference_y = previous_path_y[previous_size - 1];

						double reference_x_prev = previous_path_x[previous_size - 2];
						double reference_y_prev = previous_path_y[previous_size - 2];
						reference_yaw = atan2(reference_y - reference_y_prev, reference_x - reference_x_prev);

						points_x.push_back(reference_x_prev);
						points_x.push_back(reference_x);

						points_y.push_back(reference_y_prev);
						points_y.push_back(reference_y);
					}

					// (we will predict over LIMITS::PREDICTION_HORIZON * LIMITS::PREDICT_DISTANCE meters)
					for(int i = 1; i <= LIMITS::PREDICTION_HORIZON; ++i) {
						vector<double> waypoint = getXY(car_s + i * LIMITS::PREDICT_DISTANCE, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
						points_x.push_back(waypoint[0]);
						points_y.push_back(waypoint[1]);
					}

					// put all the points on the original reference for the cubic interpolation (in this case the current car position)
					for (int i = 0; i < points_x.size(); ++i)
					{
						double shift_x = points_x[i] - reference_x;
						double shift_y = points_y[i] - reference_y;
						auto ans = changeCoordinates(shift_x, shift_y, -reference_yaw);
						points_x[i] = ans[0]; 
						points_y[i] = ans[1];
					}

					// cubic interpolation with the current points with the origin coordinates over the car
					tk::spline spline;
					spline.set_points(points_x, points_y);

					// we fullfill all the previous points calculated on the step before to avoid same calculations over the spline
					vector<double> next_x_vals(previous_path_x.begin(), previous_path_x.end());
					vector<double> next_y_vals(previous_path_y.begin(), previous_path_y.end());

					// calculate how to break up spline points so that we travel at our desired reference
					double target_x = LIMITS::PREDICT_DISTANCE;
					double target_y = spline(target_x);
					double target_distance = euclideanDistance(target_x, target_y);

					// this is used to calculate the equally separated points on the x coordinate
					double current_x = 0;

					// we always predict 50 points (included previous and predicted points)
					for (int i = 1; i <= LIMITS::PREDICT_POINTS - previous_path_x.size(); ++i)
					{

						double N = (target_distance / (0.02 * reference_velocity / 2.24));
						double x_point = current_x + (target_x) / N;
						double y_point = spline(x_point);
						
						// this is set for the next increment
						current_x = x_point;

						// we have to rotate back to the global coordinates for the simulator
						auto result = changeCoordinates(x_point, y_point, reference_yaw);
						// also we add the car position on the global coordinates predicted
						next_x_vals.push_back(result[0] + reference_x);
						next_y_vals.push_back(result[1] + reference_y);
					}

					json msgJson;

					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
					   size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
						   char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}