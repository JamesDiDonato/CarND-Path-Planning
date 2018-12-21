#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>
#include "spline.h"

using namespace std;

using namespace tk;
// for convenience
//using json = nlohmann::json;


enum State
{ 
	keep_lane = 0,
	prep_lane_change_right = 1,
	prep_lane_change_left = 2,
	lane_change_right = 3,
	lane_change_left = 4
};

enum Lane
{
	leftlane = 0,
	centerlane = 1,
	rightlane = 2
};


class Vehicle
{
	public:

		Vehicle();
		~Vehicle();

		//Car Parameters from Simulator
		double x;
		double y;
		double s;
		double d;
		double yaw;
		double speed;
		double prev_size;	

		vector<double> next_x_vals;
        vector<double> next_y_vals;

		vector<double> prev_x_vals;
        vector<double> prev_y_vals;

        vector<double> map_waypoints_x;
  		vector<double> map_waypoints_y;
  		vector<double> map_waypoints_s;
  		vector<State> possible_future_states;

  		vector < vector<double> >  sensor_fusion;

		State current_state;

		bool displayPoints;
		double speed_limit; //speed limit in mph
		double total_points; // Total # of points to keep       	       	
		double keep_points;   //Number of points to include from previous path
		double v_adj; //Speed to adjust each iteration by	


 
 		// Spline object representing trajectory
        spline spl;
       

      	//Points to represent the state of the vehicle at the end of the previous path.
      	double ref_x;
      	double ref_y;
      	double ref_x_prev;
      	double ref_y_prev;
        double ref_yaw;         

		Lane ego_lane;
		double ref_v;
		bool too_close;	

		void GetSuccessorStates();
		void UpdateSpeed();
		void GenerateTrajectory();
		void StateTransition();
		Lane GetCurrentLane();
	    
    private:

    	vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
    	double deg2rad(double x) { return x * M_PI/ 180; }
		double rad2deg(double x) { return x * 180 / M_PI; }


		double computeOffCenterCost(State a);
		double computeRearEndCollisionCost();
		string StatetoString(State a);
		string LanetoString(Lane a);
		
};


#endif