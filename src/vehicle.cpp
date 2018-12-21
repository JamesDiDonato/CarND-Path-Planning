#include "vehicle.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include "spline.h"
#include <math.h>
#include <fstream>
#include <string>

Vehicle::~Vehicle() {}

Vehicle::Vehicle()
{
	displayPoints = false;
	speed_limit = 50; //speed limit in mph
	total_points = 40 ; // Total # of points to keep       	       	
	keep_points = 35;   //Number of points to include from previous path
	v_adj = 0.2; //Speed to adjust each iteration by	


	ego_lane = centerlane; // vehicle starts out in the center lane

	too_close = false;

	//ref_v = speed;
	ref_v = 3;

	current_state = keep_lane;

}
/* 
* The Finite State Machine Structure.
* Returns all possible future states for given current state. 
* Confines the results to the three lanes on the highway
*/
void Vehicle::GetSuccessorStates()
{
	possible_future_states.clear();

	Lane currentLane =  GetCurrentLane();
	cout << "Current State = " << StatetoString(current_state) << ". ";

	if(current_state == keep_lane)
	{
		possible_future_states.push_back(keep_lane); // can always stay in your lane!
		switch(currentLane)//limit maneuvers based on current lane
		{
			case leftlane: //avoid oncoming traffic
				possible_future_states.push_back(prep_lane_change_right); 
				break;

			case rightlane: //avoid off roading
				possible_future_states.push_back(prep_lane_change_left); 
				break;
			case centerlane: //can go either way
				possible_future_states.push_back(prep_lane_change_left);
				possible_future_states.push_back(prep_lane_change_right);
				break;

		}
	}

	else if (current_state == prep_lane_change_right)
	{
		possible_future_states.push_back(prep_lane_change_right);
		possible_future_states.push_back(lane_change_right);
	}

	else if (current_state == prep_lane_change_left)
	{
		possible_future_states.push_back(prep_lane_change_left);
		possible_future_states.push_back(lane_change_left);
	}

	else if (current_state == lane_change_right)
	{
		possible_future_states.push_back(lane_change_right);
		possible_future_states.push_back(keep_lane);		
	}

	else if (current_state == lane_change_left)
	{
		possible_future_states.push_back(lane_change_left);		
		possible_future_states.push_back(keep_lane);				
	}
    cout << "Possible future states:" << endl;

	for (int i = 0; i < possible_future_states.size(); i ++)
	{
	  cout<< "\t" << StatetoString(possible_future_states[i]) << endl;
	}
}


/* 
* Penalize state transitions that don't end up in center lane
*/
double Vehicle::computeOffCenterCost(State a)
{
	uint8_t target_lane;
	double offCenterCost = 0.5;

	//Moving to State a would put ego in the target_lane
	switch(a)
	{
		case keep_lane :
			target_lane = GetCurrentLane();
			break;

		case prep_lane_change_right :
			target_lane = GetCurrentLane() + 1;
			break;

		case prep_lane_change_left:
			target_lane = GetCurrentLane() - 1;
			break;
	}

	//Penalize moving to either the left or center lane. Want to stay in center.
	if(target_lane == leftlane || target_lane == rightlane)
	{
		return offCenterCost;
	}
	else
	{
		return 0; // We want to stay in center lane
	}
}

//Computes a cost relative to the potential of hitting vehicle in front
double Vehicle::computeRearEndCollisionCost()
{
	//Sensor Fusion Format : [ id, x, y, vx, vy, s, d]
	double distance_warning = 30;
	double id,x,y,vx,vy,sf,dist;
	for ( int i = 0; i <sensor_fusion.size();i++)
	{         			
		dist = sensor_fusion[i][6];
		sf = sensor_fusion[i][5];
		id = sensor_fusion[i][0];
		//cout << "Checking Vehicle "<< sensor_fusion[i][0] << "."<< endl;
		//Determine if vehicle is in ego lane

		if(dist <(4+ 4*ego_lane) && dist > (4*ego_lane))
		{
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double check_speed = sqrt(vx*vx + vy*vy);

			if ( abs(sf - this->s) < distance_warning && (sf > this->s))
			{
			    cout << "Found a vehicle too close!!!!" << endl;
			    return 1.0;
			}	         			
		}
	}
	return 0;
}


void Vehicle::StateTransition()
{
	
	//For each possible state transition, compute the associated cost
	vector<double> costs;
	double costsForState = 0;

	for (int i = 0 ; i < possible_future_states.size();i++)
	{
		costsForState = 0;
		//For each possible state, compute the associated costs:

		costsForState += computeOffCenterCost(possible_future_states[i]);
		costsForState += computeRearEndCollisionCost(possible_future_states[i]);
		costs.push_back(costsForState);
	}


	//Debugging the cost functions:
	for (int j = 0 ; j < possible_future_states.size();j++)
	{
		cout<< "To transition to state '" << StatetoString(possible_future_states[j]) << "', Cost = "<< costs[j]<<endl;
	}



/*
  for state in possible_successor_states:
        # generate a rough idea of what trajectory we would
        # follow IF we chose this state.
        trajectory_for_state = generate_trajectory(state, current_pose, predictions)


       # calculate the "cost" associated with that trajectory.
        cost_for_state = 0
        for i in range(len(cost_functions)) :
            # apply each cost function to the generated trajectory
            cost_function = cost_functions[i]
            cost_for_cost_function = cost_function(trajectory_for_state, predictions)

            # multiply the cost by the associated weight
            weight = weights[i]
            cost_for_state += weight * cost_for_cost_function
         costs.append({'state' : state, 'cost' : cost_for_state})

    # Find the minimum cost state.
    best_next_state = None
    min_cost = 9999999
    for i in range(len(possible_successor_states)):
        state = possible_successor_states[i]
        cost  = costs[i]
        if cost < min_cost:
            min_cost = cost
            best_next_state = state 

    return best_next_state
*/

}

/* 
* Generates the Trajectory using spline.h, targeted lane, targeted speed.
*/
void Vehicle::GenerateTrajectory()
{

	/* Generate Spline for Path*/

	vector<double> ptsy;
	vector<double> ptsx;

	double spline_starting_x;
	double spline_starting_y;


	// If previous size is almost empty, estimate the previous
	// 2 way points based on vehicle orientation
	if (prev_size <2)
	{
		ref_yaw = deg2rad(yaw);
		double prev_car_x = x - cos(ref_yaw);
		double prev_car_y = y - sin(ref_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(y);

		spline_starting_x = prev_car_x;
		spline_starting_y = prev_car_y;

		ref_x = x;
		ref_y = y;       
	}           


	else //Use the previous path's ending points as the starting point
	{
		//redefine reference state as the end of the previous point array
		ref_x = prev_x_vals[keep_points-1];
		ref_y = prev_y_vals[keep_points-1];
		ref_x_prev = prev_x_vals[keep_points-2];
		ref_y_prev = prev_y_vals[keep_points-2];

		spline_starting_x = ref_x_prev;
		spline_starting_y = ref_y_prev;

		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

	//Get waypoints up the road to generate spline
	vector<double> next_wp0 = getXY(s + 30, 2+4*ego_lane , map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = getXY(s + 60, 2+4*ego_lane , map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(s + 90, 2+4*ego_lane , map_waypoints_s, map_waypoints_x, map_waypoints_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);


	//Print Spline Waypoints in Global Coordinates
	if(displayPoints)
	{
		cout << "Global Spline Waypoints: " << endl;
		for (int i = 0 ; i < ptsx.size(); i++)
		{
			//print Global Spline Waypoints :
			cout << "\t[ " << ptsx[i] << "," << ptsy[i] << " ]" << endl;
		}
	}

	// Transform to local coordinates by setting the origin to the start of the spline path
	for (int i = 0 ; i < ptsx.size(); i++)
	{
		double shift_x = ptsx[i] - spline_starting_x;
		double shift_y = ptsy[i] - spline_starting_y;

		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0- ref_yaw));
		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0- ref_yaw)); 
	}

	//Print Spline Waypoints in Local Coordinates
	if(displayPoints)
	{
		
		cout << "Local Spline Waypoints: " <<endl;
			for (int i = 0 ; i < ptsx.size(); i++)
			{
				//print spline waypoints Global:
				cout << "\t[" << ptsx[i] << "," << ptsy[i] << "]" << endl;
			}
		cout << endl;
	}

	spl.set_points(ptsx,ptsy);

	/* Write to the output points */
	next_x_vals.clear();
	next_y_vals.clear();
    


	// Push the old values if there are any, defined by keep_points
    cout << "\nAdding " << min(prev_size,keep_points) <<" points from previous path:" <<endl;

  	for (int i = 0; i < min(prev_size,keep_points); i++)
  	{
		next_x_vals.push_back(prev_x_vals[i]);
		next_y_vals.push_back(prev_y_vals[i]);
     if(displayPoints){cout << "\t Adding Previous Point = [" << prev_x_vals[i] << " , " << prev_y_vals[i] <<"]" << endl;}
  	}
    cout << "Done Adding points from previous path." <<endl;  


  	//Generate new points:

  	double local_x_coord = 0;
  	double local_y_coord = 0;
    double global_x_coord = 0;
    double global_y_coord = 0;


  	//Compute longitudinal increment based on reference speed & simulator physics
  	double x_inc = ref_v*0.02/(2.237);
  	
    cout << "\nGenerating "<<  total_points - min(keep_points,prev_size)  <<  " Points for Future Vehicle Path:" <<endl;

	for (int i = 0; i < total_points - min(keep_points,prev_size); i ++)
	{
		//Local (x,y) coordinate for point
		local_x_coord += x_inc;
		local_y_coord = spl(local_x_coord); //Generate spline value

		if(displayPoints)
		{
			cout << "\tLocal = [" << local_x_coord <<" , " << local_y_coord <<"] , ";
		}

		//Convert local coordinates back to global:
		double x_ref = local_x_coord;
		double y_ref = local_y_coord;

		global_x_coord = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
		global_y_coord = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

		global_x_coord += ref_x;
		global_y_coord += ref_y;

		next_x_vals.push_back(global_x_coord);
		next_y_vals.push_back(global_y_coord);  

		if(displayPoints)
		{
			cout << "Global = [" << global_x_coord <<" , " << global_y_coord <<"] " <<  endl;
		}
  	}
     cout << "Done Generating New Points." <<endl;  
          	

}


\
// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Vehicle::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

Lane Vehicle::GetCurrentLane()
{
	if(d < 4.0)
	{
		return leftlane;
	}

	else if (d < 8.0)
	{
		return centerlane;
	}
	else
	{
		return rightlane;
	}
}

void Vehicle::UpdateSpeed()
{

	double start_add = 1.2;
	double end_add = 0;
	double speed_add=  speed * (end_add - start_add)/(speed_limit - 3) + start_add;
	ref_v += speed_add;

	if(ref_v > 45){ego_lane = leftlane;}

}

string Vehicle::StatetoString(State a)
{
	switch(a)
	{
		case keep_lane:
			return "Keep Lane";
			break;
		case prep_lane_change_right:
			return "Prep Lane Change Right";
			break;
		case prep_lane_change_left:
			return "Prep Lane Change Left";
			break;
		case lane_change_right:
			return "Lane Change Right";
			break;
		case lane_change_left:
			return "Lane Change Left";
			break;

	}
}

string Vehicle::LanetoString(Lane a)
{
	switch(a){			
		case leftlane:
			return "Left Lane";
			break;
		case centerlane:
			return "Center Lane";
			break;
		case rightlane:
			return "Right Lane";
			break;
		}
}