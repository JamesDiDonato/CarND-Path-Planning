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
	total_points = 60 ; // Total # of points to keep       	       	
	keep_points = 30;   //Number of points to include from previous path

	ego_lane = centerlane; // vehicle starts out in the center lane
	target_lane = centerlane ;// vehicle starts out in the center lane

	ref_v = 2;

	current_state = keep_lane;

	sim_start = true;

}
/* 
* The Finite State Machine Structure.
* Returns all possible future states for given current state. 
* Confines the results to the three lanes on the highway (ie.. cant change left when in passing lane)
*/
void Vehicle::GetSuccessorStates()
{
	possible_future_states.clear();

	Lane currentLane =  GetCurrentLane();
	cout << "Current State = " << StatetoString(current_state) << ", in " << LanetoString(GetCurrentLane()) <<"."<<endl;

	if(current_state == keep_lane)
	{
		possible_future_states.push_back(keep_lane); // can always stay in your lane!

		switch(currentLane)//limit maneuvers based on where we are
		{
			case leftlane: //avoid oncoming traffic
				possible_future_states.push_back(lane_change_right); 
				break;

			case rightlane: //avoid off roading
				possible_future_states.push_back(lane_change_left); 
				break;
			case centerlane: //can go either way
				possible_future_states.push_back(lane_change_left);
				possible_future_states.push_back(lane_change_right);
				break;
		}
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
}


/* 
* Penalize state transitions that don't end up in center lane, aim to stay in center lane
*/
double Vehicle::OffCenterCost(State a)
{
	uint8_t target_lane;
	double offCenterCost = 0.25;

	//Moving to State a would put ego in the target_lane
	switch(a)
	{
		case keep_lane :
			target_lane = GetCurrentLane();
			break;

		case lane_change_right :
			target_lane = GetCurrentLane() + 1;
			break;

		case lane_change_left:
			target_lane = GetCurrentLane() - 1;
			break;
	}

	//Penalize driving in higher risk lanes
	if(target_lane == leftlane || target_lane == rightlane)
	{
		return offCenterCost;
	}
	else
	{
		return 0; // We want to stay in center lane :)
	}
}

/* 
* Computes a cost relative to the potential of hitting vehicle in front
* Cost increases the closer you get
*/
double Vehicle::RearEndCollisionCost(State a)
{
	double distance_warning = 40;

	if(a == keep_lane) //This cost is only relevent if you want to stay in the same lane
	{
		//Sensor Fusion Format : [ id, x, y, vx, vy, s, d]
	
		double sf,d_ext,dist_relative ;
		double min_distance = GetInLaneThreat()[0]; // The closest in-lane vehicle distance away

		if(min_distance < 16)  // We are too close to change lanes, wait for ACC to slow us down
		{
			return 0;
		}
		else
		{
			//Compute cost linearly proportional to closest in lane vehicle distance
			return 1.5*(distance_warning - min(min_distance,distance_warning))/distance_warning;
		}



	}

	else
	{
		return 0;
	}

}
/*
* This cost function represents the amount of open space present in the potential future state a
* The cost is lower if the available open space gap is smaller
*/
double Vehicle::OpenLaneCost(State a)
{	
	if(a != keep_lane) //This cost is only relevent for states that would transition ego's lane
	{
		double gap_front = 50;// Amount of space ahead ego to include in gap, in m

		double gap_back = -7.5; // Amount of space behind ego to include in gap, in m
		double sf,d_ext,gap_size, dist_to_ego;

		double min_gap_size = gap_front - gap_back;


		//Compute the size of the gap for both lane change states

		if(a == lane_change_right)
		{
			//Compute the smallest gap size in the right lane
			for ( int i = 0; i <sensor_fusion.size();i++)
			{
				d_ext = sensor_fusion[i][6];
				sf = sensor_fusion[i][5];

				if(d_ext < (4*(2+ego_lane)) && d_ext > (4*(ego_lane+1))) //See if vehicle is in right lane
				{
					//Start back gap_back meters, determine how large gap is until this vehicle 

					dist_to_ego  = sf - this->s;

					if((dist_to_ego > gap_back) && (dist_to_ego < gap_front)) 
					{
						//Vehicle is inside the gap from [-gap_back,gap_front], now find worst case gap
						gap_size = dist_to_ego - gap_back;
						if(gap_size <min_gap_size)
						{
							min_gap_size = gap_size;
						}
					}
				}
			}				
		}
		else if (a == lane_change_left)
		{
			//Compute the smallest gap size in the left lane
			for ( int i = 0; i <sensor_fusion.size();i++)
			{
				d_ext = sensor_fusion[i][6];
				sf = sensor_fusion[i][5];

				if(d_ext < (4*(ego_lane)) && d_ext > (4*(ego_lane-1))) //See if vehicle is in left lane
				{
					//Start back gap_back meters, determine how large gap is until this vehicle 

					dist_to_ego = sf - this->s;

					if((dist_to_ego > gap_back) && (dist_to_ego < gap_front)) 
					{
						//Vehicle is inside the gap from [gap_back,gap_front], now find worst case gap
						gap_size = dist_to_ego - gap_back;	
						if(gap_size <min_gap_size)
						{
							min_gap_size = gap_size;
						}
					}
					
				}
			}		
		}
		//The cost gets worse the closer the vehicle gets to gap_back [m] from ego.
		return 2*(gap_front - gap_back - min_gap_size)/(gap_front - gap_back);
	}


	else // Don't punish for staying in lane
	{
		return 0;
	}
}

void Vehicle::StateTransition()
{	

	//State transition occurs once the vehicle has successfully entered the right lane
	if(current_state == lane_change_right)
	{
		target_lane = (Lane)((int)ego_lane + 1);

		if(GetCurrentLane() ==  target_lane)
		{
			current_state = keep_lane;
			cout<<"Succesfully entered " << LanetoString(target_lane) << "." <<endl;
			ego_lane = target_lane;
		}
	}
	//State transition occurs once the vehicle has successfully entered the left lane
	else if (current_state == lane_change_left)
	{
		target_lane = (Lane)((int)ego_lane - 1);

		if(GetCurrentLane() ==  target_lane)
		{
			current_state = keep_lane;
			cout<<"Succesfully entered " << LanetoString(target_lane) << "." <<endl;
			ego_lane = target_lane;
		}
	}

	//For each possible state transition out of "keep_lane" compute the associated cost
	else
	{
		vector<double> costs;
		double c1= 0,c2= 0,c3 = 0;

		cout << "Possible future states:" << endl;
		cout << "\t" << "State________________" <<"\t" <<"OffC"<<"\t" <<"RE" <<"\t" <<"OL" <<"\t" << "Tot" << endl;
		for (int i = 0 ; i < possible_future_states.size();i++)
		{			
			//For each possible state, compute the associated costs:
			cout<< "\t" << StatetoString(possible_future_states[i]);

			c1 = OffCenterCost(possible_future_states[i]);		
			c2 = RearEndCollisionCost(possible_future_states[i]);		
			c3 = OpenLaneCost(possible_future_states[i]);


			cout<< "\t" << round(100*c1)/100.0;
			cout<< "\t" << round(100*c2)/100.0;
			cout<< "\t" << round(100*c3)/100.0;
			cout<< "\t" << round(100*(c1+c2+c3))/100.0 <<endl;
			
			costs.push_back(c1+c2+c3);
			//costs.push_back(c1+c3); //Uncomment this line to avoid changing lanes
		}

		// Decide on next state based on the lowest cost:
		double lowest_cost = 999;
		for (int j = 0 ; j < possible_future_states.size(); j++)
		{
			if(costs[j] < lowest_cost)
			{
				current_state = possible_future_states[j];
				lowest_cost = costs[j];
			}
		}

		cout << "Next State = " << StatetoString(current_state) << endl;
	}
}

/* 
* Generates the Trajectory for the current_lane.
* Uses spline.h, targeted lane, targeted speed.
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
	if (prev_size <2 )
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
		ref_x = prev_x_vals[min(keep_points,prev_size)-1];
		ref_y = prev_y_vals[min(keep_points,prev_size)-1];
		ref_x_prev = prev_x_vals[min(keep_points,prev_size)-2];
		ref_y_prev = prev_y_vals[min(keep_points,prev_size)-2];

		spline_starting_x = ref_x_prev;
		spline_starting_y = ref_y_prev;

		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

	//Get waypoints up the road to generate spline
	vector<double> next_wp0 = getXY(s + 40, 2+4*target_lane , map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = getXY(s + 60, 2+4*target_lane , map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(s + 90, 2+4*target_lane , map_waypoints_s, map_waypoints_x, map_waypoints_y);

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
			cout << "\t[ " << ptsx[i] << " , " << ptsy[i] << " ]" << endl;
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
				cout << "\t[" << ptsx[i] << " , " << ptsy[i] << "]" << endl;
			}
		cout << endl;
	}

	spl.set_points(ptsx,ptsy);

	/* Write to the output points */
	next_x_vals.clear();
	next_y_vals.clear();
    


	// Push the old values if there are any, defined by keep_points
    if(displayPoints){cout << "\nAdding " << min(prev_size,keep_points) <<" points from previous path:" <<endl;}

  	for (int i = 0; i < min(prev_size,keep_points); i++)
  	{
		next_x_vals.push_back(prev_x_vals[i]);
		next_y_vals.push_back(prev_y_vals[i]);
     if(displayPoints){cout << "\t Adding Previous Point = [" << prev_x_vals[i] << " , " << prev_y_vals[i] <<"]" << endl;}
  	}
    if(displayPoints){cout << "Done Adding points from previous path." <<endl; }


  	//Generate new points:

  	double local_x_coord = 0;
  	double local_y_coord = 0;
    double global_x_coord = 0;
    double global_y_coord = 0;


  	//Compute longitudinal increment based on reference speed & simulator physics
  	double x_inc = ref_v*0.02/(2.237);
  	
    if(displayPoints){cout << "\nGenerating "<<  total_points - min(keep_points,prev_size)  <<  " Points for Future Vehicle Path:" <<endl;}

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
     if(displayPoints){cout << "Done Generating New Points." <<endl; cout<< "Previous Size =" << prev_size << endl; }
          	

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


/*
* This funtion implements adaptive cruise control by using the sensor fusion data and speed limit to set the 
* reference velocity for the trajectory generator
*/
void Vehicle::AdaptiveCruise()
{

	double holding_distance = 20;
	double start_slowing = 40;
	
	double error;
	double p;
	double rate = 0;

	double min_distance = GetInLaneThreat()[0];
	double vehicle_speed = GetInLaneThreat()[1];
	cout<< "SimStart = "<<  sim_start<<". Distance to next vehicle   = " << min_distance <<". Travelling at "<< vehicle_speed<< endl;
	

	if(sim_start) //We are just getting up to speed
	{
		rate = 1;
		ref_v += rate;
		if(ref_v > (speed_limit -5)){sim_start = false; }
	}

	else
	{

		if(false){ref_v = speed_limit -3; } //stop increasing beyond speed limit

		else
		{	

			if (current_state != keep_lane) // Tap the throttle when changing lanes
			{
				rate = 1;
				ref_v = min(ref_v + rate, speed_limit-2);
			}
			
			else if (min_distance < holding_distance)
			{
				//USE PID controller to slow car
				error = (min_distance - holding_distance);
				p = 0.05;
				rate = min(1.0,p*error); //cap the rate at 1.0 to keep acc and jerk < 10 
				ref_v = min(ref_v + rate, speed_limit-2);
			}
			else if (min_distance < start_slowing )
			{
				error = (min_distance - start_slowing);
				p = 0.005;
				rate = min(1.0,p*error); //cap the rate at 1.0 to keep acc and jerk < 10 
				
				ref_v = min(ref_v + rate, speed_limit-2);
			}

			else  if ( min_distance > start_slowing)// vehicle is further enough away we can accelerate.
			{
				rate = 0.25; //cap the rate at 1.0 to keep acc and jerk < 10 
				ref_v = min(ref_v + rate, speed_limit-2);
			}

		}

	}
	cout<< "Speed rate= " << rate <<endl;
}

/*
* This function searches the sensor fusion array and extracts the closest vehicle ahead of ego in the same lane
* returns the distance and speed 
*/
vector<double> Vehicle::GetInLaneThreat()
{
	double d_ext,sf,dist_relative,vehicle_speed,min_distance = 100;

	// Front Radar,extract high threat vehicle in same lane:
	for ( int i = 0; i <sensor_fusion.size();i++)
	{         			
		//Sensor Fusion Format : [ id, x, y, vx, vy, s, d]
		d_ext = sensor_fusion[i][6];
		sf = sensor_fusion[i][5];


		//Determine if vehicle is in our lane
		if(d_ext <(4+ 4*ego_lane) && d_ext > (4*ego_lane))
		{			
			//Compute in-lane distance ahead
			if(sf>this->s)
			{
				dist_relative = sf - this->s ;

				if(dist_relative < min_distance)
				{
					min_distance = dist_relative;
					vehicle_speed = sqrt(pow(sensor_fusion[i][3],2) + pow(sensor_fusion[i][4],2));					
				}
			}			
		}
	}
	vector<double> out;
	out.push_back (min_distance);
	out.push_back (vehicle_speed);
	return out;

}


string Vehicle::StatetoString(State a)
{
	switch(a)
	{
		case keep_lane:
			return "Keep Lane_____________";
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