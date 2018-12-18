#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"

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
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  Vehicle ego = Vehicle();

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
  while (getline(in_map_, line)) {
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') 
    {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") 
        {
          //cout << ego.num_points << endl;
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
        	  

        	  // Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];

        	  //Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];


          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
          	//Sensor Fusion Format : [ id, x, y, vx, vy, s, d]


          	// PROJECT CODE BEGINS:		

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            bool displayPoints = false;


           

          	//Points to represent the last and second last points in the previous path
          	double ref_x;
          	double ref_y;
          	double ref_x_prev;
          	double ref_y_prev;
            double ref_yaw;          

            uint prev_size = previous_path_x.size();
            double speed_limit = 47; //speed limit in mph
            uint total_points = 50;          	
            uint points_to_add; // Number of points to be added to the path         	
            uint keep_points = prev_size - 10;  //Number of points to include from previous path

            uint ego_lane = 1;
  	
          	
            double v_adj = 2.0; //Speed to adjust each iteration by

          	bool too_close = false;
            

            double ref_v = car_speed;

          	
            cout << "\n\nPrevious Size Remainder = " << prev_size << endl;
            cout << "Current Ego Position  = [ " << car_x  << " , " << car_y << " ]" <<endl;
            cout << "Current Ego Speed  = " << car_speed << endl;



            
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

	         			if ( abs(sf - car_s < 30) && (sf > car_s))
	         			{
	         				too_close = true;
                  cout << "Found a vehicle too close!!!!  = " << car_speed << endl;
	         			}	         			
         			}
         		}

            if(too_close)
            {
              ref_v -= v_adj;
              cout << "Decreasing Speed to "<< ref_v << "." << endl;
            }
            else if(ref_v < speed_limit)
            {
              ref_v += v_adj;
              cout << "Increasing Speed to "<< ref_v << "." << endl;
            }


         		/* Generate Spline for Path*/

          	vector<double> ptsy;
          	vector<double> ptsx;

            double spline_starting_x;
            double spline_starting_y;


          	// If previous size is almost empty, estimate the previous
          	// 2 way points based on vehicle orientation
          	if (prev_size <2)
          	{
              ref_yaw = deg2rad(car_yaw);
          		double prev_car_x = car_x - cos(ref_yaw);
          		double prev_car_y = car_y - sin(ref_yaw);

          		ptsx.push_back(prev_car_x);
          		ptsx.push_back(car_x);

          		ptsy.push_back(prev_car_y);
          		ptsy.push_back(car_y);

              spline_starting_x = prev_car_x;
              spline_starting_y = prev_car_y;

              ref_x = car_x;
              ref_y = car_y;       
          	}           


            else //Use the previous path's ending points as the starting point
          	{
          		//redefine reference state as the end of the previous point array
          		ref_x = previous_path_x[keep_points-1];
          		ref_y = previous_path_y[keep_points-1];
          		ref_x_prev = previous_path_x[keep_points-2];
          		ref_y_prev = previous_path_y[keep_points-2];

              spline_starting_x = ref_x_prev;
              spline_starting_y = ref_y_prev;

          		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

          		ptsx.push_back(ref_x_prev);
          		ptsx.push_back(ref_x);

          		ptsy.push_back(ref_y_prev);
          		ptsy.push_back(ref_y);
          	}

          	//Get waypoints up the road to generate spline
          	vector<double> next_wp0 = getXY(car_s + 30, 2+4*ego_lane , map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp1 = getXY(car_s + 60, 2+4*ego_lane , map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp2 = getXY(car_s + 90, 2+4*ego_lane , map_waypoints_s, map_waypoints_x, map_waypoints_y);

          	ptsx.push_back(next_wp0[0]);
          	ptsx.push_back(next_wp1[0]);
          	ptsx.push_back(next_wp2[0]);

          	ptsy.push_back(next_wp0[1]);
          	ptsy.push_back(next_wp1[1]);
          	ptsy.push_back(next_wp2[1]);

            if(displayPoints){
              //Print Global Spline Waypoints
              cout << "Global Spline Waypoints: " << endl;
              for (int i = 0 ; i < ptsx.size(); i++)
              {
                //print Global Spline Waypoints :
                cout << "\t[ " << ptsx[i] << "," << ptsy[i] << " ]" << endl;
              }
              cout << endl;
            }
          	// Transform to local coordinates by setting the origin to the start of the spline path
          	for (int i = 0 ; i < ptsx.size(); i++)
          	{
          		double shift_x = ptsx[i] - spline_starting_x;
          		double shift_y = ptsy[i] - spline_starting_y;

          		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0- ref_yaw));
          		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0- ref_yaw)); 
          	}
            
            
            if(displayPoints){
              //Print Local Spline Waypoints
              cout << "Local Spline Waypoints: " <<endl;
              for (int i = 0 ; i < ptsx.size(); i++)
              {
                //print spline waypoints Global:
                cout << "\t[" << ptsx[i] << "," << ptsy[i] << "]" << endl;
              }
              cout << endl;
            }

          	//Create spline that maps to lane path
          	tk::spline s;
          	s.set_points(ptsx,ptsy);




          	/* Write to the output points */


            //Figure out how many new points to add :
            points_to_add = total_points - min(keep_points,prev_size);


        		// Push the old values if there are any, defined by keep_points
        		int i;  
            cout << "\nAdding " << min(prev_size,keep_points) <<" points from previous path:" <<endl;

          	for (i = 0; i < min(prev_size,keep_points); i++)
          	{
        			next_x_vals.push_back(previous_path_x[i]);
							next_y_vals.push_back(previous_path_y[i]);
             if(displayPoints){cout << "\t Adding Previous Point = [" << previous_path_x[i] << " , " << previous_path_y[i] <<"]" << endl;}
          	}
            cout << "Done Adding points from previous path.\n" <<endl;  


          	//Generate new points:

          	double local_x_coord = 0;
          	double local_y_coord = 0;
            double global_x_coord = 0;
            double global_y_coord = 0;


          	//Compute longitudinal increment based on reference speed & simulator physics
          	double x_inc = ref_v*0.02/(2.237);
          	
            cout << "\nGenerating "<<  total_points - min(keep_points,prev_size)  <<  " Points for Future Vehicle Path:" <<endl;

          	for (i = 0; i < total_points - min(keep_points,prev_size); i ++)
        		{
        			//Local (x,y) coordinate for point
        			local_x_coord += x_inc;
        			local_y_coord = s(local_x_coord); //Generate spline value
              if(displayPoints){cout << "\tLocal = [" << local_x_coord <<" , " << local_y_coord <<"] , ";}

        			//Convert local coordinates back to global:
        			double x_ref = local_x_coord;
        			double y_ref = local_y_coord;

        			global_x_coord = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        			global_y_coord = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

        			global_x_coord += ref_x;
        			global_y_coord += ref_y;

        			next_x_vals.push_back(global_x_coord);
							next_y_vals.push_back(global_y_coord);  
              		
              if(displayPoints){cout << "Global = [" << global_x_coord <<" , " << global_y_coord <<"] " <<  endl;}
          	}
             cout << "Done Generating New Points.\n" <<endl;  
          	
          	json msgJson;
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
	      else
	      {
	        // Manual driving
	        std::string msg = "42[\"manual\",{}]";
	        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	      }
	    }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}


          	/*
          	uint lane = 1; //lane 0 = left, 1 = center, 2 = right
          	double ref_v = 50; // reference velocity
          	uint max_speed = 48;
          	uint num_points = 50;	
          	bool too_close = false;

          	// Collision Avoidance Logic:

         		int id,x,y,vx,vy,sf,dist;
         		//cout << "id's val's = ";
         		for ( int i = 0; i <sensor_fusion.size();i++)
         		{

         			double d = sensor_fusion[i][6];


         			//Determine if vehicle is in ego lane
         			if(d <(4+ 4*lane) && d > (4*lane))
         			{
	         			double vx = sensor_fusion[i][3];
	         			double vy = sensor_fusion[i][4];
	         			double check_speed = sqrt(vx*vx + vy*vy);
	         			double check_s = sensor_fusion[i][5];

	         			if (check_s - car_s < 30) 
	         			{
	         				too_close = true;

	         			}
         			}
         		}

         		if(false)
         		{
         			ref_v -= .224;
         			cout << "Decreasing Speed!\n";
         		}
         		else if (ref_v < max_speed)
         		{
         			ref_v+=0.224;
         			cout << "Increasing Speed!\n";
         		}


          	vector<double> ptsx;
          	vector<double> ptsy;

          	int prev_size = previous_path_x.size();

          	//either reference the starting point as current car location or at the previous path's end point
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);

          	//If previous size is almost empty, estimate the previous 2 way points
          	if (prev_size <2)
          	{
          		double prev_car_x = car_x - cos(car_yaw);
          		double prev_car_y = car_y - sin(car_yaw);

          		ptsx.push_back(prev_car_x);
          		ptsx.push_back(car_x);


          		ptsy.push_back(prev_car_y);
          		ptsy.push_back(car_y);
          	}

          	else //Use the previous path's end point as the starting point
          	{
          		//redefine reference state as the end of the last simulation cycle
          		ref_x = previous_path_x[prev_size-1];
          		ref_y = previous_path_y[prev_size-1];


          		double ref_x_prev = previous_path_x[prev_size-2];
          		double ref_y_prev = previous_path_y[prev_size-2];

          		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

          		ptsx.push_back(ref_x_prev);
          		ptsx.push_back(ref_x);


          		ptsy.push_back(ref_y_prev);
          		ptsy.push_back(ref_y);
          	}

          	vector<double> next_wp0 = getXY(car_s + 30, 2+4*lane , map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp1 = getXY(car_s + 60, 2+4*lane , map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp2 = getXY(car_s + 90, 2+4*lane , map_waypoints_s, map_waypoints_x, map_waypoints_y);

          	ptsx.push_back(next_wp0[0]);
          	ptsx.push_back(next_wp1[0]);
          	ptsx.push_back(next_wp2[0]);

          	ptsy.push_back(next_wp0[1]);
          	ptsy.push_back(next_wp1[1]);
          	ptsy.push_back(next_wp2[1]);


          	//Transform to local cars coordinates
          	//Set the origin to the current vehicle position
          	for (int i = 0 ; i < ptsx.size(); i++)
          	{
          		double shift_x = ptsx[i] - ref_x;

          		double shift_y = ptsy[i] - ref_y;

          		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0- ref_yaw));
          		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0-ref_yaw)); 
          	}

          	//Create spline that maps to lane path
          	tk::spline s;
          	s.set_points(ptsx,ptsy);



						

						int previous_size = 0;
          	//Re-use left over points that were not executed last time step
          	for (int i = 0; i < previous_size;i++)
          	{
          		next_x_vals.push_back(previous_path_x[i]);
          		next_y_vals.push_back(previous_path_y[i]);
          	}
          	
          	//use a linear approximation to genereate spline x-axis:
          	double target_x = 30 ;
          	double target_y = s(target_x);
          	double target_dist = sqrt((target_x*target_x + target_y *target_y));

          	//Compute spacing of points required to acheive target speed

          	double N = (target_dist*2.24/(0.02*ref_v));
          	//double x_inc = target_x / N;
          	double x_inc = ref_v*0.02/(2.237);
          	double x_add = 0;
          	int i;
          	//Add on new points to the end

*/			