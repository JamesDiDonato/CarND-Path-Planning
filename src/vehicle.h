#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <math.h>

using namespace std;



class Vehicle
{
	public:


		Vehicle();
		~Vehicle();


		//Car Parameters from Simulator
		double car_x;
		double car_y;
		double car_s;
		double car_d;
		double car_yaw;
		double car_speed;
		double ref_v;
		uint num_points;
    
};


#endif