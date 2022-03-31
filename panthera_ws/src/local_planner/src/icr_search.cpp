#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <local_planner/CmapClear.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <cmath>
#include <local_planner/icr_utils.h>
#include <panthera_locomotion/Status.h>
#include <fstream>
using namespace std;
class Robot
{
	private:
		ros::Subscriber CostMap;
		ros::Subscriber RobotPose;
		ros::Subscriber RobotWidth;
		ros::Subscriber turn_angle;

		ros::Publisher cmap_clear;
		ros::Publisher robot_footprint;
		ros::Publisher search_area_pub;
		ros::Publisher angle_pub;
		ros::Publisher vel_pub;

		ros::ServiceClient lb_stat, rb_stat, lf_stat, rf_stat;

		bool operation;

		// Footprint info
		double length, width;
		std::vector<geometry_msgs::Point32> footprint_points; // total cells within footprint
		std::vector<geometry_msgs::Point32> possible_icr;
		std::vector<geometry_msgs::Point32> footprint; // footprint points -> cells

		// Map info
		int len_x=50, len_y=50; // in cells
		double res=0.05;

		std::vector<signed char> data_pts;

		// run once
		int n = 0;

		// angle to rotate
		double angle;
		bool received_angle = false;

		// optimization params
		float h_steer, h_max_rot, h_icr_dist;
		int angle_interval;
		bool gradient_descent;

		// wheel seperation
		float ws_length = 1.3;
		float ws_width = 0.7;
		std::vector<geometry_msgs::Point32> wheels;

		// cmd vel
		geometry_msgs::Twist cmd_angle;
		geometry_msgs::Twist cmd_vel;
		float wz = 0.07;

		// write to file
		ofstream icr_points;
		std::string filename = "new_file.txt";

	public:
		Robot(ros::NodeHandle *nh)
		{	
			CostMap = nh->subscribe("/semantics/costmap_generator/occupancy_grid", 10, &Robot::mapCallback, this);
			RobotWidth = nh->subscribe("/can_encoder", 10, &Robot::widthCallback, this);
			turn_angle = nh->subscribe("/turn_angle", 10, &Robot::rotation_angle, this); // subscribe to get angle to rotate
			robot_footprint = nh->advertise<geometry_msgs::PolygonStamped>("/robot_footprint", 100);
			angle_pub = nh->advertise<geometry_msgs::Twist>("/panthera_cmd", 100);
			vel_pub = nh->advertise<geometry_msgs::Twist>("/reconfig", 100);
			//search_area_pub = nh->advertise<geometry_msgs::PolygonStamped>("/search", 100);

			length = nh->param("/robot_length", 2.2);
			width = nh->param("/robot_width", 1.0);

			// optimization params
			h_steer = nh->param("/steering_function", -1.0);
			h_max_rot = nh->param("/max_rotation_function", 1.0);
			h_icr_dist = nh->param("/min_icr_dist", -1.0);
			angle_interval = nh->param("/angle_sample", 10);
			ws_length = nh->param("/front_back_wheel_sep", 1.5);
			wz = nh->param("/turn_speed", 0.07);
			operation = nh->param("/operation", false);
			gradient_descent = nh->param("gradient_descent", false);

			// write to file name
			filename = nh->param("filename", filename);

			lb_stat = nh->serviceClient<panthera_locomotion::Status>("lb_steer_status");
			rb_stat = nh->serviceClient<panthera_locomotion::Status>("rb_steer_status");
			lf_stat = nh->serviceClient<panthera_locomotion::Status>("lf_steer_status");
			rf_stat = nh->serviceClient<panthera_locomotion::Status>("rf_steer_status");
		}

		struct ICR
		{	
			// index of icr wrt to occupancy grid
			int index;

			// objective functions
			double h1;
			double h2;
			double h3;

			// sum of objective functions
			double h4;

			// desired wheel angles
			std::vector<double> wheel_angles;

			// coordinates of icr
			geometry_msgs::Point32 coordinates;
		};

		void publish_vel(geometry_msgs::Point32 icr, std::vector<geometry_msgs::Point32> wheel_vec)
		{	
			// publish wheel velocities
			auto* ts = &cmd_vel;
			if (withinWheelBase(icr) == true)
			{
				ts->linear.x = -double_dist(wheel_vec[0].x, wheel_vec[0].y, icr.x, icr.y)*res * (abs(angle)/angle) * wz;
				//std::cout << double_dist(wheel_vec[0].x, wheel_vec[0].y, icr.x, icr.y) << std::endl;
				ts->linear.y = double_dist(wheel_vec[1].x, wheel_vec[1].y, icr.x, icr.y)*res * (abs(angle)/angle) * wz;
				//std::cout << double_dist(wheel_vec[1].x, wheel_vec[1].y, icr.x, icr.y) << std::endl;
				ts->linear.z = double_dist(wheel_vec[2].x, wheel_vec[2].y, icr.x, icr.y)*res * (abs(angle)/angle) * wz;
				//std::cout << double_dist(wheel_vec[2].x, wheel_vec[2].y, icr.x, icr.y) << std::endl;
				ts->angular.x = double_dist(wheel_vec[3].x, wheel_vec[3].y, icr.x, icr.y)*res * (abs(angle)/angle) * wz;
				//std::cout << double_dist(wheel_vec[3].x, wheel_vec[3].y, icr.x, icr.y) << std::endl;
			}
			else
			{	
				ts->linear.x = 0;//double_dist(wheel_vec[0].x, wheel_vec[0].y, icr.x, icr.y) * (abs(angle)/angle) * wz;
				ts->linear.y = 0;//-double_dist(wheel_vec[1].x, wheel_vec[1].y, icr.x, icr.y) * (abs(angle)/angle) * wz;
				ts->linear.z = 0;//double_dist(wheel_vec[2].x, wheel_vec[2].y, icr.x, icr.y) * (abs(angle)/angle) * wz;
				ts->angular.x = 0;//double_dist(wheel_vec[3].x, wheel_vec[3].y, icr.x, icr.y) * (abs(angle)/angle) * wz;
			}
			std::cout << *ts << std::endl;
			vel_pub.publish(*ts);

		}

		void publish_angle(ICR icr)
		{	
			// publish wheel angles
			auto* ts = &cmd_angle;
			ts->linear.x = icr.wheel_angles[0]*180/PI;
			ts->linear.y = icr.wheel_angles[1]*180/PI;
			ts->linear.z = icr.wheel_angles[2]*180/PI;
			ts->angular.x = icr.wheel_angles[3]*180/PI;
			angle_pub.publish(*ts);
		}

		void stop_pub()
		{	
			// publish command to stop robot
			auto* ts = &cmd_angle;
			ts->linear.x = 0;
			ts->linear.y = 0;
			ts->linear.z = 0;
			ts->angular.x = 0;
			angle_pub.publish(*ts);

			auto* tv = &cmd_vel;
			tv->linear.x = 0;
			tv->linear.y = 0;
			tv->linear.z = 0;
			tv->angular.x = 0;
			vel_pub.publish(*tv);
		}

		void send_cmds(geometry_msgs::Point32 icr, std::vector<geometry_msgs::Point32> wheel_vec, ICR icr_node)
		{	
			// function to send commands to publish angles/speeds
			publish_angle(icr_node);

			check_steer(); // check if wheels have aligned before moving
			
			publish_vel(icr, wheel_vec);
			float t = abs(angle)/wz;
			ros::Duration(t).sleep(); // wait until robot has rotated correct amount before stopping. Estimated based on rads-1 and time
			stop_pub(); // stop robot
		}

		void check_steer()
		{	
			// check if wheel at the correct angles
			if (operation == true)
			{
				panthera_locomotion::Status lb_req,rb_req,lf_req,rf_req;
				lb_req.request.reconfig = true;
				rb_req.request.reconfig = true;
				lf_req.request.reconfig = true;
				rf_req.request.reconfig = true;
				bool signal = false;
				ros::Rate rate(1);
				int count = 0;
				while (signal == false || count<2 ) // count: makes sure wheel has stopped at correct angle and not overshooting. Check if wheel is at correct angle for 2 secs
				{
					lb_stat.call(lb_req);
					rb_stat.call(rb_req);
					lf_stat.call(lf_req);
					rf_stat.call(rf_req);
					signal = ((bool)lb_req.response.status && (bool)lf_req.response.status && (bool)rb_req.response.status && (bool)rf_req.response.status);
					//std::cout << "Signal: " << signal << std::endl;
					rate.sleep();
					if (signal==true)
					{
						count++;
					}
					else
					{
						count = 0;
					}
				}
				printf("Clear!\n");
			}
		}
			

		// Optimization functions
		std::vector<double> angle_change(int index)
		{	
			// Total deg wheels need to rotate
			geometry_msgs::Point32 ind = index_to_coordinates(index, res, len_x);
			std::vector<double> angles;
			double total_angle;
			for (auto w : wheels)
			{
				double theta = 	gradient_angle(ind.x, ind.y, w.x, w.y); // calculate angle req to turn
				angles.push_back(theta);
				total_angle += abs(theta); // sum the angles
			}
			angles.push_back(total_angle);
			return angles;
		}

		double gradient_angle(int x0, int y0, double x1, double y1)
		{
			// diff in angle
			double grad = atan(-(x1 - x0)/(y1 - y0));
			if (grad >= PI/2)
			{
				grad = PI - grad;
			}
			else if ( grad <= -PI/2)
			{
				grad = PI + grad;
			}
			return grad;
		}

		double distance_from_centre(int index)
		{	
			// calculate distance from cell to centre of robot
			geometry_msgs::Point32 pt = index_to_coordinates(index, res, len_x);
			return distance(pt.x, pt.y, len_x/2, len_y/2)*res;
		}

		double max_rotation(int index, double min_theta)
		{	
			// calculate max possible rotation checking every angle_interval
			bool clear = true;
			geometry_msgs::Point32 pt = index_to_coordinates(index, res, len_x);
			double theta = min_theta;

			while (clear == true && abs(theta) <= 2*PI) // while robot has not hit an obstacle with rotation theta
			{	
				clear = rotation_clear(pt, theta, data_pts);
				if (clear == true)
				{
					if (theta > 0)
					{
						theta += (angle_interval*PI/180);
					}
					else
					{
						theta -= (angle_interval*PI/180);
					}
				}
			}
			return abs(theta);
		}

		// run search + optimize
		void test_run()
		{
			bool j = rotation_clear(footprint_points[footprint_points.size()/2], angle, data_pts);
		}

		geometry_msgs::Point32 best_point(geometry_msgs::Point32 curr_point, std::vector<geometry_msgs::Point32>* neighbours, ICR* bfp)
		{	
			// find best point among neighbours
			std::vector<ICR> vec;
			ICR pt = optimize(curr_point); // get objective functions of point
			vec.push_back(pt);
			for (auto i : *neighbours)
			{
				if (rotation_clear(i, angle, data_pts) == true) // if neighbour point not feasible, ignore, else add to neighbours vector
				{
					ICR x = optimize(i);
					//std::cout << x.h4 << std::endl;
					vec.push_back(x);
				}
			}
			std::sort(vec.begin(), vec.end(), sort_h);
			*bfp = vec[0];
			return vec[0].coordinates;
		}

		void get_neighbours(geometry_msgs::Point32 point, std::vector<geometry_msgs::Point32>* neighbours)
		{	
			//geometry_msgs::Point32 p0;
			//p0.x = point.x;
			//p0.y = point.y;

			// get neighbours of icr, cells surrounding point
			for (int j=-1; j<=1; j++)
			{
				for (int i=-1; i<=1; i++)
				{	
					if ((j==0 && i==0) == false) // exclude point from neighbours vector
					{
						geometry_msgs::Point32 pt;
						pt.x = point.x + i;
						pt.y = point.y + j;
						if (std::find(footprint_points.begin(), footprint_points.end(), pt) != footprint_points.end() && std::find(neighbours->begin(), neighbours->end(), pt) == neighbours->end())
						{
							neighbours->push_back(pt); // check if neighbour is within footprint_points
						}
					}
				}
			}
			std::remove(neighbours->begin(), neighbours->end(), point);

			// double check to make sure point is not in neighbours
			if (std::find(neighbours->begin(), neighbours->end(), point) != neighbours->end())
			{	
				std::remove(neighbours->begin(), neighbours->end(), point);
				printf("Failed to remove point\n");
			}
		}

		void run() // gradient descent
		{	
			geometry_msgs::Point32 current_pt = footprint_points[0]; // init corner point to start search
			//footprint_points[(int)(footprint_points.size()/2)];
			geometry_msgs::Point32 found_pt;
			ICR best_found_point; // keep track of best found point while doing search

			bool first_point_found; // bool if first initial point has been found

			first_point_found = rotation_clear(current_pt, angle, data_pts); // check if corner point can be initiated
			std::vector<geometry_msgs::Point32> neighbours;
			get_neighbours(current_pt, &neighbours);

			int c=0;
			// searching for first point
			while (first_point_found == false)
			{
				bool first_point_found = rotation_clear(current_pt, angle, data_pts);
				if (first_point_found == false)
				{
					get_neighbours(current_pt, &neighbours);
					current_pt = neighbours[c]; // iterate through neighbours, if not feasible, get expand neighbours vector
					//std::cout << c << std::endl;
					if (c == footprint_points.size()) // searched through all possible points in footprint_points
					{	
						std::cout << c << std::endl;
						std::cout << "Final pt: " << current_pt << std::endl;
						std::cout << "No possible icr" << std::endl;
						neighbours.clear();
						break;
					}
				}
				else
				{	
					// found initial point
					//std::cout << neighbours.size() << std::endl;
					current_pt = best_point(current_pt, &neighbours, &best_found_point);
					std::cout << c << std::endl;
					std::cout << current_pt << std::endl;
					break;
				}
				c++;
			}

			if (neighbours.size() != 0)
			{	
				double start = ros::Time::now().toSec();
				neighbours.clear();
				int it=1;
				while (it <= footprint_points.size()) // max numbner of iterable points
				{	
					//std::cout << "Current pt: " << current_pt << std::endl;
					get_neighbours(current_pt, &neighbours); // get neighbours of current point
					/**
					for (auto i : neighbours)
					{	
						std::cout <<"neightbour "<< i << std::endl;
					}
					**/
					found_pt = best_point(current_pt, &neighbours,  &best_found_point);
					if (current_pt == best_found_point.coordinates) // current point is the best point among neighbours
					{	
						printf("---------------------------------------\n");
						std::cout << "h1: " << best_found_point.h1 << std::endl;
						std::cout << "h2: " << best_found_point.h2 << std::endl;
						std::cout << "h3: " << best_found_point.h3 << std::endl;
						std::cout << "h4: " << best_found_point.h4 << std::endl;
						std::cout << "Iterations: " << it-1 << std::endl;
						std::cout << "Completed search: " << found_pt << std::endl;
						break;
					}
					else
					{	
						neighbours.clear();
						current_pt = found_pt;
						std::cout << it << "," << best_found_point.h4 << std::endl;
						it++;
					}
				}
				double end = ros::Time::now().toSec();
				printf("----------------------------------------\n");
				std::cout << "Time taken: " << end-start << "s" << std::endl;
				printf("Search completed!\n");
			}
		}
		void global_run()
		{	
			double start1 = ros::Time::now().toSec();
			for (auto i : footprint_points) // brute force search for all possible points
			{	
				if (rotation_clear(i, angle, data_pts) == true)
				{
					possible_icr.push_back(i); // if point is feasible, add to list
					std::cout << "Searching" << std::endl;
				}
			}
			
			printf("Search Done\n");
			double end1 = ros::Time::now().toSec();
			std::cout << "Search1 time: " << end1-start1 << "s" << std::endl;
			std::cout << "Number of possible ICRs: " << possible_icr.size() << std::endl;
			ICR best_pt;
			
			if (possible_icr.size() == 0) // if no possible icr found
			{
				printf("No possible_icr.\n");
			}

			else
			{	
				double start2 = ros::Time::now().toSec();
				best_pt = global_optimize(possible_icr); // find best global point 
				geometry_msgs::Point32 best_pt_coor = index_to_coordinates(best_pt.index, res, len_x); // get coordinates of best icr
				std::cout << "Best point: " <<  index_to_coordinates(best_pt.index, res, len_x) << std::endl;
				for (int i=0; i<4; i++)
				{
					std::cout <<"Wheel angle " << i << ": " << best_pt.wheel_angles[i]/PI*180 << std::endl;
				}
				double end2 = ros::Time::now().toSec();
				std::cout << "Search2 time: " << end2-start2 << "s" << std::endl;
				send_cmds(best_pt_coor, wheels, best_pt); // publish commands to move robot
			}
			possible_icr.clear();
		}

		ICR optimize(geometry_msgs::Point32 icrs)
		{	
			// calculate objective function
			ICR x;
			x.index = coordinates_to_index(icrs.x, icrs.y, len_x);
			x.h1 = angle_change(x.index)[4];
			x.h2 = max_rotation(x.index, angle);
			x.h3 = distance_from_centre(x.index);
			x.h4 = (x.h1*h_steer + x.h2*h_max_rot + x.h3*h_icr_dist)/(abs(h_icr_dist) + abs(h_max_rot) + abs(h_steer)); // sum h1+h2+h3
			x.coordinates = icrs; // x-y coordinates of icr
			x.wheel_angles = angle_change(x.index); // desired wheel angles

			return x;
		}
		
		ICR global_optimize(std::vector<geometry_msgs::Point32> icrs)
		{	
			icr_points.open(filename, ios::out | ios::binary); // create file to write to
			std::vector<ICR> icr_nodes; // keep track of icrs
			for (auto icr : icrs) // iterate through all possible icrs
			{	
				ICR* x = new ICR();
				x->index = coordinates_to_index(icr.x, icr.y, len_x);
				x->h1 = angle_change(x->index)[4];
				x->h2 = max_rotation(x->index, angle);
				x->h3 = distance_from_centre(x->index);
				x->h4 = (x->h1*h_steer + x->h2*h_max_rot + x->h3*h_icr_dist)/(abs(h_icr_dist) + abs(h_max_rot) + abs(h_steer));
				x->wheel_angles = angle_change(x->index);
				icr_nodes.push_back(*x);
				icr_points << std::to_string(icr.x) + "," + std::to_string(icr.y) + "," + std::to_string(x->h4) + "\n"; // write to file
				delete x;
			}
			icr_points.close(); // close file
			std::sort(icr_nodes.begin(), icr_nodes.end(), sort_h); // sort icrs according to objective function
			std::cout << "h1: " << icr_nodes[0].h1 << std::endl;
			std::cout << "h2: " << icr_nodes[0].h2 << std::endl;
			std::cout << "h3: " << icr_nodes[0].h3 << std::endl;
			std::cout << "h4: " << icr_nodes[0].h4 << std::endl;

			return icr_nodes[0]; // return best icr
		}
		
		static bool sort_h(const ICR& a, const ICR& b)
		{	
			// sort according to objective function
			return (a.h4 >= b.h4);
		}

		// subscriber for angle to turn
		void rotation_angle(const std_msgs::Float64& theta)
		{
			angle = theta.data;
			//std::cout << footprint_points.size() << std::endl;
			received_angle = true;
		}

		// robot width subscriber
		void widthCallback(const geometry_msgs::Twist& msg)
		{
			ws_width = (msg.angular.y + msg.angular.z)/2;
		}

		// occupancy grid subscriber
		void mapCallback(const nav_msgs::OccupancyGrid& msg)
		{	
			len_x = msg.info.width;
			len_y = msg.info.height;
			res = msg.info.resolution;
			if (n == 0)
			{
				cornerPoints();
				n += 1;
			}
			data_pts = msg.data;
			/**
			ofstream myfile;
			myfile.open("example.txt", ios::out | ios::binary);
			myfile << "received map";
			myfile.close();
			**/
			if (received_angle == true) // if received angle to rotate
			{
				//test_run();
				//run();
				if (gradient_descent == true)
				{
					run(); // running on gradient descent, set in param file
				}
				else
				{
					global_run(); // brute force global max
				}
				received_angle = false;
			}
		}

		void cornerPoints()
		{	
			// coordinates of footprint corners
			geometry_msgs::Point32 lb, lf, rb, rf;
			lb.x = std::floor(len_x/2 - length/res/2);
			lb.y = std::ceil(len_y/2 + width/res/2);

			lf.x = std::ceil(len_x/2 + (length/res)/2);
			lf.y = std::ceil(len_y/2 + (width/res)/2);

			rb.x = std::floor(len_x/2 - (length/res)/2);
			rb.y = std::floor(len_y/2 - (width/res)/2);

			rf.x = std::ceil(len_x/2 + (length/res)/2);
			rf.y = std::floor(len_y/2 - (width/res)/2);
			footprint.push_back(lb);
			footprint.push_back(lf);
			footprint.push_back(rf);
			footprint.push_back(rb);

			// get robot cells
			for (int j=rb.y; j<= lb.y; j++)
			{	
				for (int i=lb.x; i<=lf.x; i++)
				{
					geometry_msgs::Point32 pt;
					pt.x = i;
					pt.y = j;
					footprint_points.push_back(pt);
				}
			}


			// wheel points
			geometry_msgs::Point32 l_b, l_f, r_b, r_f;
			l_b.x = len_x/2 - ws_length/res/2;
			l_b.y = len_y/2 + ws_width/res/2;

			l_f.x = len_x/2 + ws_length/res/2;
			l_f.y = len_y/2 + ws_width/res/2;

			r_b.x = len_x/2 - ws_length/res/2;
			r_b.y = len_y/2 - ws_width/res/2;

			r_f.x = len_x/2 + ws_length/res/2;
			r_f.y = len_y/2 - ws_width/res/2;

			wheels = {l_b, r_b, l_f, r_f};
			
			for (auto w : wheels)
			{
				std::cout << w << std::endl;
			}
			
		}

		void outlinepolygon(std::vector<geometry_msgs::Point32>* polygon) // inputs vector of point32 (map coordinates)
		{	
			//printf("outlining polygon\n");
			//std::vector<geometry_msgs::Point32> outline = polygon;

			// outline polygon, input corner points
			polygon->push_back(polygon->at(0));
			//std::cout << "outline size: " << outline.size()-1 << std::endl;
			for (int i=0; i < 4; i++)
			{	
				// get cells forming line between 2 points
				int x0,x1,y0,y1;
				//std::cout << i << std::endl;

				// organise x points for convenient search
				if (polygon->at(i).x > polygon->at(i+1).x)
				{
					x0 = polygon->at(i+1).x;
					x1 = polygon->at(i).x;
				}
				else
				{	
					x0 = polygon->at(i).x;
					x1 = polygon->at(i+1).x;
				}

				// organise y points for convenient search
				if (polygon->at(i).y > polygon->at(i+1).y)
				{
					y0 = polygon->at(i+1).y;
					y1 = polygon->at(i).y;
				}
				else
				{	
					y0 = polygon->at(i).y;
					y1 = polygon->at(i+1).y;
				}
				//std::cout << x0 << ' ' << x1 << ' ' << y0 << ' ' << y1 << std::endl;

				// check if points are near line connecting the 2 points
				for (int x=x0; x<=x1; x++)
				{
					for (int y=y0; y<=y1; y++)
					{
						// distance 
						double d = distanceToLine(x, y, polygon->at(i).x, polygon->at(i).y, polygon->at(i+1).x, polygon->at(i+1).y) * res;
						//std::cout << d << std::endl;
						if (d <= res)
						{
							// if cell is near line connecting the 2 corner points, add point to outline
							geometry_msgs::Point32 pt;
							pt.x = x;
							pt.y = y;
							polygon->push_back(pt);
						}
					}
				}
			}
			std::sort(polygon->begin(), polygon->end(), sort_xy); // sort points according to increasing y and then according to x for same y
			polygon->erase(unique(polygon->begin(), polygon->end()), polygon->end()); // make sure no duplicates
			//printf("sorted list\n");
			//return outline;
		}

		double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1)
		{	
			// calculate distance from point (pX, pY) to line joining (x0,y0) and (x1,y1)
			double A = pX - x0;
			double B = pY - y0;
			double C = x1 - x0;
			double D = y1 - y0;
			double dot = A * C + B * D;
			double len_sq = C * C + D * D;
			double param = dot / len_sq;
			double xx,yy;
			if(param < 0)
			{
				xx = x0;
				yy = y0;
			}
			else if(param > 1)
			{
				xx = x1;
				yy = y1;
			}
			else
			{
				xx = x0 + param * C;
				yy = y0 + param * D;
			}
			return distance(pX,pY,xx,yy);
		}

		double double_dist(double x0, double y0, double x1, double y1)
		{	
			// get distance between 2 points
			return sqrt(pow(x0 - x1,2) + pow(y0 - y1,2));
		}

		double distance(int x0, int y0, int x1, int y1)
		{	
			// get distance between 2 points, input is int
			return sqrt(pow(x0 - x1,2) + pow(y0 - y1,2));
		}

		std::vector<int> fill_polygon_outline(std::vector<geometry_msgs::Point32> outline) // sorted vector according to y value
		{	
			//printf("Filling polygon outline...\n");
			std::vector<int> filled_cells;
			for (int i=0; i<(int)(outline.size()-1); i++)
			{
				int current = coordinates_to_index((outline[i]).x, (outline[i]).y, len_x);
				filled_cells.push_back(current);
				if ((outline[i]).y == (outline[i+1]).y && (outline[i].x != outline[i+1].x)) // for cells with same y coordinates, fill cells from first cell on left to last cell on right
				{
					int c = (int)(outline[i].x);
					while (c != (int)(outline[i+1].x))
					{	
						filled_cells.push_back(coordinates_to_index(c, outline[i].y, len_x));
						if (c > (int)(outline[i+1].x)) // if somehow next point is to left of current point
						{
							c--;
						}
						else // if point is to the right of current point
						{
							c++;
						}
						//printf("While loop\n");
						//std::cout << "c: " << c << ", outline[i+1].x: " << outline[i+1].x << std::endl;
					}
				}
			}
			//printf("Filled polygon outline\n");
			return filled_cells;
		}

		static bool sort_y(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b)
		{	
			// sort according to y value
			return (a.y < b.y);
		}

		static bool sort_xy(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b)
		{	
			// sort according to increasing y and increasing x coordinates
			if (a.y == b.y)
			{
				if (a.x < b.x)
				{
					return true;
				}
				else
				{
					return false;
				}
			}
			else if (a.y < b.y)
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		std::vector<geometry_msgs::Point32> sector_footprint(geometry_msgs::Point32 icr, geometry_msgs::Point32 corner, double theta) // angle in radians
		{	
			// rotate corner by theta radians which is the angle to robot turns and get the outline of the sector to get search area for when robot rotates
			std::vector<geometry_msgs::Point32> footp{icr, corner};
			int interval = 5; // number of points on arc
			double radius = distance(icr.x, icr.y, corner.x, corner.y);
			double d_angle = theta/interval;

			// add point on arc every interval
			for (int i=1; i<=interval; i++)
			{	
				geometry_msgs::Point32 pt = rotate_pt(corner, icr, i*d_angle);
				footp.push_back(pt);
			}
			return footp; // outline
		}

		geometry_msgs::Point32 rotate_pt(geometry_msgs::Point32 pt, geometry_msgs::Point32 icr, double gamma)
		{	
			// new corner of footprint when robot rotates
			geometry_msgs::Point32 new_corner;
			new_corner.x = (pt.x - icr.x)*cos(gamma) - (pt.y - icr.y)*sin(gamma);
			new_corner.y = (pt.x - icr.x)*sin(gamma) + (pt.y - icr.y)*cos(gamma);

			new_corner.x += icr.x;
			new_corner.y += icr.y;

			new_corner.x = std::round(new_corner.x);
			new_corner.y = std::round(new_corner.y);

			return new_corner;
		}

		std::vector<geometry_msgs::Point32> new_fp(geometry_msgs::Point32 icr, double theta)
		{	
			// get new footprint when robot rotates
			std::vector<geometry_msgs::Point32> fp;
			for (int i=0; i<4; i++)
			{
				fp.push_back(rotate_pt(footprint[i], icr, theta));
				//std::cout << rotate_pt(footprint[i], icr, theta) << std::endl;
			}
			return fp;
		}

		void concat_search_area(std::vector<int>* fp, std::vector<int> c1, std::vector<int> c2, std::vector<int> c3, std::vector<int> c4)
		{	
			// merge search areas to get full search area 
			fp->insert(fp->end(), c1.begin(), c1.end());
			fp->insert(fp->end(), c2.begin(), c2.end());
			fp->insert(fp->end(), c3.begin(), c3.end());
			fp->insert(fp->end(), c4.begin(), c4.end());
			fp->erase(unique(fp->begin(), fp->end()), fp->end()); // remove duplicate points
			//return search_area;
		}

		bool rotation_clear(geometry_msgs::Point32 icr, double theta, std::vector<signed char>& cmap)
		{
			// get new footprint
			std::vector<geometry_msgs::Point32> rotated_fp = new_fp(icr, theta);
			//std::vector<geometry_msgs::Point32> rotated_fp = outlinepolygon(&new_corners);
			outlinepolygon(&rotated_fp);
			
			std::vector<int> filled_rotated_fp = fill_polygon_outline(rotated_fp);
			//std::cout << "new fp cells: " << filled_rotated_fp.size() << std::endl;

			// get sector cells
			std::vector<std::vector<int>> sectors(4);
			for (int i=0; i<footprint.size(); i++)
			{
				sectors[i] = fill_polygon_outline(sector_footprint(icr, footprint[i], theta)); // fill sector footprints
			}
			concat_search_area(&filled_rotated_fp, sectors[0], sectors[1], sectors[2], sectors[3]); // get full search area
			
			for (auto i : filled_rotated_fp)
			{	
				if (cmap[i] > 0)
				{	
					return false; // check if icr is feasible
				}
			}
			return true;
		}

		bool withinWheelBase(geometry_msgs::Point32 icr)
		{	
			// check if icr is within the wheel base
			geometry_msgs::Point32 lb_, lf_, rb_, rf_;
			lb_ = wheels[0];
			rb_ = wheels[1];
			lf_ = wheels[2];
			rf_ = wheels[3];
			if (icr.x < lb_.x || icr.x > lf_.x || icr.y > lb_.y || icr.y < rb_.y)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "icr_search_node");
	ros::NodeHandle nh;

	Robot Panthera = Robot(&nh);
	ros::spin();
	return 0;
}