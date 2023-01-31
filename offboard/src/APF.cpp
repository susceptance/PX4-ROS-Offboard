#include <ros/ros.h>
#include <iostream>
#include <complex>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "offboard/coordinate.h"
#include "offboard/coordinate_array.h"
#include "mqtt_json2/drone_gps.h"
#include "mqtt_json2/user_gps.h"


double userX = 0, userY = 0, homeX = 0, homeY = 0;
bool detection_flag = false;

geometry_msgs::PoseStamped current_pose;
mavros_msgs::State current_state;
sensor_msgs::NavSatFix curr_drone_gps;
mqtt_json2::drone_gps drone_gps;
mqtt_json2::user_gps user_gps;
offboard::coordinate coordinate;
offboard::coordinate_array coordinates;


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}


void ekf_odom_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
 	//ROS_INFO("[odom_pos x, y, z]: %5.2f %5.2f %5.2f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);   
	current_pose = *msg;
}


void coordinate_cb(const offboard::coordinate::ConstPtr& msg)
{
	//ROS_INFO("Distance : %f", msg->coordinate);
	//ROS_INFO("goal_x : %f", msg->goal_x);
	//ROS_INFO("goal_y : %f", msg->goal_y);
	//ROS_INFO("goal_z : %f", msg->goal_z);
	//obj_x = msg->world_x;
	//obj_y = msg->world_y;
	//obj_z = msg->world_z;
	//obj_pr = msg->probability;
	//obj_name = msg->bbox_class;
	detection_flag = msg->detection_flag;
}


void coordinates_cb(const offboard::coordinate_array::ConstPtr& msg)
{
	/*for(int i = 0; i < msg->coordinates.size(); i++)
	{
		obj_x[i] = msg->coordinates[i].world_x;
		obj_y[i] = msg->coordinates[i].world_y;
		obj_z[i] = msg->coordinates[i].world_z;
	}*/

	coordinates = *msg;
}


void drone_gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	curr_drone_gps = *msg;

	static int count = 0;
	if(count > 10)
		return;

	int lon_deg, lon_min, lat_deg, lat_min;
	double lon_sec, lat_sec;
	double C, D;
	double degree;

	//128.3937 lon
	//36.146 at

	lon_deg = msg->longitude;
	lon_min = (msg->longitude-lon_deg)*60;
	lon_sec = ((msg->longitude-lon_deg)*60-lon_min) * 60;
	lat_deg = msg->latitude;
	lat_min = (msg->latitude-lat_deg)*60;
	lat_sec = ((msg->latitude-lat_deg)*60-lat_min) * 60;
	//std::cout << lon_deg << "  " << lon_min << "  " << lon_sec << std::endl;

	degree = (lat_deg+lat_deg) / 2;
	C = cos(degree*M_PI/180) * (2*M_PI*6371 / 360);
	D = (2*M_PI*6371 / 360);
	homeX = (lon_deg*C) + (lon_min*(C/60)) + (lon_sec*((C/60)/60));
	homeY = (lat_deg*D) + (lat_min*(D/60)) + (lat_sec*((D/60)/60));

	std::cout << "homeX : " << homeX << ", homeY : " << homeY << std::endl;
	count++;
}

// user 위치정보 받음.
void user_gps_cb(const mqtt_json2::user_gps::ConstPtr& msg)
{
	user_gps = *msg;
	std::cout << "user_gps_callback : " << msg->user_flag << std::endl;

	int lon_deg, lon_min, lat_deg, lat_min;
	double lon_sec, lat_sec;
	double C, D;
	double degree;

	lon_deg = msg->longitude;
	lon_min = (msg->longitude-lon_deg)*60;
	lon_sec = ((msg->longitude-lon_deg)*60-lon_min) * 60;
	lat_deg = msg->latitude;
	lat_min = (msg->latitude-lat_deg)*60;
	lat_sec = ((msg->latitude-lat_deg)*60-lat_min) * 60;

	degree = (lat_deg+lat_deg) / 2;
	C = cos(degree*M_PI/180) * (2*M_PI*6371 / 360);
	D = (2*M_PI*6371 / 360);
	userX = (lon_deg*C) + (lon_min*(C/60)) + (lon_sec*((C/60)/60));
	userY = (lat_deg*D) + (lat_min*(D/60)) + (lat_sec*((D/60)/60));
	userX = userX*1000 - homeX*1000;
	userY = userY*1000 - homeY*1000;
	std::cout << "userX : " << userX << ", userY : " << userY << std::endl;
}


void User_GPS(float* gps_x, float* gps_y, float* gps_z)
{
	//set arbitary value, this will be real time updated
	*gps_x = userX;
	*gps_y = userY;
	*gps_z = 2;
}


void Attractive_Force(float &goal_x, float &goal_y, float &goal_z, float &px, float &py, float &pz, float &Att_x, float &Att_y, float &Att_z)
{
	const float K_Att = 0.2;
	float e_x, e_y, e_z;
	Att_x = 0, Att_y = 0, Att_z = 0;

	//std::cout << "goal_x = " << goal_x << ", goal_y = " << goal_y << std::endl;

	e_x = goal_x - px;
	e_y = goal_y - py;
	e_z = goal_z - pz;

	//L2 norm
	//std::complex<float> dist_xy (1.0, 2.0);
	//std::complex<float> dist_z (sqrt(std::norm(dist_xy)), 3.0);
	//std::cout << sqrt(std::norm(dist_z)) << std::endl;
	std::complex<float> dist_xy (e_x, e_y);
	std::complex<float> dist_z (sqrt(std::norm(dist_xy)), e_z);

	//std::cout << e_x << "  " << e_y << "  " << e_z << std::endl;
	//std::cout << sqrt(std::norm(dist_z)) << std::endl;

	Att_x = K_Att * e_x / sqrt(std::norm(dist_z));
	Att_y = K_Att * e_y / sqrt(std::norm(dist_z));
	Att_z = K_Att * e_z / sqrt(std::norm(dist_z));

	//Att_x = K_Att * e_x / sqrt(std::norm(dist_xy));
	//Att_y = K_Att * e_y / sqrt(std::norm(dist_xy));

	//std::cout << "Att_x : " << Att_x << ", Att_y : " << Att_y << ", Att_z : " << Att_z << std::endl;
}


void Repulsive_Force(float (&obs_x)[16], float (&obs_y)[16], float (&obs_z)[16], float &px, float &py, float &pz, float &Rep_x, float &Rep_y, float &Rep_z)
{	
	const float K_Rep = 3;
	const float obs_bound = 6;
	float obs_dist_x[16] = {0,}, obs_dist_y[16] = {0,}, obs_dist_z[16] = {0,}, obs_dist[16] = {0,};
	Rep_x = 0, Rep_y = 0, Rep_z = 0;

	for(int i = 0; i < 16; i++)
	{
		if(obs_x[i] == 0 || obs_y[i] == 0 || obs_z[i] == 0) 
		{
			//std::cout << "0 value skip" << std::endl; 
			continue; 
		}

		obs_dist_x[i] = obs_x[i] - px;
		obs_dist_y[i] = obs_y[i] - py;
		obs_dist_z[i] = obs_z[i] - pz;

		std::complex<float> dist_xy (obs_dist_x[i], obs_dist_y[i]);
		std::complex<float> dist_z (sqrt(std::norm(dist_xy)), obs_dist_z[i]);
		obs_dist[i] = sqrt(std::norm(dist_z));
		//obs_dist[i] = sqrt(std::norm(dist_xy));
		//std::cout << i << ". --- obs_dist : " << obs_dist[i] << std::endl; 

		if(obs_dist[i] < obs_bound)
		{
			Rep_x = Rep_x - K_Rep * (1 / obs_dist[i] - 1 / obs_bound) * (1 / (obs_dist[i] * obs_dist[i])) * obs_dist_x[i] / obs_dist[i];
			Rep_y = Rep_y - K_Rep * (1 / obs_dist[i] - 1 / obs_bound) * (1 / (obs_dist[i] * obs_dist[i])) * obs_dist_y[i] / obs_dist[i];
			Rep_z = Rep_z - K_Rep * (1 / obs_dist[i] - 1 / obs_bound) * (1 / (obs_dist[i] * obs_dist[i])) * obs_dist_z[i] / obs_dist[i];
		}
		else
		{	
			Rep_x = Rep_x;
			Rep_y = Rep_y;
			Rep_z = Rep_z;
		}

		//obs_x[i] = 0;
		//obs_y[i] = 0;
		//obs_z[i] = 0;

		//std::cout << i << ". --- Rep_x : " << Rep_x << ", Rep_y : " << Rep_y << ", Rep_z : " << Rep_z << std::endl;
	}
}


int main(int argc, char** argv)
{
	std::string obj_str;

	ros::init(argc, argv, "APF_Algorithm");
	ros::NodeHandle nh;

	ros::Subscriber coordinate_sub = nh.subscribe("coordinate", 1, coordinate_cb);
	ros::Subscriber coordinates_sub = nh.subscribe("coordinates", 1, coordinates_cb);
	ros::Subscriber user_gps_sub = nh.subscribe("/user_gps_pos", 10, user_gps_cb); // 사용자로부터 정보 받아옴
	ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, ekf_odom_cb);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
	ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/raw/fix", 10, drone_gps_cb);
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
	ros::Publisher drone_gps_pub = nh.advertise<mqtt_json2::drone_gps>("/drone_gps", 10); // 사용자로 전송
	//ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
	//ros::Publisher mav_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

	ros::Rate rate(20.0);

	// wait for FCU connection
	while(ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}

	//uint16 IGNORE_PX=1
	//uint16 IGNORE_PY=2
	//uint16 IGNORE_PZ=4
	//uint16 IGNORE_VX=8
	//uint16 IGNORE_VY=16
	//uint16 IGNORE_VZ=32
	//uint16 IGNORE_AFX=64
	//uint16 IGNORE_AFY=128
	//uint16 IGNORE_AFZ=256
	//uint16 FORCE=512
	//uint16 IGNORE_YAW=1024
	//uint16 IGNORE_YAW_RATE=2048

	//mavros_msgs::PositionTarget cmd_mav;
	//cmd_mav.header.stamp = ros::Time::now();
	//cmd_mav.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	//cmd_mav.header.frame_id = "Drone";
	//cmd_mav.type_mask = 0b011111000111;

	//geometry_msgs::TwistStamped cmd_vel;
	geometry_msgs::Quaternion yaw_value;
	geometry_msgs::PoseStamped cmd_pose;
	cmd_pose.pose.position.x = 0;
	cmd_pose.pose.position.y = 0;
	cmd_pose.pose.position.z = 3;

	//send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i)
	{
		local_pos_pub.publish(cmd_pose);
		ros::spinOnce();
		rate.sleep();
	}

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	mavros_msgs::CommandTOL land;

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();
	ros::Time last_comeback = ros::Time::now();
	ros::Time last_scan = ros::Time::now();
	ros::Time last_drone_gps = ros::Time::now();


	float roll = 0, pitch = 0, yaw = 0;
	float px = 0, py = 0, pz = 2; // this parameters are used to command pose
	float goal_x, goal_y, goal_z;
	float gps_x, gps_y, gps_z;  
	float obs_x[16] = {0,}, obs_y[16] = {0,}, obs_z[16] = {0,};
	float tmp_obs_x[16] = {0,}, tmp_obs_y[16] = {0,}, tmp_obs_z[16] = {0,};
	float world[3][1] = {0, 0, 0};
	float goal_cam[3][1] = {0, 0, 0};
	float obs_cam[3][1] = {0, 0, 0};
	float rotation_z[3][3] =   {{cos(yaw), -sin(yaw), 0},
									{sin(yaw), cos(yaw), 0},
									{0, 0, 1}};
	float Att_x = 0, Att_y = 0, Att_z = 0;
	float Rep_x = 0, Rep_y = 0, Rep_z = 0;
	float APF_x = 0, APF_y = 0, APF_z = 0;
	bool GPS_mission = false;
	bool flight = false;
	bool Control_pose = false;
	bool cam_scan = false;
	bool object_detect = false;
	bool user_detect = false;
	bool goal_detect = false;
	bool mission_flag = false; // 사용자에게 전송
	bool return_flag = false; 
	bool return_home_flag = false;
	bool arm_flag = false;


	int delivery = 3;
	//0:배송 요청(사용자로부터 수신)
	//1:배송 중
	//2:배송 확인 완료
	//3:return to home(노드 내부에서만 사용)

	while(ros::ok())
	{
		if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
		{	// 5초마다 한 번씩 체크 
			if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
			{ 	// 서비스 요청하여 성공한 경우 
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		} 
		else 
		{
			if( !current_state.armed &&	(ros::Time::now() - last_request > ros::Duration(3.0)))
			{
				if( arming_client.call(arm_cmd) &&	arm_cmd.response.success)
				{
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}

		ros::spinOnce();
		delivery = user_gps.user_flag;
		local_pos_pub.publish(cmd_pose);
		
		if(ros::Time::now() - last_drone_gps > ros::Duration(0.5))
		{
			std::cout << "user_flag : " << delivery << std::endl;
			std::cout << "drone_flag : " << mission_flag << std::endl;
			drone_gps.latitude = curr_drone_gps.latitude;
			drone_gps.longitude = curr_drone_gps.longitude;
			drone_gps.drone_flag = mission_flag;
			//std::cout << "mssion_flag : " << mission_flag << std::endl;
			drone_gps_pub.publish(drone_gps);
			last_drone_gps = ros::Time::now();
		}


		if(delivery == 0)
		{
			px = 0;
			py = 0;
			pz = 3;
		}

		if(current_state.armed && current_pose.pose.position.z >= 2 && !Control_pose && delivery == 1)
		{
			GPS_mission = true;
			pz = 2;
			ROS_INFO("Start GPS mission");
		}

		if(delivery == 2 && !return_home_flag)
		{
			pz = 10;
			return_flag = true;
		}

		if(return_flag)
		{
			return_flag = false;
			return_home_flag = true;
			last_comeback = ros::Time::now();
		}

		if(return_home_flag && ros::Time::now() - last_comeback > ros::Duration(5.0))
		{
			px = 0;
			py = 0;
			yaw = atan2(py, px);
		}


		if(GPS_mission)
		{
			User_GPS(&gps_x, &gps_y, &gps_z);
			//ROS_INFO("Get User GPS position");
			cam_scan = true;
			Control_pose = true;
			flight = true;
			object_detect = true;
			yaw = atan2(gps_y, gps_x);
		}


		if(cam_scan)
		{
			//std::cout << "object num : " << coordinates.coordinates.size() << std::endl

			for(int i = 0; i < coordinates.coordinates.size(); i++)
			{
				if(i == 16)
				{
					std::cout << "Bounding Boxes number over!" << std::endl;
					continue;
				}

				if(coordinates.coordinates[i].cam_x == 0 || coordinates.coordinates[i].cam_y == 0 
					|| coordinates.coordinates[i].cam_z == 0 || !detection_flag)
					continue;

				//std::cout << "Detected object : " << coordinates.coordinates[i].bbox_class << std::endl;

				if(coordinates.coordinates[i].bbox_class == "person" && user_detect)
				{
					//ROS_INFO("User was detected!");
					goal_x = coordinates.coordinates[i].cam_x;
					goal_y = coordinates.coordinates[i].cam_y;
					goal_z = coordinates.coordinates[i].cam_z;
					//std::cout << "goal_x = " << goal_x << ", goal_y = " << goal_y << std::endl;
					goal_detect = true;
				}

				if(!(coordinates.coordinates[i].bbox_class == "person"))
				{
					//ROS_INFO("Obstacle was detected!");
					tmp_obs_x[i] = coordinates.coordinates[i].cam_x;
					tmp_obs_y[i] = coordinates.coordinates[i].cam_y;
					tmp_obs_z[i] = coordinates.coordinates[i].cam_z;
				}
			}

			for(int i = 0; i < 15; i++)
			{
				if(tmp_obs_x[i] == 0 || tmp_obs_y[i] == 0 || tmp_obs_z[i] == 0) { continue; }

				for(int j = i; j < 15; j++)
				{
					if(abs(tmp_obs_x[i] - tmp_obs_x[j+1]) <= 0.5 && abs(tmp_obs_y[i] - tmp_obs_y[j+1]) && abs(tmp_obs_z[i] - tmp_obs_z[j+1]) <= 0.5)
					{
						tmp_obs_x[j+1] = 0;
						tmp_obs_y[j+1] = 0;
						tmp_obs_z[j+1] = 0;
					}
				}
			}
		}

		
		if(object_detect)
		{
			/*for(int i = 0; i < coordinates.coordinates.size(); i++)
			{
				if(coordinates.coordinates[i].cam_x <= 6 && coordinates.coordinates[i].cam_y >= 1.5)
				{
					//yaw = yaw + 2; 
					yaw = atan2(coordinates.coordinates[i].cam_y, coordinates.coordinates[i].cam_x)*180 / M_PI; 
					last_yaw_scan = ros::Time::now();
				}
				else if(coordinates.coordinates[i].cam_x <= 6 && coordinates.coordinates[i].cam_y <= -1.5)
				{
					//yaw = yaw - 2;
					yaw = atan2(coordinates.coordinates[i].cam_y, coordinates.coordinates[i].cam_x)*180 / M_PI;
					last_yaw_scan = ros::Time::now();
				}
			}*/

			
			//std::cout << "Degree yaw : " << yaw*180 / M_PI << std::endl;

			float rotation_z[3][3] = {{cos(yaw), -sin(yaw), 0},
										{sin(yaw), cos(yaw), 0},
										{0, 0, 1}};

			for(int obs = 0; obs < 16; obs++)
			{
				if(tmp_obs_x[obs] == 0 || tmp_obs_y[obs] == 0 || tmp_obs_z[obs] == 0 || !detection_flag) { continue; }

				obs_cam[0][0] = tmp_obs_x[obs];
				obs_cam[1][0] = tmp_obs_y[obs];
				obs_cam[2][0] = tmp_obs_z[obs];

				for(int i = 0; i < 3; i++)
				{
					for(int j = 0; j < 1; j++)
					{
						float tmp = 0;

						for(int k = 0; k < 3; k++)
						{
							tmp = tmp + rotation_z[i][k]*obs_cam[k][j];
							//std::cout << tmp << std::endl;	
						}   

						if(i == 0)
							obs_x[obs] = tmp;
						else if(i == 1)
							obs_y[obs] = tmp;
						else if(i == 2)
							obs_z[obs] = tmp;  			
					}

				}
			}

			for(int i = 0; i < 16; i++)
			{
				if(tmp_obs_x[i] == 0 || tmp_obs_y[i] == 0 || tmp_obs_z[i] == 0 || !detection_flag) { continue; }

				obs_x[i] = obs_x[i] + px;
				obs_y[i] = obs_y[i] + py;
				obs_z[i] = obs_z[i] + pz;
			}
		}


		if(user_detect && goal_detect)	
		{
			float rotation_z[3][3] = {{cos(yaw), -sin(yaw), 0},
										{sin(yaw), cos(yaw), 0},
										{0, 0, 1}};

			goal_cam[0][0] = goal_x;
			goal_cam[1][0] = goal_y;
			goal_cam[2][0] = goal_z;

			//std::cout << "goal_cam : " << goal_cam[0][0] << " " << goal_cam[1][0] << " " << goal_cam[2][0] << std::endl;

			for(int i = 0; i < 3; i++)
			{
				for(int j = 0; j < 1; j++)
				{
					float tmp = 0;

					for(int k = 0; k < 3; k++)
					{
						tmp = tmp + rotation_z[i][k]*goal_cam[k][j];
						//std::cout << tmp << std::endl;			
					}   

					if(i == 0)
						goal_x = tmp;
					else if(i == 1)
						goal_y = tmp;
					else if(i == 2)
						goal_z = tmp;
				}
			}

			goal_x = goal_x + px;
			goal_y = goal_y + py;
			goal_z = goal_z + pz;
			std::cout << "rotated goal : " << goal_x << "  " << goal_y << "  " << goal_z << std::endl;
		}


		if(flight && ros::Time::now() - last_scan > ros::Duration(0.1))
		{
			if(GPS_mission)
			{
				goal_x = gps_x;
				goal_y = gps_y;
				goal_z = gps_z;
			}

			Attractive_Force(goal_x, goal_y, goal_z, px, py, pz, Att_x, Att_y, Att_z);
			Repulsive_Force(obs_x, obs_y, obs_z, px, py, pz, Rep_x, Rep_y, Rep_z);

			yaw = atan2(goal_y, goal_x);

			if(user_detect && goal_detect)
			{
				std::complex<float> dist_xy (goal_x-px, goal_y-py);
				//std::complex<float> dist_z (sqrt(std::norm(dist_xy)), goal_z-pz);
				if(sqrt(std::norm(dist_xy)) >= 4.5 && sqrt(std::norm(dist_xy)) <= 5.5)
				{
					Att_x = -Att_x;
					Att_y = -Att_y;
					//Att_z = -Att_z;
					pz = 1;
					mission_flag = true;
					//flight = false;
				}
				else if(sqrt(std::norm(dist_xy)) < 4.5)
				{
					Att_x = -Att_x;
					Att_y = -Att_y;
					//Att_z = -Att_z;
					pz = 1;
				}

				goal_detect = false;
			}

			if(mission_flag)
			{
				user_detect = false;
				goal_detect = false;
				flight = false;
				ROS_INFO("Mission completed!");
			}

			APF_x = Att_x + Rep_x; 
			APF_y = Att_y + Rep_y; 
			APF_z = Att_z + Rep_z;

			px = px + APF_x;
			py = py + APF_y;
			pz = pz + APF_z;
			//std::cout << "px : " << px << ", py : " << py << ", pz : " << pz << std::endl;

			if(pz > 10)
				pz = 10;

			if((px >= gps_x-10 && px <= gps_x+10) && (py >= gps_y-10 && py <= gps_y+10))
			{
				ROS_INFO("Start searching the User");
				user_detect = true;
				GPS_mission = false;
			}

			last_scan = ros::Time::now();
		}


		if(Control_pose)
		{
			yaw_value = tf::createQuaternionMsgFromYaw(yaw);

			cmd_pose.pose.position.x = px;
			cmd_pose.pose.position.y = py;
			cmd_pose.pose.position.z = pz;

			//cmd_pose.pose.orientation.x = qx;
			//cmd_pose.pose.orientation.y = qy;
			cmd_pose.pose.orientation.z = yaw_value.z;
			cmd_pose.pose.orientation.w = yaw_value.w;

		}

		rate.sleep();
	}

	return 0;
}


