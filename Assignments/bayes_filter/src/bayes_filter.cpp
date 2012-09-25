/* Autonomous Systems : Bayes filter
// For this assignment you are going to implement a bayes filter

- uniformly distibute the belief over all beliefstates
- implement a representation of the world for every beliefstate
- implement the measurement model
- implement the motion model (forward & turn)
 */

#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "laser_to_wall/WallScan.h"
#include "std_msgs/Int32.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include <boost/lexical_cast.hpp>
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <sstream>


class BayesFilter {


public:
	// Construst a new BayesFilter object and hook up this ROS node
	// to the simulated robot's velocity control and laser topics
	BayesFilter(ros::NodeHandle& nh) : rotateStartTime(ros::Time::now()),rotateDuration(1.8f), moveDuration(0.75f) {
		// Initialize random time generator
		srand(time(NULL));
		// Advertise a new publisher for the simulated robot's velocity command topic
		// (the second argument indicates that if multiple command messages are in
		//  the queue to be sent, only the last command will be sent)
		commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		// Subscribe to the simulated robot's wall scan topic and tell ROS to call
		// this->commandCallback() whenever a new message is published on that topic
		wallSub = nh.subscribe("wall_scan", 1, &BayesFilter::commandCallbackWallScan, this);
		actionSub = nh.subscribe("action", 1, &BayesFilter::commandCallbackAction, this);
		markerPub = nh.advertise<visualization_msgs::MarkerArray>("beliefs",1);
		movenoise = false;
		measnoise = false;

		/*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
		// Initial belief distribution
		for (int i = 0; i<NUM_STATES; i++) { 
			beliefStates.push_back(1.0/NUM_STATES); 
			mapDescription[i][0] = true;
			mapDescription[i][1] = true;
			mapDescription[i][2] = false;
		};

		mapDescription[1][0] = false;
		mapDescription[3][1] = false;
		mapDescription[7][0] = false;
		mapDescription[9][2] = true;
		mapDescription[12][1] = false;
		mapDescription[16][0] = false;
		mapDescription[18][1] = false;
		mapDescription[19][2] = true;



		/*============================================*/
	}; 



	// publish visual information to RVIZ of the beliefstates 
	void publishBeliefMarkers() {
		visualization_msgs::MarkerArray beliefs;
		for (int i = 0; i < NUM_STATES; i++) {
			visualization_msgs::Marker marker;	
			marker.header.frame_id = "/map";
			marker.header.stamp = ros::Time();
			marker.ns = "beliefs";
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::ADD;
			if (i >= 10) {	
				marker.pose.position.x = -0.8;
				marker.pose.position.y =  4.5 -i%10;
			}	
			else{
				marker.pose.position.x = 0.8;
				marker.pose.position.y = -4.5 +i;
			}
			marker.pose.position.z = 0.2;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.5;
			marker.scale.y = 1.0;
			marker.scale.z = 1.0;
			// Set the color -- be sure to set alpha to something non-zero!
			marker.color.r = 1.0f;
			marker.color.g = 0.0f;
			marker.color.b = 0.0f;
			marker.color.a = 1.0  * beliefStates[i]		;
			marker.id = i;
			beliefs.markers.push_back(marker); 

			//Text
			visualization_msgs::Marker marker2;
			marker2.header.frame_id = "/map";
			marker2.header.stamp = ros::Time();
			marker2.ns = "beliefs";
			marker2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			marker2.action = visualization_msgs::Marker::ADD;
			if (i >= 10) {	
				marker2.pose.position.x = -0.8;
				marker2.pose.position.y =  4.5 -i%10;
			}	
			else{
				marker2.pose.position.x = 0.8;
				marker2.pose.position.y = -4.5 +i;
			}
			marker2.pose.position.z = 0.2;
			marker2.pose.orientation.x = 0.0;
			marker2.pose.orientation.y = 0.0;
			marker2.pose.orientation.z = 0.0;
			marker2.pose.orientation.w = 1.0;
			marker2.scale.x = 0.5;
			marker2.scale.y = 1.0;
			marker2.scale.z = 0.15;
			// Set the color -- be sure to set alpha to something non-zero!
			marker2.color.r = 1.0f;
			marker2.color.g = 1.0f;
			marker2.color.b = 1.0f;
			marker2.color.a = 1.0;
			//std::string text = boost::lexical_cast<string>(i);
			std::ostringstream oss;
			oss << i;	
			std::ostringstream oss2;
			oss2 << beliefStates[i];	
			marker2.text = "State: " + oss.str() + "\nBelief:\n" + oss2.str();
			marker2.id = NUM_STATES + i;
			beliefs.markers.push_back(marker2);    
		}
		markerPub.publish(beliefs);
	};



	/*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
	void updateMove() {
		std::vector<double> beliefStatesNon;

		for (int i = 0; i<NUM_STATES; i++) {
			//Calculating our temporary belief
			double tempBelief=0.1*beliefStates[i];
			if(i>0 && i != 10 && i != 11){
				tempBelief+=0.8*beliefStates[i-1];
				if(i>1 && i != 11){
					tempBelief+=0.1*beliefStates[i-2];
				}
			}
			
			beliefStatesNon.push_back(tempBelief);
		}

		double sum=0;
		for (int i = 0; i<NUM_STATES; i++){
			sum+=beliefStatesNon[i];
		}
		for (int i = 0; i<NUM_STATES; i++){
			beliefStates[i]=beliefStatesNon[i]*(1.0/sum);
		}
	};				


	/*==========================================*/




	/*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
	void updateTurn() {
		std::vector<double> beliefStatesNon;

		for (int i = 0; i<NUM_STATES; i++) {
			//Calculating our temporary belief
			double tempBelief=0.1*beliefStates[i];
			tempBelief +=0.9*beliefStates[19-i];

			
			beliefStatesNon.push_back(tempBelief);
		}

		double sum=0;
		for (int i = 0; i<NUM_STATES; i++){
			sum+=beliefStatesNon[i];
		}
		for (int i = 0; i<NUM_STATES; i++){
			beliefStates[i]=beliefStatesNon[i]*(1.0/sum);
		}
	};
	/*==========================================*/



	/*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
	void updateSensing() {
		// example routine, generates random beliefs
		std::vector<double> beliefStatesNon;

		for (int i = 0; i<NUM_STATES; i++) {
			//Calculating our temporary belief
			double tempBelief=beliefStates[i];

			//Calculating belief without n.
			double beliefNoN=1;

			if(wall_right && mapDescription[i][1]) {
				beliefNoN=0.8;
			} else if(wall_right && !mapDescription[i][1]) {
				beliefNoN=0.3;
			}else if(!wall_right && mapDescription[i][1]) {
				beliefNoN=0.2;
			} else if(!wall_right && !mapDescription[i][1]) {
				beliefNoN=0.7;
			}
			if(wall_left && mapDescription[i][0]) {
				beliefNoN*=0.8;
			} else if(wall_left && !mapDescription[i][0]) {
				beliefNoN*=0.3;
			}else if(!wall_left && mapDescription[i][0]) {
				beliefNoN*=0.2;
			} else if(!wall_left && !mapDescription[i][0]) {
				beliefNoN*=0.7;
			}

			if(wall_front && mapDescription[i][2]) {
				beliefNoN*=0.8;
			} else if(wall_front && !mapDescription[i][2]) {
				beliefNoN*=0.3;
			}else if(!wall_front && mapDescription[i][2]) {
				beliefNoN*=0.2;
			} else if(!wall_front && !mapDescription[i][2]) {
				beliefNoN*=0.7;
			}
			beliefNoN*=tempBelief;
			beliefStatesNon.push_back(beliefNoN);
		}

		double sum=0;
		for (int i = 0; i<NUM_STATES; i++){
			sum+=beliefStatesNon[i];
		}
		for (int i = 0; i<NUM_STATES; i++){
			beliefStates[i]=beliefStatesNon[i]*(1.0/sum);
		}
	}
	/*==========================================*/


	// Send a velocity command 
	void move(double linearVelMPS, double angularVelRadPS) {
		geometry_msgs::Twist msg; // The default constructor will set all commands to 0
		msg.linear.x = linearVelMPS;
		msg.angular.z = angularVelRadPS;
		commandPub.publish(msg);
	}

	// introduce discrete movement noise
	int movementNoise()
	{
		if (movenoise)
		{ 
			int val = rand()%100;
			if (val<LOWER_NOISE_THRESHOLD) return 0;
			if (val>=UPPER_NOISE_THRESHOLD) return 2;
		}
		return 1;
	}

	// Introduce measurement noise 
	bool measurementNoise(bool measurement)
	{
		if (measnoise)
		{    
			int val = rand()%100;
			if (measurement) {
				if (val>=80)
					return !measurement; 
			}
			else 
				if (val>=70) 
					return !measurement;
		}
		return measurement;
	}

	// Process the incoming action message
	void commandCallbackAction(const std_msgs::Int32::ConstPtr& msg) {
		int steps = movementNoise();
		if (msg->data == 0) {
			for (int i = 0; i<steps; i++) {
				if (!obstacle) {
					moveStartTime = ros::Time::now();
					while (ros::Time::now() - moveStartTime <= moveDuration) 
						move(FORWARD_SPEED_MPS,0);
				}
				ros::Duration(0.2).sleep();
			}
			updateMove();
		}
		if (msg->data == 1) {
			for (int i = 0; i<std::min(steps,1); i++) {
				rotateStartTime = ros::Time::now();
				while (ros::Time::now() - rotateStartTime <= rotateDuration) 
					move(0,ROTATE_SPEED_RADPS);
			}
			updateTurn();
		}
		if (msg->data == 2) {	
			updateSensing();
		}
		if (msg->data == 3) {
			if (movenoise==false) movenoise = true;
			else movenoise = false;
			ROS_INFO_STREAM("movementnoise: " << movenoise);
		}
		if (msg->data == 4) {
			if (measnoise==false) measnoise = true;
			else measnoise = false;
			ROS_INFO_STREAM("measurementnoise: " << measnoise);
		}
		publishBeliefMarkers();
	}

	// Process the incoming wall scan message
	void commandCallbackWallScan(const laser_to_wall::WallScan::ConstPtr& msg) {
		wall_left  =  measurementNoise((bool)msg->wall_left);
		wall_right =  measurementNoise((bool)msg->wall_right);
		wall_front =  measurementNoise((bool)msg->wall_front);
		obstacle =  (bool)msg->wall_front;
	}


protected:
	ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
	ros::Publisher markerPub;
	ros::Subscriber wallSub; // Subscriber to the simulated robot's wall scan topic
	ros::Subscriber actionSub; // Subscriber to the action topic
	ros::Time rotateStartTime; // Start time of the rotation
	ros::Duration rotateDuration; // Duration of the rotation
	ros::Time moveStartTime; // Start time of the rotation
	ros::Duration moveDuration; // Duration of the rotation
	std::vector<double> beliefStates;
	bool wall_front, wall_left, wall_right, movenoise, measnoise, obstacle;
	const static int NUM_STATES = 20;
	const static double FORWARD_SPEED_MPS = 1.0;
	const static double ROTATE_SPEED_RADPS = M_PI/2;
	const static int UPPER_NOISE_THRESHOLD = 90;
	const static int LOWER_NOISE_THRESHOLD = 10;
	/*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
	//describing walls for each state, in the order: left, right, front
	bool mapDescription[20][3]; 
	/*==========================================*/
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "bayes_filter"); // Initiate new ROS node named "bayes_filter"
	ros::NodeHandle n;
	BayesFilter filter(n); // Create new filter object
	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();
	//  ros::spin();
	return 0;
};
