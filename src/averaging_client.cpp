#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/AveragingAction.h>
#include <boost/thread.hpp>
 // This example program spins a thread,
 // creates an action client, and sends a goal to the action server.

/*
actionlib_tutorials Averaging.action file description
					**********************
					# goal definition
					int32 samples
					---
					# result definition
					float32 mean
					float32 std_dev
					---
					# feedback
					int32 sample
					float32 data
					float32 mean
					float32 std_dev
					*********************

samples at goal definition mean data number.

*/
 void spinThread()
 {
 	ros::spin();
 }

 int main(int argc,char **argv)
 {
 	ros::init(argc,argv,"test_averaging");

 	// The action client constructor also takes two arguments,
 	// the server name to connect to and a boolean option to automatically spin a thread. 
 	actionlib::SimpleActionClient<actionlib_tutorials::AveragingAction> ac("averaging");
 	// becuase of a boolean option setted false, a thread must be created.
 	// test description-> add option true, then can unuse thread.

 	// By using this method you can create multiple threads
 	// the thread is created and the ros node is started spinning in the background
 	boost::thread spin_thread(&spinThread);

 	ROS_INFO("Waiting for action server to start");

 	// the action client will wait for the action server to start before continuing.
 	ac.waitForServer(); // will wait for infinite time

 	ROS_INFO("Action server started, sending goal.");
 	//send a goal to the action server
 	actionlib_tutorials::AveragingGoal goal;
 	goal.samples = 2;
 	ac.sendGoal(goal);

 	// wait for the action to return.
 	// The timeout on the wait is set to 30 seconds,
 	// this means after 30 seconds the function will return 
 	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

 	if(finished_before_timeout)
 	{
 		actionlib::SimpleClientGoalState state = ac.getState();
 		ROS_INFO("Action finished: %s",state.toString().c_str());
 	}
 	else	
 		ROS_INFO("Action did not finish before the time out");

 	// we need to shutdown the ros node and join our thread back before exiting. 
 	ros::shutdown();
 	spin_thread.join();

 	return 0;

 }