#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/AveragingAction.h>
#include <boost/thread.hpp>

/*
blocking until a goal completes doesn't provide enough flexibility. 
 Below is an example of how to use callbacks 
 to avoid using a waitForResult() call to block for the goal to finish. 
*/




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

using namespace actionlib_tutorials;
typedef actionlib::SimpleActionClient<AveragingAction> Client;

void doneCB(const actionlib::SimpleClientGoalState& state,
			const AveragingResultConstPtr& result);
void activeCB();
void feedbackCB(const AveragingFeedbackConstPtr& feedback);

 int main(int argc,char **argv)
 {
 	ros::init(argc,argv,"test_averaging2");

 	// The action client constructor also takes two arguments,
 	// the server name to connect to and a boolean option to automatically spin a thread. 
 	Client ac("averaging",true);
 	// becuase of a boolean option setted false, a thread must be created.

 	// By using this method you can create multiple threads
 	// the thread is created and the ros node is started spinning in the background

 	ROS_INFO("Waiting for action server to start");

 	// the action client will wait for the action server to start before continuing.
 	ac.waitForServer(); // will wait for infinite time

 	ROS_INFO("Action server started, sending goal.");
 	//send a goal to the action server
 	actionlib_tutorials::AveragingGoal goal;
 	goal.samples = 2;
 	ac.sendGoal(goal , &doneCB, &activeCB ,&feedbackCB );

 	ros::spin();

 	return 0;

 }

void doneCB(const actionlib::SimpleClientGoalState& state,
			const AveragingResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]",state.toString().c_str());

	ROS_INFO("result mean : %.3i", result->mean);
	ROS_INFO("result std_dev : %.3f",result->std_dev);

	ros::shutdown(); // note : action client code is thread ! 
}

 void activeCB()
 {
 	ROS_INFO("Active Function Called");
 	ROS_INFO("Goal reached to Action Server");
 	ROS_INFO("Client node is Active State ");
 }

void feedbackCB(const AveragingFeedbackConstPtr& feedback)
{
	ROS_INFO("Feedback Function Called");
	ROS_INFO("sample : %d",feedback->sample);
	ROS_INFO("data : %.3f", feedback->data);
	ROS_INFO("mean : %.3f", feedback->mean);
	ROS_INFO("std_dev : %.3f", feedback->std_dev);
}