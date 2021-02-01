/*
action client node used goal,active,feedback callback isn't the most convenient registering class method
however , boost::bind can helpful (but syntax understanding hard.)
*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_tutorials/AveragingAction.h>

using namespace actionlib_tutorials;
typedef actionlib::SimpleActionClient<AveragingAction> Client;


class MyNode
{
public:
	MyNode() : ac_("averaging",true)
	{
		ROS_INFO("Waiting for action server to start");
		ac_.waitForServer();
		ROS_INFO("Action server started");
	}

	void doSturff(int order)
	{
		AveragingGoal goal;
		goal.samples = order;
		//when i used only result callback function
		ac_.sendGoal(goal , boost::bind(&MyNode::doneCB, this, _1, _2), Client::SimpleActiveCallback(),Client::SimpleFeedbackCallback());//&MyNode::activeCB ,&MyNode::feedbackCB );
		//when i used result + active func
		ac_.sendGoal(goal , boost::bind(&MyNode::doneCB, this, _1, _2), boost::bind(&MyNode::activeCB,this),boost::bind(&MyNode::feedbackCB,this , _1));//&MyNode::activeCB ,&MyNode::feedbackCB );
	



	}

	void doneCB(const actionlib::SimpleClientGoalState& state,
			const AveragingResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]",state.toString().c_str());

		ROS_INFO("result mean : %.3f", result->mean);
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

private:
	Client ac_;

};


int main(int argc, char **argv)
{
	ros::init(argc,argv,"test_averaging3");
	MyNode my_node;
	my_node.doSturff(2);
	ros::spin();
	return 0;

}



