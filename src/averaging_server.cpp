#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/AveragingAction.h>
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

class AveragingAction
{
public:
	// The action server takes arguments of a node handle, name of the action, and optionally an executeCB.
	// In this example the action server is created without the arguments for the executeCB. 
	// Instead the goal and preempt callbacks are registered with the action server in the constructor for the action 
	// after the action server has been constructed. 
	AveragingAction(std::string name) : as_(nh_, name, false), action_name_(name)
	{
		// register the goal and feedback callbacks
		as_.registerGoalCallback(boost::bind(&AveragingAction::goalCB,this));
		as_.registerPreemptCallback(boost::bind(&AveragingAction::preemptCB,this));

		sub_ = nh_.subscribe("/random_number",1,&AveragingAction::analysisCB,this);
		as_.start(); // action server is started. 
	}
	~AveragingAction(){}

	void goalCB();
	void preemptCB();
	void analysisCB(const std_msgs::Float32::ConstPtr& msg);

protected:
	ros::NodeHandle nh_;
	// NodeHandle instance must be created before this line. 
	// Otherwise strange error occurs.
	actionlib::SimpleActionServer<actionlib_tutorials::AveragingAction> as_;
	std::string action_name_;
	int data_count_,goal_;
	float sum_,sum_sq_;
	actionlib_tutorials::AveragingFeedback feedback_;
	actionlib_tutorials::AveragingResult result_;
	ros::Subscriber sub_;

};

// When the goalCB is called
// the action needs to accept the goal and store any important information. 
void AveragingAction::goalCB()
{
	// when server accpet goal, initialize variables !  
	data_count_ = 0;
	sum_ = 0;
	sum_sq_ = 0;
	// accept the new goal
	goal_ = as_.acceptNewGoal()->samples;
}

void AveragingAction::preemptCB()
{
	ROS_INFO("%s: Preempted",action_name_.c_str());
	as_.setPreempted(); // look like mean mutex
}

void AveragingAction::analysisCB(const std_msgs::Float32::ConstPtr& msg)
{	// checks that the action is still in an active state before continuing to process the data. 
	if(!as_.isActive())	return;

	data_count_++;
	feedback_.sample = data_count_;
	feedback_.data = msg->data;
	
	//compute the mean and std_dev of the data
	sum_ += msg->data;
	feedback_.mean = sum_/data_count_;
	sum_sq_ += pow(msg->data,2);
	feedback_.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback_.mean, 2)));
	as_.publishFeedback(feedback_);

	if(data_count_ >= goal_)
	{
		result_.mean = feedback_.mean;
		result_.std_dev = feedback_.std_dev;
		if(result_.mean < 5.0)
		{
			ROS_INFO("%s : Aborted",action_name_.c_str());
			as_.setAborted(result_);
		}
		else
		{
			ROS_INFO("%s: Succeeded",action_name_.c_str());
			as_.setSucceeded(result_);
		}
	}
}




int main(int argc,char** argv)
{
	ros::init(argc,argv,"averaging");
	AveragingAction averaging(ros::this_node::getName());
	ros::spin();

	return 0;
}