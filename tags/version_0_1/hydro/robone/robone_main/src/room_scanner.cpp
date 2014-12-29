#include "room_scanner.h"
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define BASE_LINK "base_link"
#define MAP_LINK "map"
#define ODOM_LINK "odom"

#define SMALL_ERROR 0.01
#define MAX_RETRY_COUNT 1000

double square(const double &value)
{
    return value * value;
}

double goalVectorLength(const move_base_msgs::MoveBaseGoal& goal)
{
    double result = 0.0;

    result += square(goal.target_pose.pose.position.x);
    result += square(goal.target_pose.pose.position.y);
    result += square(goal.target_pose.pose.position.z);

    return result;
}

void waitForTransformation(const tf::TransformListener &listener, const std::string &targetFrame,
                           const std::string &sourceFrame)
{
    ros::Time last_error = ros::Time::now();
    std::string tf_error;

    //we need to make sure that the transform between the robot base frame and the global frame is available
    while(ros::ok() && !listener.waitForTransform(targetFrame, sourceFrame, ros::Time(),
                                             ros::Duration(0.1), ros::Duration(0.01), &tf_error))
    {
      ros::spinOnce();
      if (last_error + ros::Duration(5.0) < ros::Time::now())
      {
        ROS_WARN("Waiting on transform from %s to %s to become available before running costmap, tf error: %s",
            targetFrame.c_str(), sourceFrame.c_str(), tf_error.c_str());
        last_error = ros::Time::now();
      }
    }
}

bool goToGoal(const tf::TransformListener &listener, MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal)
{
    bool result = true;

    double distanceToGoal = goalVectorLength(goal);

    goal.target_pose.header.frame_id = BASE_LINK;
    goal.target_pose.header.stamp = ros::Time::now();

    ROS_INFO("Sending goal...");
    ac.sendGoal(goal);

    waitForTransformation(listener, BASE_LINK, ODOM_LINK);

    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener.lookupTransform(BASE_LINK, ODOM_LINK, ros::Time(0), start_transform);

    ros::Rate rate(50.0);
    ros::Duration waitDuration(5.0);
    bool waitForGoal = true;
    unsigned int goalReachTries = 0;

    ROS_INFO("Waiting for goal to be reached...");

    while (ros::ok() && waitForGoal && (goalReachTries < MAX_RETRY_COUNT))
    {

        ROS_INFO("Waiting for goal result...");
        ac.waitForResult(waitDuration);

        ros::spinOnce();

        rate.sleep();

        //get the current transform
        try
        {
            listener.lookupTransform(BASE_LINK, ODOM_LINK, ros::Time(0), current_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            break;
        }

        //see how far we've traveled
        tf::Transform relative_transform = start_transform.inverse() * current_transform;
        double distance = relative_transform.getOrigin().length();

        if (distance >= (distanceToGoal - SMALL_ERROR))
        {
            waitForGoal = false;
            ROS_INFO("Goal reached");
        }
        else
        {
            goalReachTries++;
            ROS_INFO("Goal not reached. Attempt: %d distance: %f", goalReachTries, distance);
        }
    }

    if (goalReachTries >= MAX_RETRY_COUNT)
    {
        result = false;
        ROS_ERROR("Goal was not reached with given %d retries", MAX_RETRY_COUNT);
    }

    return result;
}

void globalCostmapInitCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    //TODO: Implement logic
    ROS_INFO("globalCostmapInit received");
}


void globalCostmapChangedCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg)
{
    //TODO: Implement logic
    ROS_INFO("globalCostmapChanged received");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_navigation_goals");

    ros::NodeHandle nodeHandler;

    ROS_INFO("Starting robone main node...");

    MoveBaseClient ac("move_base", true);
    tf::TransformListener listener;

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ros::Subscriber globalCostmapInit = nodeHandler.subscribe("/move_base/global_costmap/costmap", 1, globalCostmapInitCallback);
    ros::Subscriber globalCostmapUpdates = nodeHandler.subscribe("/move_base/global_costmap/costmap_updates", 1,
                                                                 globalCostmapChangedCallback);

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;

    goToGoal(listener, ac, goal);

    return 0;
}
