#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include "patrolling_sim/GoToStartPosSrv.h" 


 typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

 using namespace std;
  
 ros::NodeHandle* n_ptr;
 int teamsize;
 double last_cmd_vel_time;
 
 
 void cmd_velCB(const geometry_msgs::Twist::ConstPtr& msg){
     //ROS_INFO("receiving cmd_vels");
     last_cmd_vel_time = ros::Time::now().toSec();
 }
 

 bool GotoStartPosSrvCallback(patrolling_sim::GoToStartPosSrv::Request& Req, patrolling_sim::GoToStartPosSrv::Response& Rep){
        
    if (Req.teamsize.data != teamsize) {
        ROS_INFO("Service was called with a different team size (%d) than expected (%d). Leaving.", Req.teamsize.data, teamsize);
        return false;
    }
    
    double starting_patrol_pos_x [teamsize];
    double starting_patrol_pos_y [teamsize];
   
    //list of doubles from the parameter server
    vector<double> initial_pos_list;
    n_ptr->getParam("initial_pos", initial_pos_list);
    
    unsigned i;
    int j=0;    //robot id
    
    for(i = 0; i < initial_pos_list.size(); i++) {        
        if ( i % 2 == 0 ){ //even: x
            starting_patrol_pos_x[j] = initial_pos_list[i];
            //ROS_INFO("starting_patrol_pos_x[%d] = %f", j, starting_patrol_pos_x[j]);
            
        }else{ //odd: y
            starting_patrol_pos_y[j] = initial_pos_list[i];
            //ROS_INFO("starting_patrol_pos_y[%d] = %f", j, starting_patrol_pos_y[j]);
            j++;
        }
    }
    
    
    //connect to move_base server and send robots one by one to starting positions...
    
    //array of pointers:
    MoveBaseClient *ac_ptr[teamsize];
    ros::Rate loop_rate(1); //1 sec
    
    for (j=teamsize-1; j>=0; j--){
        
        char move_string[20];
        sprintf(move_string,"robot_%d/move_base",j);
        //ROS_INFO("%s",move_string);

        
        MoveBaseClient ac(move_string, true);
        ac_ptr[j] = &ac;
        
        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        } 
        ROS_INFO("Connected with move_base action server");     
        
        move_base_msgs::MoveBaseGoal goal;
        
        geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(0.0);  

        //we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = starting_patrol_pos_x[j];
        goal.target_pose.pose.position.y = starting_patrol_pos_y[j];
        goal.target_pose.pose.orientation = angle_quat; //doesn't matter really.

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);
        
        //wait a bit and send next goal to other robots.
        
        i=0;
        
        while( i<Req.sleep_between_goals.data ){
            i++;
            ros::spinOnce();    //trigger cmd_vel callbacks
            loop_rate.sleep(); 
        }
    }
    
    ROS_INFO("Let's make sure that all robots reach their goals...!");

    last_cmd_vel_time = ros::Time::now().toSec();   //safe initialization
    double current_time = ros::Time::now().toSec();
    
    while( current_time-last_cmd_vel_time < 5.0 ){ //check cmd_vels not received in the last 5 secs
        current_time = ros::Time::now().toSec();
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    ROS_INFO("All robots successfully reached their starting positions");    
    return true;
 }
 
 
int main(int argc, char **argv){

  ros::init(argc, argv, "GoToStartPos");
  ros::NodeHandle n;
  n_ptr = &n;
  
  ros::Duration(2.0).sleep();   //waiting for all navigation topics to show up.
   
  //get cmd_vel topics:
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  
  vector<string> cmd_vel_topic_array(32, ""); 
  teamsize=0;

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        if(info.datatype=="geometry_msgs/Twist"){   //cmd_vel topics
            cmd_vel_topic_array[teamsize] = info.name;
            teamsize++;
        }
  }
  
  //for(int o=0; o<teamsize; o++){ ROS_ERROR("%s",cmd_vel_topic_array[o].c_str()); }
  
  if (teamsize==0){
    ROS_ERROR("No navigation information retrieved. Is \"move_base\" running?");
    return -1;   
    
  }else{
      ROS_INFO("Detected %d robots.", teamsize);
  }
  
  vector<ros::Subscriber> cmd_vel_sub(teamsize);
  
  for(int o=0; o<teamsize; o++){ //create subscribers with a common callback:
      cmd_vel_sub[o] = n.subscribe(cmd_vel_topic_array[o], 1, cmd_velCB);
  }  

  ros::ServiceServer service = n.advertiseService("GotoStartPosSrv", GotoStartPosSrvCallback);
  ROS_WARN("Ready to send robots to starting position.");
  
  ros::spin();

  return 0;
}