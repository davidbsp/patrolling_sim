#include <sstream>
#include <string>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8MultiArray.h>

#include "PatrolAgent.h"

using namespace std;

void PatrolAgent::init(int argc, char** argv) {
        /*
            argv[0]=/.../patrolling_sim/bin/GBS
            argv[1]=__name:=XXXXXX
            argv[2]=maps/1r-5-map.graph
            argv[3]=ID_ROBOT
        */
        
    //More than One robot (ID between 0 and 99)
    if ( atoi(argv[3])>NUM_MAX_ROBOTS || atoi(argv[3])<-1 ){
        ROS_INFO("The Robot's ID must be an integer number between 0 an 99"); //max 100 robots 
        return;
    }else{
        ID_ROBOT = atoi(argv[3]); 
        //printf("ID_ROBOT = %d\n",ID_ROBOT); //-1 in the case there is only 1 robot.
    }

    graph_file = string(argv[2]);
    
    //Check Graph Dimension:
    dimension = GetGraphDimension(graph_file.c_str());
    
    //Create Structure to save the Graph Info;
    vertex_web = new vertex[dimension];
    
    //Get the Graph info from the Graph File
    GetGraphInfo(vertex_web, dimension, graph_file.c_str());

#if 0
    /* Output Graph Data */   
    for (i=0;i<dimension;i++){
        printf ("ID= %u\n", vertex_web[i].id);
        printf ("X= %f, Y= %f\n", vertex_web[i].x, vertex_web[i].y);
        printf ("#Neigh= %u\n", vertex_web[i].num_neigh);
        
        for (j=0;j<vertex_web[i].num_neigh; j++){
        printf("\tID = %u, DIR = %s, COST = %u\n", vertex_web[i].id_neigh[j], vertex_web[i].dir[j], vertex_web[i].cost[j]);
        }
        
        printf("\n");   
    }
#endif
    
    //instantaneous idleness and last visit initialized with zeros:
    instantaneous_idleness = new double[dimension];
    last_visit = new double[dimension];
    for(size_t i=0; i<dimension; i++){ 
        instantaneous_idleness[i]= 0.0; 
        last_visit[i]= 0.0; 
        
        if (i==current_vertex){
            last_visit[i]= 0.1; //Avoids getting back at the initial vertex
        }
    }
    
    interference = false;
    ResendGoal = false;
    goal_complete = true;
    
    /* Define Starting Vertex/Position (Launch File Parameters) */

    ros::init(argc, argv, "patrol_agent");  // will be replaced by __name:=XXXXXX
    ros::NodeHandle nh;
    
    double initial_x, initial_y;
    
    XmlRpc::XmlRpcValue list;
    nh.getParam("initial_pos", list);
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    
    int value = ID_ROBOT;
    if (value == -1){value = 0;}
    
    ROS_ASSERT(list[2*value].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    initial_x = static_cast<double>(list[2*value]);
    
    ROS_ASSERT(list[2*value+1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    initial_y = static_cast<double>(list[2*value+1]);
    
    //   printf("initial position: x = %f, y = %f\n", initial_x, initial_y);
    current_vertex = IdentifyVertex(vertex_web, dimension, initial_x, initial_y);
    //   printf("initial vertex = %d\n\n",current_vertex);  
    
    
    //Publicar dados de "odom" para nó de posições
    positions_pub = nh.advertise<nav_msgs::Odometry>("positions", 1); //only concerned about the most recent
        
    //Subscrever posições de outros robots
    positions_sub = nh.subscribe<nav_msgs::Odometry>("positions", 10, boost::bind(&PatrolAgent::positionsCB, this, _1));  
    
    char string1[40];
    char string2[40];
    
    if(ID_ROBOT==-1){ 
        strcpy (string1,"odom"); //string = "odom"
        strcpy (string2,"cmd_vel"); //string = "cmd_vel"
        TEAMSIZE = 1;
    }else{ 
        sprintf(string1,"robot_%d/odom",ID_ROBOT);
        sprintf(string2,"robot_%d/cmd_vel",ID_ROBOT);
        TEAMSIZE = ID_ROBOT + 1;
    }   

    /* Set up listener for global coordinates of robots */
    listener = new tf::TransformListener();

    //Cmd_vel to backup:
    cmd_vel_pub  = nh.advertise<geometry_msgs::Twist>(string2, 1);
    
    //Subscrever para obter dados de "odom" do robot corrente
    odom_sub = nh.subscribe<nav_msgs::Odometry>(string1, 1, boost::bind(&PatrolAgent::odomCB, this, _1)); //size of the buffer = 1 (?)
    
    ros::spinOnce(); 
    
    //Publicar dados para "results"
    results_pub = nh.advertise<std_msgs::Int8MultiArray>("results", 100);
    // results_sub = nh.subscribe("results", 10, resultsCB); //Subscrever "results" vindo dos robots
    results_sub = nh.subscribe<std_msgs::Int8MultiArray>("results", 10, boost::bind(&PatrolAgent::resultsCB, this, _1) ); //Subscrever "results" vindo dos robots

    initialize_node(); //dizer q está vivo
    ros::Rate loop_rate(1); //1 segundo
    
    /* Define Goal */
    if(ID_ROBOT==-1){ 
        strcpy (string1,"move_base"); //string = "move_base
    }else{
        sprintf(string1,"robot_%d/move_base",ID_ROBOT);
    }
    
    //printf("string = %s\n",string);
    ac = new MoveBaseClient(string1, true); 
    
    //wait for the action server to come up
    while(!ac->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    } 
    
    /* Wait until all nodes are ready.. */
    while(initialize){
        ros::spinOnce();
        loop_rate.sleep();
    }
}


void PatrolAgent::run() {
    
    /* Run Algorithm */ 
    
    while(ros::ok()){
        
        if (goal_complete) {
            
            if(next_vertex>-1) {
                //Update Idleness Table:
                update_idleness();
                
                current_vertex = next_vertex;       
            }
        
            //devolver proximo vertex tendo em conta apenas as idlenesses;
            next_vertex = compute_next_vertex();
            //printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);

            /** SEND GOAL (REACHED) AND INTENTION **/
            send_results(); 
            
            //Send the goal to the robot (Global Map)
            ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
            sendGoal(ac,vertex_web[next_vertex].x, vertex_web[next_vertex].y);  
            
            goal_complete = false;
        
        }
        else { //goal not complete (active)
            if (interference) {
                do_interference_behavior();
            }       
            
            if (ResendGoal) {
                //Send the goal to the robot (Global Map)
                ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
                sendGoal(ac,vertex_web[next_vertex].x, vertex_web[next_vertex].y);  
                ResendGoal = false; //para nao voltar a entrar (envia goal so uma vez)
            }
            
            if (end_simulation) {
                return;
            }   
        
        }
        ros::Duration delay = ros::Duration(0.1);
        delay.sleep();

    } // while ros.ok    
}

void PatrolAgent::update_idleness() {
    double now = ros::Time::now().toSec();
        
    for(int i=0; i<dimension; i++){
        if (i == next_vertex){
            last_visit[i] = now;    
        }
        instantaneous_idleness[i] = now - last_visit[i];           
    } 

    //Show Idleness Table:
    /*  for (i=0; i<dimension; i++){
        printf("idleness[%u] = %f\n",i,instantaneous_idleness[i]);      
    } */

}

void PatrolAgent::initialize_node (){ //ID,-1-1,ID
  
    ROS_INFO("Initialize Node: Robot %d",ID_ROBOT); 
    
    std_msgs::Int8MultiArray msg;   
    msg.data.clear();
    msg.data.push_back(ID_ROBOT);
    msg.data.push_back(-1);
    msg.data.push_back(-1);
    msg.data.push_back(ID_ROBOT);   
    
    int count = 0;
    
    //ATENÇÃO ao PUBLICADOR!
    ros::Rate loop_rate(0.5); //meio segundo
    
    while (count<3){ //send activation msg 3times
        results_pub.publish(msg);
        //ROS_INFO("publiquei msg: %s\n", msg.data.c_str());
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
}

void PatrolAgent::getRobotPose(int robotid, float &x, float &y, float &theta) {
    
    if (listener==NULL) {
        ROS_ERROR("TF listener null");
        return;
    }
    
    std::stringstream ss; ss << "robot_" << robotid;
    std::string robotname = ss.str();
    std::string sframe = "/map";                //Patch David Portugal: Remember that the global map frame is "/map"
    std::string dframe = "/" + robotname + "/base_link";
    tf::StampedTransform transform;

    try {
        listener->waitForTransform(sframe, dframe, ros::Time(0), ros::Duration(3));
        listener->lookupTransform(sframe, dframe, ros::Time(0), transform);
    }
    catch(tf::TransformException ex) {
        ROS_ERROR("Cannot transform from %s to %s\n",sframe.c_str(),dframe.c_str());
        ROS_ERROR("%s", ex.what());
    }

    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    theta = tf::getYaw(transform.getRotation());
    // printf("Robot %d pose : %.1f %.1f \n",robotid,x,y);
}

void PatrolAgent::odomCB(const nav_msgs::Odometry::ConstPtr& msg) { //colocar propria posicao na tabela
    
//  printf("Colocar Propria posição na tabela, ID_ROBOT = %d\n",ID_ROBOT);
    int idx = ID_ROBOT;
    
    if (ID_ROBOT<=-1){
        idx = 0;
    }
    
    float x,y,th;
    getRobotPose(idx,x,y,th);
    
    xPos[idx]=x; // msg->pose.pose.position.x;
    yPos[idx]=y; // msg->pose.pose.position.y;
    
//  printf("Posicao colocada em Pos[%d]\n",idx);
}



void PatrolAgent::sendGoal(MoveBaseClient *ac, double target_x, double target_y) 
{
    //Define Goal:
    move_base_msgs::MoveBaseGoal goal;
    //Send the goal to the robot (Global Map)
    geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(0.0);     
    goal.target_pose.header.frame_id = "map"; 
    goal.target_pose.header.stamp = ros::Time::now();    
    goal.target_pose.pose.position.x = target_x; // vertex_web[current_vertex].x;
    goal.target_pose.pose.position.y = target_y; // vertex_web[current_vertex].y;  
    goal.target_pose.pose.orientation = angle_quat; //alpha -> orientação  (queria optimizar este parametro -> através da direcção do vizinho!)
    ac->sendGoal(goal, boost::bind(&PatrolAgent::goalDoneCallback, this, _1, _2), boost::bind(&PatrolAgent::goalActiveCallback,this), boost::bind(&PatrolAgent::goalFeedbackCallback, this,_1));  
}



void PatrolAgent::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result){ //goal terminado (completo ou cancelado)
//  ROS_INFO("Goal is complete (suceeded, aborted or cancelled).");
    // If the goal succeeded send a new one!
    //if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) sendNewGoal = true;
    // If it was aborted time to back up!
    //if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) needToBackUp = true;    
    
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("SUCCESS");
        goal_complete = true;
    }else{
        ROS_INFO("CANCELLED or ABORTED...BACKUP & Resend Goal!");   //tentar voltar a enviar goal..
        backUpCounter = 0;
        backup();
        ResendGoal = true;
    }
}

void PatrolAgent::goalActiveCallback(){  //enquanto o robot esta a andar para o goal...
    goal_complete = false;
//      ROS_INFO("Goal is active.");
}

void PatrolAgent::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){    //publicar posições

    send_positions();
    
    interference = check_interference(ID_ROBOT);    
}


bool PatrolAgent::check_interference (int ID_ROBOT){ //verificar se os robots estao proximos
    
    int i;
    double dist_quad;
    
    /* Poderei usar TEAMSIZE para afinar */
    for (i=0; i<ID_ROBOT; i++){ //percorrer vizinhos (assim asseguro q cada interferencia é so encontrada 1 vez)
        
        dist_quad = (xPos[i] - xPos[ID_ROBOT])*(xPos[i] - xPos[ID_ROBOT]) + (yPos[i] - yPos[ID_ROBOT])*(yPos[i] - yPos[ID_ROBOT]);
        
        if (dist_quad <= /*sqrt*/4){    //robots are 2 meter or less apart
//          ROS_INFO("Feedback: Robots are very close. INTERFERENCE! Dist_Quad = %f", dist_quad);
            return true;
        }       
    }
    return false;
    
}

void PatrolAgent::backup(){
    
    while (backUpCounter<=100){
    
    if(backUpCounter==0){
        ROS_INFO("The wall is too close! I need to do some backing up...");
        // Move the robot back...
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = -0.1;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub.publish(cmd_vel);
    }
            
    if(backUpCounter==40){
        // Turn the robot around...
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.5;
        cmd_vel_pub.publish(cmd_vel);
    }
            
    if(backUpCounter==100){
        // Stop the robot...
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub.publish(cmd_vel);
            
        ROS_INFO("Done backing up, now on with my life!");      
    }

    backUpCounter++;
    }
    
}

void PatrolAgent::do_interference_behavior()
{
    // Stop the robot..         
    ROS_INFO("Interference detected!\n");   
    send_interference();

    //get own "odom" positions...
    ros::spinOnce();        
                
    //Waiting until conflict is solved...
    while(interference){
        interference = check_interference(ID_ROBOT);
        if (goal_complete || ResendGoal){
            interference = false;
        }
    }
}




// ROBOT-ROBOT COMMUNICATION



void PatrolAgent::send_positions()
{
    //Publish Position to common node:
    nav_msgs::Odometry msg; 
    
    int idx = ID_ROBOT;

    if (ID_ROBOT <= -1){
        msg.header.frame_id = "map";    //identificador do robot q publicou
        idx = 0;
    }else{
        char string[20];
        sprintf(string,"robot_%d/map",ID_ROBOT);
        msg.header.frame_id = string;
    }

    msg.pose.pose.position.x = xPos[idx]; //send odometry.x
    msg.pose.pose.position.y = yPos[idx]; //send odometry.y
  
    positions_pub.publish(msg);
    ros::spinOnce();
}


void PatrolAgent::receive_positions()
{
    
}

void PatrolAgent::positionsCB(const nav_msgs::Odometry::ConstPtr& msg) { //construir tabelas de posições
        
//     printf("Construir tabela de posicoes (receber posicoes), ID_ROBOT = %d\n",ID_ROBOT);    
        
    char id[20]; //identificador do robot q enviou a msg d posição...
    strcpy( id, msg->header.frame_id.c_str() );
    //int stamp = msg->header.seq;
//     printf("robot q mandou msg = %s\n", id);
    
    // Build Positions Table
    
    if (ID_ROBOT>-1){
    //verify id "XX" of robot: (string: "robot_XX/map")
    
        char str_idx[4];
        uint i;
        
        for (i=6; i<10; i++){
            if (id[i]=='/'){
                str_idx[i-6] = '\0';
                break;
            }else{
                str_idx[i-6] = id[i];
            }
        }
        
        int idx = atoi (str_idx);
    //  printf("id robot q mandou msg = %d\n",idx);
        
        if (idx >= TEAMSIZE && TEAMSIZE <= NUM_MAX_ROBOTS){
            //update teamsize:
            TEAMSIZE = idx+1;
        }
        
        if (ID_ROBOT != idx){  //Ignore own positions   
            xPos[idx]=msg->pose.pose.position.x;
            yPos[idx]=msg->pose.pose.position.y;        
        }   
//      printf ("Position Table:\n frame.id = %s\n id_robot = %d\n xPos[%d] = %f\n yPos[%d] = %f\n\n", id, idx, idx, xPos[idx], idx, yPos[idx] );       
    }
    
    receive_positions();
}


void PatrolAgent::send_results() { //goal and intention joint together
//goal: [ID,vertex,intention,0]

    std_msgs::Int8MultiArray msg;   
    msg.data.clear();
    msg.data.push_back(ID_ROBOT);
    msg.data.push_back(current_vertex);
    msg.data.push_back(next_vertex);
    msg.data.push_back(0);
    
    results_pub.publish(msg);   
    ros::spinOnce();    
}

void PatrolAgent::receive_results(int *vres)
{
    if(initialize==true && vres[0]==-1 && vres[1]==vres[2] && vres[2] == vres[3] && vres[3] ==0){   //"-1,0,0,0" (BEGINNING)
        ROS_INFO("Let's Patrol!\n");
        initialize = false;
    }
    
    if(initialize==false && vres[0]==-1 && vres[1]==1 && vres[2] == vres[3] && vres[3] ==0){   //"-1,1,0,0" (END)
       ROS_INFO("The simulation is over. Let's leave");
       end_simulation = true;     
    }    

    //received vertex and intention from other robot
    if(initialize==false && vres[0]>-1 && vres[1]>-1 && vres[2]>-1 && vres[3]==0){    //ID,vertex,intention,0

        if (vres[0] != ID_ROBOT){ //protection
        robot_arrived = vres[0];
        vertex_arrived = vres[1];
        arrived = true;
        
        //this will only be used by SEBS:
        robot_intention = vres[0];
        vertex_intention = vres[2];
        intention = true;
        
        }   
    }    
}

void PatrolAgent::send_interference(){
//interference: [ID,-1,-2,1]

    printf("Send Interference: Robot %d\n",ID_ROBOT);   
    
    std_msgs::Int8MultiArray msg;   
    msg.data.clear();
    msg.data.push_back(ID_ROBOT);
    msg.data.push_back(-1);
    msg.data.push_back(-2);
    msg.data.push_back(1);
    
    results_pub.publish(msg);   
    ros::spinOnce();
}




void PatrolAgent::resultsCB(const std_msgs::Int8MultiArray::ConstPtr& msg) { 

    std::vector<signed char>::const_iterator it = msg->data.begin();    
    int vres[4];
    
    for (int k=0; k<4; k++) {
        vres[k] = *it; it++;
    }
/*        
    int p1 = *it; //data[0]
    ++it;
     = *it; //data[1]
    ++it;
    int p3 = *it; //data[2]
    ++it;
    int p4 = *it; //data[2]
    ++it;  
*/   
    receive_results(vres);

    ros::spinOnce();
  
}
