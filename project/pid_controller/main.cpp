/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 				Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 * @brief       
 **/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <iostream>
#include <fstream>
#include <typeinfo>

#include "json.hpp" // JSON    
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include "Eigen/QR"
#include "behavior_planner_FSM.h" //      
#include "motion_planner.h" //    
#include "planning_params.h"  //      
#include "utils.h"    //   
#include "pid_controller.h"  // PID    (main.cpp )

#include <limits>
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>  // WebSocket 
#include <math.h>  
#include <vector>
#include <cmath>
#include <time.h>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

//  main.cpp  
//    : `pid_controller.h` (PID   )
//    : `pid_controller.cpp` (PID   )



// PID parameters (    PID  )
//pid_controller.h planning_params.h      
//mithul12  () - PID  
static const double KP_STEER = 0.3;
static const double KI_STEER = 0.001;
static const double KD_STEER = 0.3;
static const double MAX_STEER = 1.2;
static const double MIN_STEER = -1.2;

static const double KP_THROTTLE = 0.2;
static const double KI_THROTTLE = 0.0009;
static const double KD_THROTTLE = 0.1;
static const double MAX_THROTTLE = 1.0;
static const double MIN_THROTTLE = -1.0;



/**
 * @brief JSON     

 hasData(string s)   :   JSON       
                                   (WebSocket)   JSON  ,  JSON    .
 */
string hasData(string s) { //  `s`  
  auto found_null = s.find("null");  //  `s` "null" 
    auto b1 = s.find_first_of("{");  // `{` (JSON )  
    auto b2 = s.find_first_of("}");   // `}` (JSON )  
    if (found_null != string::npos) {  // "null" 
      return ""; //    (JSON  )
    }
    else if (b1 != string::npos && b2 != string::npos) { // `{`  `}` 
      return s.substr(b1, b2 - b1 + 1); // `{`  `}` JSON   
    }
    return ""; // JSON     
}

/**
sgn(T val)   :    (Sign)   
                        ,  1,  -1, 0 0  .
 * @brief     (: 1, : -1, 0: 0)
 */
template <typename T>  //  :   (T)    ,  int, double, float          
                       //, (int)  (double, float)  .
int sgn(T val) { //  val   ---> (sgn(T val)     .)
    return (T(0) < val) - (val < T(0)); // :       true(1)  false(0)  
                                       // T(0) < val  (val 0  true(1),  false(0))
                                       //val < T(0) (val 0  true(1),  false(0))
                                       //(T(0) < val) - (val < T(0)) ---->   0  1 - 0 = 1 / 0 0 - 0 = 0 / 0  0 - 1 = -1
                                      // 1,  -1, 0 0 .
}                                       

/**
 * @brief      (x1, y1) (x2, y2)    
                ,       
                 .
                  .
           path_planner()   
           
 */
double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1); //atan2(y2 - y1, x2 - x1)  (x1, y1) (x2, y2)     
}


//        
//   (Behavior Planning)  BehaviorPlannerFSM  ()
//,  , ,     

//        
//     ,  ,      
//BehaviorPlannerFSM  behavior_planner_FSM.h  behavior_planner_FSM.cpp  .
BehaviorPlannerFSM behavior_planner( //BehaviorPlannerFSM   behavior_planner  -->  path_planner()   .

      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
// (motion_planner) motion_planner.cpp  MotionPlanner   -->         
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

//       
//  set_obst()      
bool have_obst = false; //    (: false)
vector<State> obstacles; //      

// #1


/**
 * @brief      , 
           (, , )**    
        , ,      
          main.cpp  ,   (motion_planner.cpp)    

   motion_planner.cpp -->  motion_planner.generate_spirals(), motion_planner.get_best_spiral_idx()    
   behavior_planner_FSM.cpp --> behavior_planner.get_active_maneuver()    
   main.cpp --> path_planner()   WebSocket    
   #### main.cpp        !!!!
 * @note    
 */

 /* 
void path_planner(
  vector<double>& x_points, //main.cpp  X    -->  :   X 
  vector<double>& y_points, //  Y 
  vector<double>& v_points,  //main.cpp     -->  :  
  double yaw,  //main.cpp  ()   -->  :  
  double velocity, // main.cpp     ---> :   
  State goal, //  ,behavior_planner_FSM.cpp     -->  :   
  bool is_junction, // , main.cpp ,    -->  :   
  string tl_state, //, main.cpp    --->  :  
  vector< vector<double> >& spirals_x, //2D , motion_planner.cpp    X   --> :    (X)
  vector< vector<double> >& spirals_y, //    (Y)
  vector< vector<double> >& spirals_v, //    ()
  vector<int>& best_spirals){  //,motion_planner.cpp      -->  :   

  State ego_state; //ego_state  (Ego Vehicle)    (State) 
                   // , ,  (yaw)     
                   //State  utils.h      
                   //    , ,    
                   //path_planner()   , ,    
                   //      

  
  //  ego_state(  )       
  // X , Y , (velocity) ego_state 
  // ego_state    -->            , 
  //  , path_planner()     , motion_planner    .
  ego_state.location.x = x_points[x_points.size()-1]; //x_points   X    (vector)
                                                      //x_points.size()-1   (  )
                                                      //,   X  ego_state.location.x 
  ego_state.location.y = y_points[y_points.size()-1]; //Y    
  ego_state.velocity.x = velocity; // velocity    ( m/s),  ego_state.velocity.x    




  //   (yaw) ,   
  //  2      ,      () .
  if( x_points.size() > 1 ){ //     2     
                            //x_points.size() > 1   :    2     
                                                                      // 1    .
  	ego_state.rotation.yaw = angle_between_points( //     () .
      x_points[x_points.size()-2], y_points[y_points.size()-2], //  
      x_points[x_points.size()-1], y_points[y_points.size()-1] //  
      //   (x_n-2, y_n-2)    (x_n-1, y_n-1)   yaw( ) .
      //,   ( ) yaw .
    );
  	
    ego_state.velocity.x = v_points[v_points.size()-1];  //   
                                                        //v_points   .
                                                        //  (v_points[v_points.size()-1])  ego_state.velocity.x .
                                                        //,    ego_state .
  	if(velocity < 0.01) //    
  		ego_state.rotation.yaw = yaw; //  yaw   ,,       

  }
//     (, ,  ),        
//behavior_planner     ,      .
// get_active_maneuver()  behavior_planner_FSM.h Inline  ,   (FOLLOW_LANE, STOPPED )   .
//get_active_maneuver()  main.cpp behavior_planner.get_active_maneuver(); 
  Maneuver behavior = behavior_planner.get_active_maneuver(); //    

  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state); //   (goal)  .
                                                               //  (ego_state),  (is_junction),  (tl_state)   goal 
                                                                //,      .
                                                                //,  goal   ,      goal  
  if(behavior == STOPPED){ //   (STOPPED ),     

  	int max_points = 20;
    // (x_points, y_points) .
  	double point_x = x_points[x_points.size()-1]; //  ,       (x_points.size()-1) 
  	double point_y = y_points[x_points.size()-1];
    

    //       .
    //x_points, y_points       .
    //v_points.push_back(0);   0    .
    //            
  	while( x_points.size() < max_points ){
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);

  	}
  	return; //       -->       
  } */




  /* 
  // path_planner  -->  ,     !
  void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, double yaw, double velocity, State goal, bool is_junction, string tl_state, vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, vector<int>& best_spirals){

    State ego_state;
  
    ego_state.location.x = x_points[x_points.size()-1];
    ego_state.location.y = y_points[y_points.size()-1];
    ego_state.velocity.x = velocity;
  
    if( x_points.size() > 1 ){
        ego_state.rotation.yaw = angle_between_points(x_points[x_points.size()-2], y_points[y_points.size()-2], x_points[x_points.size()-1], y_points[y_points.size()-1]);
        ego_state.velocity.x = v_points[v_points.size()-1];
        if(velocity < 0.01)
            ego_state.rotation.yaw = yaw;
  
    }
  
    Maneuver behavior = behavior_planner.get_active_maneuver();
  
    goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);
  
    if(behavior == STOPPED){
  
        int max_points = 20;
        double point_x = x_points[x_points.size()-1];
        double point_y = y_points[x_points.size()-1];
        while( x_points.size() < max_points ){
          x_points.push_back(point_x);
          y_points.push_back(point_y);
          v_points.push_back(0);
  
        }
        return;
    }
  */

//   path_planner
void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, double yaw, double velocity, State goal, bool is_junction, string tl_state, vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, vector<int>& best_spirals){
  State ego_state;
  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  ego_state.velocity.x = velocity;
  if( x_points.size() > 1 ){
  ego_state.rotation.yaw = angle_between_points(x_points[x_points.size()-2], y_points[y_points.size()-2], x_points[x_points.size()-1], y_points[y_points.size()-1]);
  ego_state.velocity.x = v_points[v_points.size()-1];
  if(velocity < 0.01)
  ego_state.rotation.yaw = yaw;
  }
  Maneuver behavior = behavior_planner.get_active_maneuver();
  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);
  if(behavior == STOPPED){
  int max_points = 20;
  double point_x = x_points[x_points.size()-1];
  double point_y = y_points[x_points.size()-1];
  while( x_points.size() < max_points ){
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(0);
  }
  return;
  }
  auto goal_set = motion_planner.generate_offset_goals(goal);
  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);
  auto desired_speed = utils::magnitude(goal.velocity);
  State lead_car_state;  // = to the vehicle ahead...
  if(spirals.size() == 0){
  cout << "Error: No spirals generated " << endl;
  return;
  }
  for(int i = 0; i < spirals.size(); i++){
    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory( spirals[i], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);
    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;
    for(int j = 0; j < trajectory.size(); j++){
      double point_x = trajectory[j].path_point.x;
      double point_y = trajectory[j].path_point.y;
      double velocity = trajectory[j].v;
      spiral_x.push_back(point_x);
      spiral_y.push_back(point_y);
      spiral_v.push_back(velocity);
    }
    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);
  }
  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;
  if(best_spirals.size() > 0)
  best_spiral_idx = best_spirals[best_spirals.size()-1];
  int index = 0;
  int max_points = 20;
  int add_points = spirals_x[best_spiral_idx].size();
  while( x_points.size() < max_points && index < add_points ){
    double point_x = spirals_x[best_spiral_idx][index];
    double point_y = spirals_y[best_spiral_idx][index];
    double velocity = spirals_v[best_spiral_idx][index];
    index++;
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(velocity);
  }
} //   path_planner,   starter   







  //   ,   
  //motion_planner         ,   
  //utils::magnitude(goal.velocity)    
  auto goal_set = motion_planner.generate_offset_goals(goal); // (goal)      (Goal Set) 
                                                              //           
                                                              //goal  ,             
                                                              //     goal_set 

  auto spirals = motion_planner.generate_spirals(ego_state, goal_set); //  (ego_state)  (goal_set)     (spirals) 
                                                                      //  spiral        .
                                                                      // goal_set     ,       (spiral)  .
                                                                      // motion_planner.generate_spirals(ego_state, goal_set);        spirals 
                                                                      //:        ,     
  auto desired_speed = utils::magnitude(goal.velocity); //   
                                                        //  (goal)      desired_speed 
                                                        // goal.velocity State   
                                                        // utils::magnitude(goal.velocity)   ( ) 
                                                        // ,            


  //     (mithul12   X)
  //    , (lead car)  ,   (spirals)     
  State lead_car_state;  // = to the vehicle ahead...
                         //lead_car_state  ( , Lead Car)    
                         //   ,        
                         //  State  ,  , ,      
                         //     ,    

  if(spirals.size() == 0){ //  (spirals)  ,     
                           // spirals motion_planner.generate_spirals()        (vector).
                           // spirals.size() == 0,             .
                           //, spirals       ,    (path_planner()) .
  	cout << "Error: No spirals generated " << endl;
  	return;
  }



  //       (spirals) ,       
  // spiral( )   ,  spirals_x, spirals_y, spirals_v .
  for(int i = 0; i < spirals.size(); i++){ //spirals     
                                           //spirals motion_planner.generate_spirals()     () 
                                           //spirals.size()    

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory( 
      spirals[i], desired_speed, ego_state,lead_car_state, behavior); //  spiral( )     trajectory 
                                                                      //generate_trajectory() motion_planner   
                                                                      //spirals[i]  i  
                                                                      //desired_speed     
                                                                      //ego_state     (,  )
                                                                      //lead_car_state    ( )
                                                                      //behavior     (, ,  )
                                                                      // trajectory i spiral       
//  X, Y      
//   (spirals)    X, Y      
//motion_planner._velocity_profile_generator.generate_trajectory()    .
//      (spirals_x, spirals_y, spirals_v)       .
    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;

    // trajectory() X, Y     spiral_x, spiral_y, spiral_v 
    //trajectory[j] j      
    //,  spirals[i]     X, Y     
    for(int j = 0; j < trajectory.size(); j++){
      double point_x = trajectory[j].path_point.x; //trajectory[j].path_point.x --> X
      double point_y = trajectory[j].path_point.y; // trajectory[j].path_point.y --> y
      double velocity = trajectory[j].v; //
                                       
      
      // spiral  X, Y,   spirals_x, spirals_y, spirals_v 
      //,   (spirals)  X, Y           .
                                  // "  " (  )
      spiral_x.push_back(point_x);  //point_x    (  X, Y  )
                                   // spiral_x    (spiral)    X, Y,  
      spiral_v.push_back(velocity); 
    }

                                  
                                  //   (spirals)" 
                                  //   spirals    spiral 
                                  // spirals_x    (spirals)  ...s 
    spirals_x.push_back(spiral_x); //spirals_x[i]  i    X 
    spirals_y.push_back(spiral_y); //spirals_y[i]  i    Y 
    spirals_v.push_back(spiral_v); //spirals_v[i]  i     

/*
, point_x, point_y, velocity    X, Y    
spiral_x, spiral_y, spiral_v     (spiral)  X, Y,  
spirals_x, spirals_y, spirals_v     (spirals)   
*/

  }


  //  "  (spiral) ,        "
  // motion_planner.get_best_spiral_idx()         
  //   x_points, y_points, v_points      


  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal); //get_best_spiral_idx()     
                                                                       // spirals(    )  (obstacles)   (goal)    
                                                                      //     best_spirals  
  int best_spiral_idx = -1; //       -1 
                            //  get_best_spiral_idx()     ,  -1 .

  if(best_spirals.size() > 0) //best_spirals     ,     .
  	best_spiral_idx = best_spirals[best_spirals.size()-1]; //best_spirals.size() - 1     
                                                          // ,    best_spiral_idx  ,   -1 


  //                                                         
  int index = 0; // index = 0;       
  int max_points = 20; // 20    (     )
  int add_points = spirals_x[best_spiral_idx].size(); //add_points = spirals_x[best_spiral_idx].size();   (best_spiral_idx)    
  while( x_points.size() < max_points && index < add_points ){ //    (X, Y)    
                                                               //x_points.size() < max_points      (20)   
                                                               //index < add_points         

   //  index (X, Y)  
    double point_x = spirals_x[best_spiral_idx][index]; //   index X 
    double point_y = spirals_y[best_spiral_idx][index]; //   index Y 
    double velocity = spirals_v[best_spiral_idx][index]; //   index 
    index++;
    x_points.push_back(point_x); //   (X, Y)   x_points, y_points, v_points 
    y_points.push_back(point_y); //          .
    v_points.push_back(velocity);
    // x_points, y_points, v_points           
  } 


// (25 2 13 )


/**
 * @brief     (   )
 */
/*
  set_obst() (Obstacle)    
, x_points y_points    , obstacles  State    .

  main.cpp    obstacles    
    obstacles          
*/

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag){

	for( int i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i]; //x_points:  x   ,x_points    State  ,obstacles     
		obstacle.location.y = y_points[i]; //y_points:  y  
		obstacles.push_back(obstacle); //obstacles:  (State)   ( )
	}
	obst_flag = true; //obst_flag:      ( )
                    //   obst_flag true 
}
// #2


// normalize_angle
double normalize_angle(double angle)
{
    if (std::abs(angle) > M_PI)
    {
        std::cout << "Renormalizing angle" <<  std::endl;
        if(angle > M_PI)
            std::cout << "Angle > Pi" <<  std::endl;
            angle -= 2*M_PI;
        if(angle < -M_PI)
            std::cout << "Angle < -Pi" <<  std::endl;
            angle += 2*M_PI;
        return angle;
    }
}

// find_closet_point
// find the point that is closest to the point (x_position, y_position)
// and return its index in the list of points (x_pts, y_pts)
int find_closest_point(double x_position, double y_position, const vector<double> &x_pts, const vector<double> &y_pts)
{
    int closest_point_index = 0;
    double min_dist = std::numeric_limits<double>::max();
    double dist = 0;

    for (int i=0; i < x_pts.size(); ++i)
    {
        dist = std::hypot(x_pts[i] - x_position, y_pts[i] - y_position);
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_point_index = i;
        }
    }

    return closest_point_index;
}
/**
 * @brief   
  * @note WebSocket      
 */

 //main()  (   ),  WebSocket         
int main ()
{
  cout << "starting server" << endl; //"starting server"    
  uWS::Hub h;  // WebSocket               

  double new_delta_time;
  int i = 0;


   // PID      (steer_pid_data.txt, throttle_pid_data.txt    )
   // PID(--)      
  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc); //std::ofstream::trunc       
                                                                                   //,     PID    
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();


  //     
  time_t prev_timer; //prev_timer, timer      
  time_t timer;
  time(&prev_timer); //time(&prev_timer);      prev_timer 
                     //  new_delta_time        


  // initialize pid steer
  /**
  * TODO (Step 1): create pid (pid_steer) for steer command and initialize values
  **/


  // initialize pid throttle
  /**
  * TODO (Step 1): create pid (pid_throttle) for throttle command and initialize values
  **/


 // TODO (Step 1): PID    
  PID pid_steer = PID(); // (steer) PID  
  pid_steer.Init(KP_STEER, KI_STEER, KD_STEER, MAX_STEER, MIN_STEER); //mithul12 

  PID pid_throttle = PID(); //  (throttle) PID  
  pid_throttle.Init(KP_THROTTLE, KI_THROTTLE, KD_THROTTLE, MAX_THROTTLE, MIN_THROTTLE); //mithul12 
// TODO (Step 1)  PID  ,         .






/*
  WebSocket  (h.onMessage())  ,     ** (path_planner )**  
,          
*/
  //path_planner()  WebSocket  (h.onMessage()) 
  //uWS::WebSocket<uWS::SERVER> ws  WebSocket ,   
  //: WebSocket     ,    .
  h.onMessage([&pid_steer, &pid_throttle, &new_delta_time, &timer, &prev_timer, //[&pid_steer, &pid_throttle, &new_delta_time, ...]     (     )
    &i, &prev_timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)  //char *data, size_t length, uWS::OpCode opCode    
  {
        auto s = hasData(data);  // JSON  
                                 //hasData(data)   null   (   )

        if (s != "") {

          auto data = json::parse(s); // JSON  
                                      //json::parse(s)   JSON  
                                      //:   JSON       .


          // create file to save values
          // PID     
          fstream file_steer; //fstream   OPEN
          file_steer.open("steer_pid_data.txt"); //steer_pid_data.txt  (steering) PID  
          fstream file_throttle; //fstream   OPEN
          file_throttle.open("throttle_pid_data.txt"); //throttle_pid_data.txt  (throttle) PID  

          
          //    
          vector<double> x_points = data["traj_x"]; //x_points, y_points      
          vector<double> y_points = data["traj_y"];
          vector<double> v_points = data["traj_v"]; //v_points      
          double yaw = data["yaw"]; //yaw    ()
          double velocity = data["velocity"]; //velocity    
          double sim_time = data["time"]; //sim_time    
          double waypoint_x = data["waypoint_x"]; //waypoint_x, waypoint_y, waypoint_t    (x, y) (yaw)
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"]; //is_junction     
          string tl_state = data["tl_state"]; //tl_state   (: "red", "green" )

          double x_position = data["location_x"]; //x_position, y_position, z_position    
          double y_position = data["location_y"];
          double z_position = data["location_z"];
          // --- :       .

          //   
          if(!have_obst){
          	vector<double> x_obst = data["obst_x"]; //data["obst_x"], data["obst_y"]   x, y  
          	vector<double> y_obst = data["obst_y"];
          	set_obst(x_obst, y_obst, obstacles, have_obst); //  obstacles  
                                                          //have_obst = true;      
          } // --:        .
         

          //   
          //goal       (waypoint) .
          //waypoint_x, waypoint_y, waypoint_t     
          State goal;
          goal.location.x = waypoint_x;
          goal.location.y = waypoint_y;
          goal.rotation.yaw = waypoint_t;

           //   
           //    path_planner()  .
           //     
          vector< vector<double> > spirals_x; //spirals_x, spirals_y, spirals_v    (spirals) x, y    
          vector< vector<double> > spirals_y;
          vector< vector<double> > spirals_v;
          vector<int> best_spirals; //best_spirals    

          // path_planner()        
          path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);



          // Save time and compute delta time
            //      
          time(&timer);   //time(&timer);    . 


          new_delta_time = difftime(timer, prev_timer); //difftime(timer, prev_timer);      ( ) .
          prev_timer = timer; //prev_timer   

          /* 
            WebSocket       ,   path_planner()      
            ,   PID      
          */





          //    (steering) PID    
          //,          .
          ////////////////////////////////////////
          // Steering control
          ////////////////////////////////////////

          /**
          * TODO (step 3): uncomment these lines
          **/
//           // Update the delta time with the previous command
           pid_steer.UpdateDeltaTime(new_delta_time);  //PID  delta time( )  .
                                                         //      PID     .
                                                         //     
                                                         //464,467 pid  


           //  TODO (Step 3): (steer)  
          // Compute steer error
        
        double steer_output = 0.0;
        //double steer_output; //mithul12       , mithul12  ,   
                            //steer_output : PID     
        
        //double error_steer; //  
        double error_steer = 0.0;  //   (  ) ,error_steer :     
                                   // 0   -->      ,  0 
                                   //      .
                                   // 3      
 
        // 3  ,mithul12                              
        int closest_point_index = find_closest_point(x_position, y_position, x_points, y_points); //  (x_position, y_position)      .
                                                                                              //,            .
        double angle = angle_between_points(x_position, y_position, x_points[closest_point_index], y_points[closest_point_index]); 
                       //       () . 
                       //,     (waypoint ) .
        error_steer = normalize_angle(angle - yaw); // (waypoint)   (yaw)   .
                                                    // ,       error_steer.                       
                                   


        //double steer_output;  //steer_output : PID     
          // TODO (Step 3)   ,     pid_steer    .
          // mithul12 




          /**
          * TODO (step 3): compute the steer error (error_steer) from the position and the desired trajectory
           TODO (Step 3):  (error_steer)  PID  
          **/
//           error_steer = 0;  //   (error_steer) 
                               //        (waypoint)   .
                              //     error_steer = 0; .

          /**
          * TODO (step 3): uncomment these lines
          **/
//           // Compute control to apply
           pid_steer.UpdateError(error_steer);   //PID   ,PID    ,    .
                                                  //UpdateError(error_steer) : PID      .
           steer_output = pid_steer.TotalError(); //TotalError() :    .
                                                    //  :  (error_steer)         
                                                    //error_steer = 0;  , PID     , mithul12  602  ,  

//          
// 
           // Save data                                    //  PID   ,PID     (steer_pid_data.txt) .
                                                                 //    ,       .
                                                                 // steer control  
           file_steer.seekg(std::ios::beg);
           for(int j=0; j < i - 1; ++j) {
               file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
           }
           file_steer  << i ;
           file_steer  << " " << error_steer;
           file_steer  << " " << steer_output << endl;
           //  pid_steer.UpdateError(error_steer); steer_output = pid_steer.TotalError();    

           

//  : TODO (Step 3)   (error_steer, steer_output)
// (error_steer)    PID    
//PID (pid_steer.UpdateError())       
//   , PID     
// :  PID   ! (error_steer)      ! 

      



/*
   PID    (throttle) (brake)  
        :  (error_throttle)     

*/
        ////////////////////////////////////////
          // Throttle control
          ////////////////////////////////////////

          /**
          * TODO (step 2): uncomment these lines
          **/
//           // Update the delta time with the previous command
           pid_throttle.UpdateDeltaTime(new_delta_time); //PID   (delta time)  .
                                                           //  PID      ( ).
                                                           //    ,    .
                                                           //464 pid_steer.Init(KP_STEER, KI_STEER, KD_STEER, MAX_STEER, MIN_STEER);
                                                           //467 pid_throttle.Init(KP_THROTTLE, KI_THROTTLE, KD_THROTTLE, MAX_THROTTLE, MIN_THROTTLE); 
                                                           // 464,467  
                                                          
           //  TODO (Step 2): (throttle)    
          // Compute error of speed
          //double error_throttle; //   
          double error_throttle = 0.0;  //   (  ) ,error_throttle :         
                                      //      error_throttle = 0;  
                                      //error_throttle = 0;     (     )
                                      //     ,      .
          /**
          * TODO (step 2): compute the throttle error (error_throttle) from the position and the desired speed
          **/
          // modify the following line for step 2
          // error_throttle = 0; // .. 
          error_throttle = v_points[closest_point_index] - velocity; //mithul12  ,     

          //double throttle_output;
          double throttle_output = 0.0;
          double brake_output;

          //* TODO (Step 2):  (error_throttle)  PID  
          /**
          * TODO (step 2): uncomment these lines
          **/
//           // Compute control to apply
             pid_throttle.UpdateError(error_throttle); //PID   (error_throttle)     .
             double throttle = pid_throttle.TotalError(); //PID    throttle ( ) .
                                                        //error_throttle       
                                                        //    ,     .
                                                        //    ( 687   ,error_throttle = v_points[closest_point_index] - velocity;    

//           
// 
//           // --      ( )
//           //throttle_output() brake_output( )  .
//           // Adapt the negative throttle to break
            //error_throttle = v_points[closest_point_index] - velocity;    

             if (throttle > 0.0) {                      //throttle()  (>0)  throttle_output  ()
                                                        //throttle()  (<0)  brake_output  ()
                                                        // throttle        .   
                                                        // (error_throttle)      . ( 687  )
              throttle_output = throttle;
             brake_output = 0;
             } else {
             throttle_output = 0;
             brake_output = -throttle;
           }  //      (    )
             // PID   ,  error_throttle, brake_output, throttle_output  throttle_pid_data.txt   .
             //   throttle      .
             //PID   ,   PID    .



           // Save data
           file_throttle.seekg(std::ios::beg);
           for(int j=0; j < i - 1; ++j){
               file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
           }
           file_throttle  << i ;
           file_throttle  << " " << error_throttle;
           file_throttle  << " " << brake_output;
           file_throttle  << " " << throttle_output << endl;
            //  PID   
//error_throttle( )       0  --> ( 682  )
//PID (pid_throttle.UpdateError())             
//throttle_output brake_output   PID    
// PID   !        





//  : WebSocket        
//        (steer, throttle, brake) WebSocket  ,      
          // Send control
          // JSON    
          // (brake, throttle, steer) JSON  
          json msgJson;
          msgJson["brake"] = brake_output; //brake_output :  
          msgJson["throttle"] = throttle_output; //throttle_output :  
          msgJson["steer"] = steer_output; //steer_output : (steering) 
          // WebSocket          

          

          //    (NaN :steer_output throttle_output    NaN (Not a Number)   . 
          //   0          )
          // : if (isnan(steer_output) || isnan(throttle_output))  NaN  ,   0 
          //name 'steer' is not defined   -->  steer_output        .  NaN  
          if (isnan(steer_output) || isnan(throttle_output)) {
            cout << "Error: NaN detected in steer or throttle output" << endl;
            steer_output = 0.0;
            throttle_output = 0.0;
          }

          //  
          //name 'steer' is not defined  WebSocket  JSON  steer     . 
          //   steer_output     
          cout << "Steer Output: " << steer_output << ", Throttle Output: " << throttle_output << endl;




          //  JSON  
          msgJson["trajectory_x"] = x_points; //trajectory_x, trajectory_y :   (x, y )
          msgJson["trajectory_y"] = y_points;
          msgJson["trajectory_v"] = v_points; //trajectory_v :    
          //        .


         //  (spiral paths) JSON  
          msgJson["spirals_x"] = spirals_x; //spirals_x, spirals_y :  (spiral path) x, y 
          msgJson["spirals_y"] = spirals_y;
          msgJson["spirals_v"] = spirals_v; //spirals_v :    
          msgJson["spiral_idx"] = best_spirals; //spiral_idx :    (best spiral)
          //       ,   .

          //     (maneuver) JSON  
          //  STOPPED, FOLLOW_LANE, DECELERATE_TO_STOP     .
          //           
          msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();




          //  min point threshold before doing the update
          // for high update rate use 19 for slow update rate use 4
             //  
          msgJson["update_point_thresh"] = 16; //      16   .
                                               //    ,     .
                                               //      .



          auto msg = msgJson.dump(); //JSON    WebSocket   .
                                      // WebSocket   JSON   .

          i = i + 1;
          file_steer.close(); //(steer_pid_data.txt, throttle_pid_data.txt) .
          file_throttle.close();
          //    ,      .



      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT); //WebSocket  JSON   
                                                            //ws.send()   msg.data()()  .
                                                            //   (, ,  )   .

    }

  });


  // (WebSocket)    
  //   
  //WebSocket     .
  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
      cout << "Connected!!!" << endl; //cout << "Connected!!!" << endl;     .
    }); 



 //    
 //     .
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
      ws.close(); //ws.close();  WebSocket  .
      cout << "Disconnected" << endl; //cout << "Disconnected" << endl;     .
    }); // WebSocket     .





    // WebSocket  
  int port = 4567; //WebSocket    4567   
  if (h.listen("0.0.0.0", port)) //h.listen("0.0.0.0", port) :    4567    .
    {
      cout << "Listening to port " << port << endl;
      h.run(); //h.run(); : WebSocket  .
    }
  else
    {
      cerr << "Failed to listen to port" << endl; //  (if  )  "Failed to listen to port"    .
      return -1;
    } // WebSocket  ,   .
//   (brake, throttle, steer)   JSON   WebSocket  .
//     , WebSocket  .
// update_point_thresh     .
//WebSocket   4567    .
}



/*
 1. pid_steer.UpdateDeltaTime(new_delta_time); (576)   
  :

pid_steer.Init(KP_STEER, KI_STEER, KD_STEER, MAX_STEER, MIN_STEER); (464)
pid_throttle.Init(KP_THROTTLE, KI_THROTTLE, KD_THROTTLE, MAX_THROTTLE, MIN_THROTTLE); (467)
 :

Init()   PID       .
 2. pid_steer.UpdateError(error_steer); / steer_output = pid_steer.TotalError(); (623, 625)   
  :

error_steer = normalize_angle(angle - yaw); (602)
int closest_point_index = find_closest_point(x_position, y_position, x_points, y_points); (600)
double angle = angle_between_points(x_position, y_position, x_points[closest_point_index], y_points[closest_point_index]); (601)
 :

 (error_steer)    PID     .
find_closest_point() angle_between_points()       .
 3. file_steer    (631~635)   
  :

pid_steer.UpdateError(error_steer); (623)
steer_output = pid_steer.TotalError(); (625)
 :

   steer_pid_data.txt      .
 4. pid_throttle.UpdateDeltaTime(new_delta_time); (668)   
  :

pid_throttle.Init(KP_THROTTLE, KI_THROTTLE, KD_THROTTLE, MAX_THROTTLE, MIN_THROTTLE); (467)
 :

PID    delta time  .
 5. pid_throttle.UpdateError(error_throttle); / double throttle = pid_throttle.TotalError(); (702, 703)   
  :

error_throttle = v_points[closest_point_index] - velocity; (687)
 :

 (error_throttle)    PID    
 6. throttle_output  brake_output   (718~722)   
  :

error_throttle = v_points[closest_point_index] - velocity; (687)
double throttle = pid_throttle.TotalError(); (703)
 :

   PID    /  .
 7. file_throttle    (727~733)   
  :

pid_throttle.UpdateError(error_throttle); (702)
double throttle = pid_throttle.TotalError(); (703)
throttle_output  brake_output   (718~722)
 :

PID /    throttle_pid_data.txt   .
  :
 PID (Init()), /  (error_steer, error_throttle)  ,  PID          ! 





*/