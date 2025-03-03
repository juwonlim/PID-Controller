//main(2월23일 백업_delete모든 주석).cpp
//너무 utf-8관련 오류가 많아서 대부분의 주석은 삭제했음
//나중에 복원해줄 것 , 현재는 프로젝트가 늦어서 일단 진행
//치명적인 utf-8오류는 void path_planner함수내에서 발생했는데 notepad++에서 일단 제거 했음

/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 				Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 * @brief 자율주행 차량의 주요 로직을 처리하는 메인 파일
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

#include "json.hpp" 
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
#include "behavior_planner_FSM.h" 
#include "motion_planner.h" 
#include "planning_params.h"  
#include "utils.h"    
#include "pid_controller.h" 

#include <limits>
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>  
#include <math.h>  
#include <vector>
#include <cmath>
#include <time.h>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

//  main.cpp가 주고받는 파일들
//  값을 가져오는 파일: `pid_controller.h` (PID 제어 클래스 사용)
//  값을 전달하는 파일: `pid_controller.cpp` (PID 제어 변수 업데이트)



// PID parameters (조향 및 가속도를 위한 PID 계수 설정)
//pid_controller.h나 planning_params.h에서 값이 정의될 수도 있으나 여기에 정의
//mithul12의 코드응용 () - PID 파라메터 추가

/* interation #1 : 차량이 인도로 올라가 버림
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
*/

/* iteration #2 : 차량이 앞차만나면 한참 정지, 한번은 회피성공후에 가로수 분리대로 올라간후 반대차선으로 넘어가버림
static const double KP_STEER = 0.2;
static const double KI_STEER = 0.0005;
static const double KD_STEER = 0.1;

static const double MAX_STEER = 1.2;
static const double MIN_STEER = -1.2;

static const double KP_THROTTLE = 0.1;
static const double KI_THROTTLE = 0.0005;
static const double KD_THROTTLE = 0.05;

static const double MAX_THROTTLE = 1.0;
static const double MIN_THROTTLE = -1.0;
*/

/* iteration #3 : 차선변경시 멈추는 시간은 짧아졌으나 역시 도로중앙 가로수 있는 곳으로 넘어가버림
//조향반응속도 개선 (문제 : 앞차를 피한 후 너무 큰 조향을 해서 방지턱을 넘어감)
static const double KP_STEER = 0.25;
static const double KI_STEER = 0.0008;
static const double KD_STEER = 0.12;

static const double MAX_STEER = 1.2;
static const double MIN_STEER = -1.2;

//가속반응속도 개선(문제 : 앞차를 피하는데 너무 오래정지한다)
static const double KP_THROTTLE = 0.12;
static const double KI_THROTTLE = 0.0008;
static const double KD_THROTTLE = 0.03;

//아래 값은 아직 조정변수가 아님
static const double MAX_THROTTLE = 1.0;
static const double MIN_THROTTLE = -1.0;
*/


/*  iteration #4
static const double KP_STEER = 0.22; //조향강도 살짝 줄임
static const double KI_STEER = 0.001; //조향 오차가 누적될 경우 조금 더 정밀하게 보정하도록 만듦.
static const double KD_STEER = 0.15; // 빠르게 조향하려는 움직임을 감쇠하여 조향이 너무 크지 않도록 만듦.

static const double MAX_STEER = 1.2;
static const double MIN_STEER = -1.2;

//가속반응속도 개선
static const double KP_THROTTLE = 0.15; //0.15로 증가 ,장애물을 피할 때 너무 느려지지 않도록 가속도를 조금 더 높임
static const double KI_THROTTLE = 0.0008;
static const double KD_THROTTLE = 0.05; //0.05로 증가, 속도 변화가 너무 급격하지 않도록 조정 (느려졌다가 빠르게 가속하는 문제 방지)

//아래 값은 아직 조정변수가 아님
static const double MAX_THROTTLE = 1.0;
static const double MIN_THROTTLE = -1.0;
*/


/* iteration #5 
// 남은 문제 : 
// 1.첫 번째 앞차를 피해 차선 변경 중 멈칫하는 현상
// 2. 두 번째 앞차를 피해 우측 차선이 아니라 중앙분리대 쪽으로 진입
// 1,2번 해결 필요          
static const double KP_STEER = 0.24; //0.24로 증가시킴, 더 빠르게 차선변경 
static const double KI_STEER = 0.0008; //0.0008로 감소,오차 보정을 줄여 불필요한 조향이 발생하지 않도록 함
static const double KD_STEER = 0.12; //0.12로 감소, 과도한 핸들 반응을 방지하고 부드러운 움직임 유도

//MAX_STEER와 MIN_STEER을 1.0으로 조정
//차량이 너무 급격하게 조향하지 않도록 제한
// +1.0, -1.0으로 좌우 동일하게 균형유지
static const double MAX_STEER = 1.0; 
static const double MIN_STEER = -1.0;

//가속반응속도 개선
static const double KP_THROTTLE = 0.13;  //KP_THROTTLE 값을 0.13으로 감소, 차선 변경 중 가속이 과하게 이루어지는 문제 방지
static const double KI_THROTTLE = 0.0008;
static const double KD_THROTTLE = 0.06; // KD_THROTTLE 값을 0.06으로 증가, 속도 변화가 너무 급격하지 않도록 보정하여 차선 변경을 자연스럽게 유지


//급가속 발생우려로 아래 2개의 파라메터는 이번은 수정안함
static const double MAX_THROTTLE = 1.0;
static const double MIN_THROTTLE = -1.0;
*/



/* 
// iteration #6 
static const double KP_STEER = 0.26; // KP_STEER = 0.26 (+0.02 증가) → 차선 변경 반응 속도 증가
static const double KI_STEER = 0.0006;  //KI_STEER = 0.0006 (-0.0002 감소) → 조향 오차 누적 완화
static const double KD_STEER = 0.10; //KD_STEER = 0.10 (-0.02 감소) → 조향값 튀는 현상 억제


static const double MAX_STEER = 0.9; //MAX_STEER = 0.9 (-0.1 감소) → 급격한 조향 억제
static const double MIN_STEER = -0.9;

//가속반응속도 개선
static const double KP_THROTTLE = 0.14;  //KP_THROTTLE = 0.14 (+0.01 증가) → 가속 반응 조금 더 빠르게
static const double KI_THROTTLE = 0.0007; //KI_THROTTLE = 0.0007 (-0.0001 감소) → 속도 오차 누적 줄임
static const double KD_THROTTLE = 0.05; //KD_THROTTLE = 0.05 (-0.01 감소) → 급격한 속도 변화 방지


static const double MAX_THROTTLE = 1.0;
static const double MIN_THROTTLE = -1.0;
*/



// iteration #7
static const double KP_STEER = 0.3; // 조향 반응 속도 증가
static const double KI_STEER = 0.0006;  // 기존 유지
static const double KD_STEER = 0.08;  // 조향값 튀는 현상 완화

static const double MAX_STEER = 0.7;  // 조향 최대값 줄임 (급격한 조향 방지)  
static const double MIN_STEER = -0.7; //좌우 동일

//가속반응속도 개선
static const double KP_THROTTLE = 0.18;  // 가속 반응 증가  
static const double KI_THROTTLE = 0.0007; // 기존 유지  
static const double KD_THROTTLE = 0.04;  // 급격한 가속 완충 

static const double MAX_THROTTLE = 1.0; // 기존 유지
static const double MIN_THROTTLE = -1.0; //기존 유지






/**
 * @brief JSON 데이터에서 필요한 정보를 추출하는 함수

 */
string hasData(string s) { 
  auto found_null = s.find("null");  
    auto b1 = s.find_first_of("{");  
    auto b2 = s.find_first_of("}");   
    if (found_null != string::npos) {  
      return ""; 
    }
    else if (b1 != string::npos && b2 != string::npos) { 
      return s.substr(b1, b2 - b1 + 1); 
    }
    return ""; 
}

/**
sgn(T val) 함수의 의미 : 이 함수는 숫자의 부호(Sign)를 반환하는 역할을 수행
                        즉, 양수는 1, 음수는 -1, 0이면 0을 반환하는 함수임.
 * @brief 숫자의 부호를 반환하는 함수 (양수: 1, 음수: -1, 0: 0)
 */
template <typename T>  
int sgn(T val) { 
    return (T(0) < val) - (val < T(0)); 
                                      
}                                       

/**
 * @brief  
           
 */
double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1); 
}


// 행동 계획 및 경로 계획을 위한 객체 생성

BehaviorPlannerFSM behavior_planner( 

      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

// 이 변수들은 장애물 데이터를 저장하고 관리하는 역할
bool have_obst = false; 
vector<State> obstacles; 

//여기까지 #1


/**
 * @brief 
 * @note 스타터 코드 원본 유지
 */
//멘토가 주신 path_planeer 함수
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
} //멘토가 주신 path_planeer 함수의 끝부분





/**
 * @brief 장애물 데이터 설정 함수 (스타터 코드 원본 유지)
 */

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag){

	for( int i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i]; 
		obstacle.location.y = y_points[i]; 
		obstacles.push_back(obstacle); 
	}
	obst_flag = true; 
}
//여기까지 #2


//누락디었던 normalize_angle추가
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

//누락되었던 find_closet_point추가
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
 * @brief 메인 실행 함수
  * @note WebSocket을 통해 자율주행 차량의 데이터를 실시간으로 송수신
 */

 //main() 함수 (자율주행 차량의 실행 시작점),이 코드는 WebSocket을 통해 자율주행 차량 데이터를 송수신하는 서버를 초기화하고 실행하는 역할
int main ()
{
  cout << "starting server" << endl; 
  uWS::Hub h;  
  double new_delta_time;
  int i = 0;


   // PID 제어기 데이터 저장 파일 초기화 (steer_pid_data.txt, throttle_pid_data.txt 두 개의 파일을 초기화)
   //→ PID(비례-적분-미분) 제어기의 조향 및 가속 데이터 저장
  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc); 
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();


  // 시간 측정을 위한 변수 설정
  time_t prev_timer; //prev_timer, timer → 실행 시간 측정을 위한 변수
  time_t timer;
  time(&prev_timer); 


  // initialize pid steer
  /**
  * TODO (Step 1): create pid (pid_steer) for steer command and initialize values
  **/


  // initialize pid throttle
  /**
  * TODO (Step 1): create pid (pid_throttle) for throttle command and initialize values
  **/


 // TODO (Step 1): PID 제어기 생성 및 초기화
  PID pid_steer = PID(); 
  pid_steer.Init(KP_STEER, KI_STEER, KD_STEER, MAX_STEER, MIN_STEER); //mithul12의 코드응용

  PID pid_throttle = PID(); 
  pid_throttle.Init(KP_THROTTLE, KI_THROTTLE, KD_THROTTLE, MAX_THROTTLE, MIN_THROTTLE); //mithul12의 코드응용







/*
아래 코드는 WebSocket 이벤트 핸들러(h.onMessage()) 내부에서 실행되며, 차량의 주행 데이터를 받아 **경로를 계획(path_planner 호출)**하는 역할 수행
즉, 자율주행 차량과 서버 간의 실시간 데이터 송수신 및 처리를 담당
*/

  h.onMessage([&pid_steer, &pid_throttle, &new_delta_time, &timer, &prev_timer, 
    &i, &prev_timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)  
  {
        auto s = hasData(data);  

        if (s != "") {

          auto data = json::parse(s); 


          // create file to save values
          // PID 제어 데이터를 저장할 파일 열기
          fstream file_steer; 
          file_steer.open("steer_pid_data.txt"); 
          fstream file_throttle; 
          file_throttle.open("throttle_pid_data.txt"); 

          
          // 차량의 주행 데이터 받아오기
          vector<double> x_points = data["traj_x"]; 
          vector<double> y_points = data["traj_y"];
          vector<double> v_points = data["traj_v"]; 
          double yaw = data["yaw"]; 
          double velocity = data["velocity"]; 
          double sim_time = data["time"]; 
          double waypoint_x = data["waypoint_x"]; 
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"]; 
          string tl_state = data["tl_state"]; 

          double x_position = data["location_x"]; 
          double y_position = data["location_y"];
          double z_position = data["location_z"];
          

          // 장애물 데이터 처리
          if(!have_obst){
          	vector<double> x_obst = data["obst_x"]; 
          	vector<double> y_obst = data["obst_y"];
          	set_obst(x_obst, y_obst, obstacles, have_obst); 
          } 
         

          // 목표 위치 설정
          //goal 객체를 생성하여 차량이 이동해야 할 목표 위치(waypoint)를 설정.
          //waypoint_x, waypoint_y, waypoint_t 값을 사용하여 목표 상태를 정의
          State goal;
          goal.location.x = waypoint_x;
          goal.location.y = waypoint_y;
          goal.rotation.yaw = waypoint_t;

           // 경로 계획 수행
           //경로 계획을 수행하기 위해 path_planner() 함수를 호출.
           //경로 생성 결과를 저장할 벡터 선언
          vector< vector<double> > spirals_x; 
          vector< vector<double> > spirals_y;
          vector< vector<double> > spirals_v;
          vector<int> best_spirals; 

          
          path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);



          // Save time and compute delta time
          time(&timer); 


          new_delta_time = difftime(timer, prev_timer); 
          prev_timer = timer; 

       

         
          ////////////////////////////////////////
          // Steering control
          ////////////////////////////////////////

          /**
          * TODO (step 3): uncomment these lines
          **/
          // Update the delta time with the previous command
           pid_steer.UpdateDeltaTime(new_delta_time);  


           // TODO (Step 3): 조향(steer) 제어 적용
          // Compute steer error
        
        
        double steer_output; //mithul12의 코드 적용인줄 알았는데 원래 있던 스타터 코드, mithul12가 위치만 바꾼것, 아래의 중복코드는 주석처리
        //double steer_output = 0.0;                   
        
        double error_steer; //스타터 코드 원본
        //double error_steer = 0.0;  
                                 
 
        //아래 3줄의 코드 ,mithul12의 코드응용                             
        int closest_point_index = find_closest_point(x_position, y_position, x_points, y_points); 
        double angle = angle_between_points(x_position, y_position, x_points[closest_point_index], y_points[closest_point_index]); 
        error_steer = normalize_angle(angle - yaw);                       
        //여기까지 mithul12의 코드응용

          /**
          * TODO (step 3): compute the steer error (error_steer) from the position and the desired trajectory
           TODO (Step 3): 조향 오차(error_steer)를 계산하여 PID 입력으로 전달
          **/
          // error_steer = 0;  //주석 처리된 조향 오차(error_steer) 계산
                            

          /**
          * TODO (step 3): uncomment these lines
          **/
        // Compute control to apply
           pid_steer.UpdateError(error_steer);   
           steer_output = pid_steer.TotalError(); 
                                                   

           file_steer.seekg(std::ios::beg);
           for(int j=0; j < i - 1; ++j) {
               file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
           }
           file_steer  << i ;
           file_steer  << " " << error_steer;
           file_steer  << " " << steer_output << endl;
    

           

// 여기까지 : TODO (Step 3)에서 변수 선언 (error_steer, steer_output)
//조향 오차(error_steer) 계산이 구현되지 않아 PID 적용 코드가 주석 처리됨
//PID 제어기(pid_steer.UpdateError())를 호출하려면 먼저 오차 계산 로직을 추가해야 함
//현재는 단순히 변수만 정의하고, PID 제어기 로직을 나중에 추가할 예정
// 결론: 현재는 PID 적용 전 상태! 오차(error_steer) 계산이 구현되면 주석을 해제하고 활성화해야 함! 

      
/*
아래의 코드는  PID 제어기를 이용해 차량의 가속(throttle)과 감속(brake)을 조절하는 역할
그런데 스타터 코드에서 주석 처리된 부분이 많음 → 이유: 속도 오차(error_throttle)를 계산하는 코드가 아직 없기 때문

*/
        ////////////////////////////////////////
          // Throttle control
          ////////////////////////////////////////

          /**
          * TODO (step 2): uncomment these lines
          **/
//           // Update the delta time with the previous command
           pid_throttle.UpdateDeltaTime(new_delta_time); 
                                                          
           //TODO (Step 2): 가속(throttle) 및 브레이크 제어 적용
          // Compute error of speed
          //double error_throttle; //스타터 코드 원본 
          double error_throttle = 0.0;  
                                     
          /**
          * TODO (step 2): compute the throttle error (error_throttle) from the position and the desired speed
          **/
          // modify the following line for step 2
          // error_throttle = 0; //
          error_throttle = v_points[closest_point_index] - velocity; //mithul12의 코드응용 

          //double throttle_output;
          double throttle_output = 0.0;
          double brake_output;

          //* TODO (Step 2): 속도 오차(error_throttle)를 계산하여 PID 입력으로 전달
          /**
          * TODO (step 2): uncomment these lines
          **/
         // Compute control to apply
             pid_throttle.UpdateError(error_throttle); 
             double throttle = pid_throttle.TotalError(); 
                                                        
                                                      
             if (throttle > 0.0) {                      
                                                        
              throttle_output = throttle;
             brake_output = 0;
             } else {
             throttle_output = 0;
             brake_output = -throttle;
           }  //여기까지 브레이크 및 가속도 설정 코드 (원래 스타터코드에서 주석 처리되었었는데 해제함)
             



           // Save data
           file_throttle.seekg(std::ios::beg);
           for(int j=0; j < i - 1; ++j){
               file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
           }
           file_throttle  << i ;
           file_throttle  << " " << error_throttle;
           file_throttle  << " " << brake_output;
           file_throttle  << " " << throttle_output << endl;
            // 여기까지 PID 값 저장 코드



          // Send control
          // JSON 데이터 생성 및 전송
          //제어 신호(brake, throttle, steer)를 JSON 객체에 저장
          json msgJson;
          msgJson["brake"] = brake_output; 
          msgJson["throttle"] = throttle_output; 
          msgJson["steer"] = steer_output; 
        

          

          // 유효성 검사 추가 (NaN체크 :steer_output이나 throttle_output 값이 계산 중에 NaN (Not a Number)이 될 가능성이 있음. 
          //name 'steer' is not defined 에러가 발생 --> 이는 steer_output 값이 정의되지 않거나 잘못된 데이터를 전달했을 가능성을 나타냄. 따라서 NaN 체크는 필요
          if (isnan(steer_output) || isnan(throttle_output)) {
            cout << "Error: NaN detected in steer or throttle output" << endl;
            steer_output = 0.0;
            throttle_output = 0.0;
          }

         
          //name 'steer' is not defined 에러는 WebSocket으로 전송되는 JSON 데이터에서 steer 필드가 빠졌거나 잘못 전달되었음을 의미. 
          cout << "Steer Output: " << steer_output << ", Throttle Output: " << throttle_output << endl;




          //경로 데이터를 JSON 객체에 저장
          msgJson["trajectory_x"] = x_points; 
          msgJson["trajectory_y"] = y_points;
          msgJson["trajectory_v"] = v_points; 
       


         //생성된 주행 경로(spiral paths)를 JSON 객체에 저장
          msgJson["spirals_x"] = spirals_x; 
          msgJson["spirals_y"] = spirals_y;
          msgJson["spirals_v"] = spirals_v; 
          msgJson["spiral_idx"] = best_spirals; 
     

          //현재 차량이 수행 중인 행동 상태(maneuver)를 JSON 객체에 저장
          msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();

          //  min point threshold before doing the update
          // for high update rate use 19 for slow update rate use 4
             // 주기적인 업데이트
          msgJson["update_point_thresh"] = 16; 


          auto msg = msgJson.dump(); 

          i = i + 1;
          file_steer.close(); 
          file_throttle.close();
        

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT); 

    }

  });


  //여기서부터 클라이언트(WebSocket) 연결 및 해제 관리
  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
      cout << "Connected!!!" << endl; 
    }); 



 // 클라이언트 연결 해제 이벤트
 //클라이언트가 연결을 끊었을 때 실행되는 코드.
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
      ws.close(); 
      cout << "Disconnected" << endl; 
    }); 





    // WebSocket 서버 실행
  int port = 4567; 
  if (h.listen("0.0.0.0", port)) 
    {
      cout << "Listening to port " << port << endl;
      h.run(); 
    }
  else
    {
      cerr << "Failed to listen to port" << endl; 
      return -1;
    } 

}



/*
 1. pid_steer.UpdateDeltaTime(new_delta_time); 주석 해제 이유
추가된 코드:

pid_steer.Init(KP_STEER, KI_STEER, KD_STEER, MAX_STEER, MIN_STEER); 
pid_throttle.Init(KP_THROTTLE, KI_THROTTLE, KD_THROTTLE, MAX_THROTTLE, MIN_THROTTLE); 
설명: Init() 함수를 통해 PID 컨트롤러가 올바르게 초기화되었기 때문에 주석 해제 가능해짐.

2. pid_steer.UpdateError(error_steer); / steer_output = pid_steer.TotalError();  주석 해제 이유
추가된 코드: 
error_steer = normalize_angle(angle - yaw); 
int closest_point_index = find_closest_point(x_position, y_position, x_points, y_points); 
double angle = angle_between_points(x_position, y_position, x_points[closest_point_index], y_points[closest_point_index]); 

설명: 조향 오차(error_steer) 계산이 구현되었기 때문에 PID 컨트롤러가 정상적으로 동작할 수 있음.
find_closest_point()와 angle_between_points()를 통해 조향 오차가 계산되므로 주석 해제 가능.

3. file_steer 데이터 저장 코드 주석 해제 이유
추가된 코드:
pid_steer.UpdateError(error_steer); 
steer_output = pid_steer.TotalError(); 
설명:  조향 제어값이 계산되었으므로 steer_pid_data.txt에 저장하는 코드가 정상적으로 동작할 수 있음.


4. pid_throttle.UpdateDeltaTime(new_delta_time);  주석 해제 이유
추가된 코드:
pid_throttle.Init(KP_THROTTLE, KI_THROTTLE, KD_THROTTLE, MAX_THROTTLE, MIN_THROTTLE); 
설명: PID 컨트롤러가 초기화되었기 때문에 delta time 업데이트가 가능해짐.

5. pid_throttle.UpdateError(error_throttle); / double throttle = pid_throttle.TotalError();  주석 해제 이유
추가된 코드: error_throttle = v_points[closest_point_index] - velocity; 
설명:  속도 오차(error_throttle) 계산이 구현되었기 때문에 PID 컨트롤러가 정상 동작 가능

6. throttle_output 및 brake_output 설정 코드 주석 해제 이유
추가된 코드: 
error_throttle = v_points[closest_point_index] - velocity;
double throttle = pid_throttle.TotalError(); 
설명: 속도 오차 및 PID 계산 결과가 나오므로 가속/감속 조정이 가능해짐.


7. file_throttle 데이터 저장 코드 주석 해제 이유
추가된 코드: 
pid_throttle.UpdateError(error_throttle); 
double throttle = pid_throttle.TotalError();
throttle_output 및 brake_output 설정 코드 
설명:
PID 가속/감속 값이 정상적으로 계산되었으므로 throttle_pid_data.txt 파일에 저장 가능.
최종 결론:
PID 초기화(Init()), 조향/속도 오차 계산(error_steer, error_throttle)이 추가되었기 때문에, 관련 PID 제어 코드와 로그 저장 코드가 정상적으로 주석 해제될 수 있었음!


*/