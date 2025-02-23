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

#include "json.hpp" // JSON 데이터를 처리하기 위한 라이브러리
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
#include "behavior_planner_FSM.h" // 자율주행 차량의 행동 계획을 담당하는 파일
#include "motion_planner.h" // 경로 계획을 담당하는 파일
#include "planning_params.h"  // 주행 계획에 필요한 파라미터 설정 파일
#include "utils.h"    // 보조 유틸리티 함수들
#include "pid_controller.h"  // PID 제어기 관련 파일 (main.cpp에서 사용됨)

#include <limits>
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>  // WebSocket 라이브러리
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
 * @brief JSON 데이터에서 필요한 정보를 추출하는 함수

 hasData(string s) 함수의 의미 : 이 함수는 JSON 데이터를 분석해서 필요한 부분만 추출하는 역할 수행
                                 자율주행 차량이 서버(WebSocket)로부터 받은 데이터가 JSON 형식인지 확인하고, 올바른 JSON 데이터를 반환하는 기능을 함.
 */
string hasData(string s) { // 문자열 `s`를 입력으로 받음
  auto found_null = s.find("null");  // 문자열 `s`에서 "null"을 찾음
    auto b1 = s.find_first_of("{");  // `{` (JSON 시작) 위치 찾기
    auto b2 = s.find_first_of("}");   // `}` (JSON 끝) 위치 찾기
    if (found_null != string::npos) {  // "null"이 있으면
      return ""; // 빈 문자열 반환 (JSON 데이터가 없음)
    }
    else if (b1 != string::npos && b2 != string::npos) { // `{` 와 `}`가 있으면
      return s.substr(b1, b2 - b1 + 1); // `{` 부터 `}`까지 JSON 문자열을 추출하여 반환
    }
    return ""; // JSON 데이터가 없으면 빈 문자열 반환
}

/**
sgn(T val) 함수의 의미 : 이 함수는 숫자의 부호(Sign)를 반환하는 역할을 수행
                        즉, 양수는 1, 음수는 -1, 0이면 0을 반환하는 함수임.
 * @brief 숫자의 부호를 반환하는 함수 (양수: 1, 음수: -1, 0: 0)
 */
template <typename T>  // 템플릿 함수: 어떤 데이터 타입(T)이든 사용할 수 있음 즉,이 함수는 int, double, float 등 다양한 숫자 타입을 받을 수 있도록 설계된 템플릿 함수
                       //즉, 정수(int)뿐만 아니라 실수(double, float)도 처리 가능.
int sgn(T val) { // 숫자 val을 입력으로 받음 ---> (sgn(T val) 함수는 숫자의 부호를 반환하는 함수임.)
    return (T(0) < val) - (val < T(0)); //동작방식 : 이 함수의 핵심은 비교 연산자의 결과가 true(1) 또는 false(0)이 된다는 점
                                       // T(0) < val  (val이 0보다 크면 true(1), 아니면 false(0))
                                       //val < T(0) (val이 0보다 작으면 true(1), 아니면 false(0))
                                       //(T(0) < val) - (val < T(0)) ----> 의미는  0보다 크면 1 - 0 = 1 / 0이면 0 - 0 = 0 / 0보다 작으면 0 - 1 = -1
                                      //양수면 1, 음수면 -1, 0이면 0을 반환.
}                                       

/**
 * @brief  이 함수는 두 점 (x1, y1)과 (x2, y2) 사이의 각도를 계산하는 함수
           자율주행 시스템에서 차량의 진행 방향을 계산하거나, 목표 지점과의 상대적인 각도를 구하는 데 사용
           기본적으로 차량의 진행 방향을 계산하는 데 활용됨.
           차량이 어디를 향하고 있는지 확인하는 데 필수적인 함수.
           path_planner() 함수 내부에서 사용됨
           
 */
double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1); //atan2(y2 - y1, x2 - x1)는 점 (x1, y1)에서 (x2, y2) 방향의 라디안 각도를 반환하는 함수
}


// 행동 계획 및 경로 계획을 위한 객체 생성
//자율주행 차량의 행동 계획(Behavior Planning)을 담당하는 BehaviorPlannerFSM 클래스의 인스턴스(객체)
//즉, 차량이 멈출지, 진행할지, 감속할지 등의 결정을 내리는 역할

// 행동 계획 및 경로 계획을 위한 객체 생성
//이 객체는 차량이 언제 감속해야 하고, 멈춰야 하고, 속도를 유지해야 하는지 판단하는 데 쓰임
//BehaviorPlannerFSM 클래스는 behavior_planner_FSM.h 및 behavior_planner_FSM.cpp 파일에서 정의됨.
BehaviorPlannerFSM behavior_planner( //BehaviorPlannerFSM 클래스의 객체 behavior_planner를 생성 -->이 객체는 path_planner() 함수 내부에서 사용됨.

      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
//이 객체(motion_planner)는 motion_planner.cpp에서 정의된 MotionPlanner 클래스의 인스턴스 --> 경로 생성 및 최적 경로 선택을 수행하는 데 사용됨
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

// 이 변수들은 장애물 데이터를 저장하고 관리하는 역할
//이 변수들은 set_obst() 함수에서 장애물 데이터를 업데이트하는 데 사용됨
bool have_obst = false; //장애물이 있는지 여부를 나타냄 (초기값: false)
vector<State> obstacles; //장애물들의 위치 및 상태 정보를 저장하는 벡터

//여기까지 #1


/**
 * @brief 자율주행 차량의 최적 경로를 생성하는 함수, 
         차량의 현재 상태(위치, 속도, 방향)**를 바탕으로 주행 경로를 계획
        장애물, 신호등, 도로 상황을 고려하여 최적의 경로를 결정
        출력 데이터를 main.cpp 내에서 사용하지만, 내부적으로 다른 파일(motion_planner.cpp)의 함수를 호출하여 경로를 계산

   motion_planner.cpp -->  motion_planner.generate_spirals(), motion_planner.get_best_spiral_idx() 호출하여 최적의 경로를 계산
   behavior_planner_FSM.cpp --> behavior_planner.get_active_maneuver()를 호출하여 차량의 행동 결정
   main.cpp --> path_planner()를 직접 호출하여 WebSocket을 통해 차량에 경로를 전달
   ####결과적으로 main.cpp에서 호출되어 차량의 주행 데이터를 실시간으로 업데이트하는 역할을 함!!!!
 * @note 스타터 코드 원본 유지
 */

 /* */
void path_planner(
  vector<double>& x_points, //main.cpp에서 차량의 X 좌표 데이터를 전달 --> 용도 : 주행 경로 X 좌표
  vector<double>& y_points, //주행 경로 Y 좌표
  vector<double>& v_points,  //main.cpp에서 차량의 속도 데이터를 전달 --> 용도 :주행 경로 속도
  double yaw,  //main.cpp에서 차량의 방향(각도) 데이터를 전달 --> 용도 :차량의 현재 방향
  double velocity, // main.cpp에서 차량의 현재 속도를 전달 --->용도 : 차량의 현재 속도
  State goal, // 구조체 ,behavior_planner_FSM.cpp에서 계산된 목표 지점 정보 --> 용도 : 차량의 목표 위치
  bool is_junction, //불리언 값, main.cpp에서 신호등, 교차로 여부를 전달 --> 용도 : 교차로 여부 판단
  string tl_state, //문자열, main.cpp에서 신호등 상태를 전달 ---> 용도 : 신호등 정보
  vector< vector<double> >& spirals_x, //2D 벡터, motion_planner.cpp에서 생성된 여러 개의 X 좌표 경로 -->용도 :여러 개의 주행 경로 (X)
  vector< vector<double> >& spirals_y, //여러 개의 주행 경로 (Y)
  vector< vector<double> >& spirals_v, //여러 개의 주행 경로 (속도)
  vector<int>& best_spirals){  //벡터,motion_planner.cpp에서 최적의 주행 경로 인덱스 전달 --> 용도 : 선택된 최적의 경로

  State ego_state; //ego_state는 자율주행 차량(Ego Vehicle)의 현재 상태를 저장하는 구조체(State) 변수
                   //차량의 위치, 속도, 회전 방향(yaw) 같은 중요한 정보를 담고 있음
                   //State 구조체는 utils.h 또는 관련 헤더 파일에서 정의되어 있음
                   // 이 구조체는 차량의 위치, 속도, 방향 데이터를 저장하는 역할수행
                   //path_planner() 내부에서 차량의 위치, 속도, 방향을 업데이트하는 데 사용됨
                   //차량이 어떻게 이동할지를 결정하는 데 필수적인 정보임

  
  //이 코드는 ego_state(현재 차량 상태) 변수에 차량의 최신 위치와 속도를 저장하는 역할
  //차량의 X 좌표, Y 좌표, 속도(velocity)를 ego_state에 업데이트
  // ego_state에 값 저장하는 이유 --> 차량의 최신 상태를 저장해서 다른 함수에서 활용할 수 있도록 하기 위해 , 
  // 예를 들어, path_planner()는 이 정보를 이용해 경로를 생성하고, motion_planner는 속도를 조정할 수 있음.
  ego_state.location.x = x_points[x_points.size()-1]; //x_points는 차량이 지나온 X 좌표 값들을 저장한 벡터(vector)
                                                      //x_points.size()-1는 벡터의 마지막 값(가장 최근 위치)
                                                      //즉, 차량의 최신 X 좌표를 ego_state.location.x에 저장
  ego_state.location.y = y_points[y_points.size()-1]; //Y좌표 값에 대해서 위와 같음
  ego_state.velocity.x = velocity; // velocity는 현재 차량의 속도 값(초당 m/s),이 값을 ego_state.velocity.x에 저장하여 현재 속도를 기록




  //이 코드는 차량의 방향(yaw)을 계산하고, 속도를 업데이트하는 역할
  //차량이 최소한 2개 이상의 위치 데이터를 가지고 있을 때, 최근 두 점을 이용해 이동 방향(각도)을 계산함.
  if( x_points.size() > 1 ){ // 차량의 위치 데이터가 최소 2개 이상 있어야 방향을 계산 가능
                            //x_points.size() > 1 조건을 체크하는 이유: 차량이 이동하려면 최소한 2개 이상의 위치 정보가 필요하기 때문
                                                                      //만약 1개뿐이면 방향을 계산할 수 없음.
  	ego_state.rotation.yaw = angle_between_points( //최근 두 점을 이용해 차량의 방향(각도)을 계산함.
      x_points[x_points.size()-2], y_points[y_points.size()-2], // 이전 위치
      x_points[x_points.size()-1], y_points[y_points.size()-1] // 현재 위치
      // 이전 좌표 (x_n-2, y_n-2) → 현재 좌표 (x_n-1, y_n-1) 방향을 계산하여 yaw(방향 각도)를 설정.
      //결국, 차량이 이동하는 방향(라디안 값)이 yaw에 저장됨.
    );
  	
    ego_state.velocity.x = v_points[v_points.size()-1];  // 최근 속도 업데이트
                                                        //v_points는 차량 속도 벡터.
                                                        //가장 최신 속도(v_points[v_points.size()-1])를 가져와 ego_state.velocity.x에 저장.
                                                        //즉, 현재 차량의 속도를 ego_state에 반영.
  	if(velocity < 0.01) // 차량이 거의 정지 상태라면
  		ego_state.rotation.yaw = yaw; // 기존 yaw 값 유지 ,즉, 차량이 움직이지 않으면 새로운 방향을 계산하지 않음

  }
//이 코드는 자율주행 차량의 행동을 결정(가속, 감속, 정지 등)하고, 차량이 멈춰야 할 경우 정지 경로를 생성하는 역할
//behavior_planner를 사용해 차량의 현재 상태를 확인하고, 차량이 멈춰야 하면 정지 경로를 설정함.
// get_active_maneuver() 함수는 behavior_planner_FSM.h에서 Inline으로 정의되어 있음,현재 차량의 주행 상태(FOLLOW_LANE, STOPPED 등)를 반환하는 역할을 함.
//get_active_maneuver() 함수는 main.cpp에서 behavior_planner.get_active_maneuver();로 호출됨
  Maneuver behavior = behavior_planner.get_active_maneuver(); //차량의 현재 주행 상태를 반환

  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state); //  차량의 목표(goal) 상태를 업데이트함.
                                                               //현재 차량의 상태(ego_state), 교차로 여부(is_junction), 신호등 상태(tl_state)를 이용해 새로운 goal을 설정
                                                                //즉, 차량이 앞으로 어디로 가야 하는지를 결정함.
                                                                //즉,신호등이 빨간불이면 goal이 멈추는 위치로 설정,   앞차가 속도를 줄이면 goal이 감속하도록 변경
  if(behavior == STOPPED){ //차량이 정지해야 하는 경우(STOPPED 상태)라면, 현재 위치에서 정지하는 경로를 생성

  	int max_points = 20;
    //현재 위치(x_points, y_points)를 가져옴.
  	double point_x = x_points[x_points.size()-1]; //차량이 정지할 때, 현재 위치에서 멈춰야 하므로 마지막 위치 값(x_points.size()-1)을 사용
  	double point_y = y_points[x_points.size()-1];
    

    //차량이 현재 위치에서 멈춰 있도록 정지 경로를 생성.
    //x_points, y_points에 현재 위치를 계속 추가해서 멈춘 상태 유지.
    //v_points.push_back(0); → 속도를 0으로 설정하여 정지 상태 유지.
    // 이렇게 하면 차량이 더 이상 앞으로 가지 않고 현재 위치에서 멈추게 됨
  	while( x_points.size() < max_points ){
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);

  	}
  	return; //차량이 정지해야 하는 상황이면 여기서 함수 종료 --> 더 이상 주행 경로를 생성하지 않음
  }



  //차량의 주행 경로를 생성하고, 속도를 결정하는 역할
  //motion_planner 객체를 사용해 가능한 여러 개의 주행 경로를 생성한 후, 최적의 경로를 선택
  //utils::magnitude(goal.velocity)를 사용해 목표 속도를 계산
  auto goal_set = motion_planner.generate_offset_goals(goal); //목표 지점(goal)을 기반으로 여러 개의 대체 목표 지점(Goal Set)을 생성
                                                              //차량이 도로의 여러 위치를 고려하면서 최적의 목표 지점을 찾을 수 있도록 도와줌
                                                              //goal이 도로의 중앙이라면, 차선 변경을 고려하여 좌우로 약간 다른 목표 지점을 여러 개 생성할 수 있음
                                                              //목표 지점을 여러 개 생성하여 goal_set에 저장

  auto spirals = motion_planner.generate_spirals(ego_state, goal_set); //현재 차량 상태(ego_state)와 목표 지점(goal_set)을 기반으로 여러 개의 주행 경로(spirals)를 생성
                                                                      // 각각의 spiral은 차량이 이동할 수 있는 하나의 가능 경로를 의미함.
                                                                      // goal_set에 여러 개의 목표 지점이 있다면, 각 목표 지점까지 가는 여러 개의 곡선(spiral) 경로를 생성함.
                                                                      // motion_planner.generate_spirals(ego_state, goal_set); → 각 목표 지점까지 가는 다양한 경로를 spirals에 저장
                                                                      //결론: 차량이 이동할 수 있는 여러 개의 경로를 만들고, 이후에 최적의 경로를 선택하는 과정임
  auto desired_speed = utils::magnitude(goal.velocity); //목표 속도를 계산하는 코드
                                                        // 목표 지점(goal)에서 차량이 유지해야 할 속도를 계산하여 desired_speed에 저장
                                                        // goal.velocity는 State 구조체의 속도 정보
                                                        // utils::magnitude(goal.velocity)는 목표 속도의 크기(벡터의 크기)를 반환
                                                        // 즉, 차량이 해당 목표 지점에 도달할 때 유지해야 할 목표 속도를 계산하는 역할


  // 스타터 코드 원본 유지 (mithul12 방식 적용 X)
  //자율주행 차량이 경로를 생성하는 과정에서, 앞차(lead car)의 상태를 추적하고, 유효한 주행 경로(spirals)가 없을 경우 오류를 출력하는 역할
  State lead_car_state;  // = to the vehicle ahead...
                         //lead_car_state 변수는 앞차(선행 차량, Lead Car)의 상태 정보를 저장하는 변수
                         //자율주행 차량이 앞차의 속도, 위치 등을 참고하여 주행 전략을 조정하는 데 사용
                         //이 변수는 State 구조체 타입이며, 앞차의 위치, 속도, 방향 등의 정보를 담을 수 있음
                         //하지만 이 코드에서는 초기화만 되어 있고, 아직 값이 설정되지는 않음

  if(spirals.size() == 0){ //만약 생성된 경로(spirals)가 하나도 없다면, 오류 메시지를 출력하고 함수 종료
                           // spirals는 motion_planner.generate_spirals() 함수에서 생성된 여러 개의 주행 경로를 저장하는 벡터(vector).
                           //만약 spirals.size() == 0이면, 차량이 이동할 수 있는 주행 경로가 없다는 뜻이므로 더 이상 진행할 수 없음.
                           //즉, spirals가 비어 있으면 차량이 갈 길을 찾지 못했으므로, 오류 메시지를 출력하고 함수(path_planner())를 종료함.
  	cout << "Error: No spirals generated " << endl;
  	return;
  }



  //이 코드는 차량의 이동 가능한 여러 개의 경로(spirals)를 생성하고, 각각의 경로를 속도 프로파일과 함께 저장하는 역할수행
  //각 spiral(곡선 경로)에 대해 속도를 설정하고, 최종적으로 spirals_x, spirals_y, spirals_v에 저장함.
  for(int i = 0; i < spirals.size(); i++){ //spirals 벡터에 저장된 모든 경로를 반복
                                           //spirals는 motion_planner.generate_spirals()에서 생성된 여러 개의 주행 경로(곡선) 목록
                                           //spirals.size()만큼 반복하여 각 경로를 처리함

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory( 
      spirals[i], desired_speed, ego_state,lead_car_state, behavior); // 각 spiral(곡선 경로)에 대해 속도 프로파일을 적용하여 trajectory를 생성
                                                                      //generate_trajectory()는 motion_planner 내에서 호출되는 함수
                                                                      //spirals[i] → i번째 곡선 경로
                                                                      //desired_speed → 차량이 목표로 하는 속도
                                                                      //ego_state → 현재 차량의 상태 (위치, 속도 등)
                                                                      //lead_car_state → 앞차의 상태 (있을 경우)
                                                                      //behavior → 차량의 현재 행동 (정지, 주행, 감속 등)
                                                                      //결과적으로 trajectory에는 i번째 spiral을 따라가는 차량의 속도 및 위치 정보가 저장됨
//각 경로의 X, Y 좌표 및 속도를 저장할 벡터 선언
//여러 개의 주행 경로(spirals)를 처리하여 각 경로의 X, Y 좌표 및 속도 데이터를 저장하는 역할
//motion_planner._velocity_profile_generator.generate_trajectory()를 사용해 속도 프로파일을 적용.
// 각 경로의 좌표 및 속도를 벡터(spirals_x, spirals_y, spirals_v)에 저장하여 나중에 최적의 경로를 선택하는 데 활용.
    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;

    //각 trajectory(경로)에서 X, Y 좌표 및 속도를 추출하여 spiral_x, spiral_y, spiral_v에 저장
    //trajectory[j]는 j번째 점의 위치 및 속도 정보를 포함
    //즉, 차량이 spirals[i] 경로를 따라 이동할 때의 X, Y 좌표 및 속도를 저장하는 과정
    for(int j = 0; j < trajectory.size(); j++){
      double point_x = trajectory[j].path_point.x; //trajectory[j].path_point.x --> X좌표
      double point_y = trajectory[j].path_point.y; // trajectory[j].path_point.y --> y좌표
      double velocity = trajectory[j].v; //속도
                                       
      
      //각 spiral 경로의 X, Y, 속도 정보를 spirals_x, spirals_y, spirals_v에 저장
      //즉, 여러 개의 경로(spirals)에 대한 X, Y 좌표 및 속도 정보를 저장하여 나중에 최적의 경로를 선택하는 데 사용됨.
                                  //각각의 "개별 좌표 값" (한 점의 정보)
      spiral_x.push_back(point_x);  //point_x는 단일 좌표 값 (한 점의 X, Y 위치와 속도)
                                   // spiral_x는 한 개의 주행 경로(spiral)에 속한 모든 점들의 X, Y, 속도 리스트
      spiral_v.push_back(velocity); 
    }

                                  
                                  //여러 개의 주행 경로(spirals)"들의 리스트
                                  // 여러 개의 spirals를 저장하는 벡터에 현재 spiral 추가
                                  // spirals_x는 여러 개의 주행 경로(spirals)들을 저장하는 리스트임...s로 복수
    spirals_x.push_back(spiral_x); //spirals_x[i] → i번째 곡선 경로의 모든 X 좌표
    spirals_y.push_back(spiral_y); //spirals_y[i] → i번째 곡선 경로의 모든 Y 좌표
    spirals_v.push_back(spiral_v); //spirals_v[i] → i번째 곡선 경로에서 차량이 이동할 속도

/*
즉, point_x, point_y, velocity → 개별 점의 X, Y 좌표 및 속도 값
spiral_x, spiral_y, spiral_v → 한 개의 주행 경로(spiral)에 대한 X, Y, 속도 리스트
spirals_x, spirals_y, spirals_v → 여러 개의 주행 경로(spirals)를 저장하는 벡터 리스트
*/

  }


  //이 코드는 "최적의 주행 경로(spiral)를 선택하고, 그 경로를 따라 차량의 이동 지점을 설정하는 역할"
  // motion_planner.get_best_spiral_idx()를 사용해 여러 개의 주행 경로 중 최적의 경로를 선택
  //선택된 최적 경로를 x_points, y_points, v_points에 저장하여 차량이 따라갈 수 있도록 함


  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal); //get_best_spiral_idx()를 호출하여 최적의 주행 경로를 선택함
                                                                       // spirals(생성된 여러 개의 주행 경로) 중에서 장애물(obstacles)을 피하면서 목표 지점(goal)에 가장 적합한 경로를 선택
                                                                      // 선택된 최적의 경로의 인덱스를 best_spirals 벡터에 저장
  int best_spiral_idx = -1; //최적의 주행 경로가 없을 경우를 대비해 기본값 -1을 설정
                            // 만약 get_best_spiral_idx()가 유효한 주행 경로를 찾지 못하면, 기본적으로 -1이 유지됨.

  if(best_spirals.size() > 0) //best_spirals에 저장된 최적 경로가 하나 이상이면, 가장 마지막에 추가된 경로를 선택.
  	best_spiral_idx = best_spirals[best_spirals.size()-1]; //best_spirals.size() - 1는 가장 최신의 최적 경로를 의미
                                                          // 즉, 최적의 경로가 있으면 best_spiral_idx를 올바르게 설정하고, 없으면 그대로 -1 유지


  //최적 경로에서 사용할 데이터 개수를 설정                                                    
  int index = 0; // index = 0; → 최적 경로에서 처음부터 값을 가져오기 시작
  int max_points = 20; //최대 20개의 이동 포인트를 설정 (너무 긴 경로를 가져오지 않도록 제한)
  int add_points = spirals_x[best_spiral_idx].size(); //add_points = spirals_x[best_spiral_idx].size(); → 최적의 경로(best_spiral_idx)에서 사용할 총 포인트 개수
  while( x_points.size() < max_points && index < add_points ){ //최적의 경로를 따라 이동할 좌표(X, Y) 및 속도 데이터를 저장
                                                               //x_points.size() < max_points → 이동할 좌표 개수가 최대 개수(20개)를 넘지 않도록 제한
                                                               //index < add_points → 최적의 경로에 존재하는 포인트 개수를 초과하지 않도록 제한

   //최적의 경로에서 index번째 좌표(X, Y)와 속도를 가져옴
    double point_x = spirals_x[best_spiral_idx][index]; //선택된 최적 경로의 index번째 X 좌표
    double point_y = spirals_y[best_spiral_idx][index]; //선택된 최적 경로의 index번째 Y 좌표
    double velocity = spirals_v[best_spiral_idx][index]; //선택된 최적 경로의 index번째 속도
    index++;
    x_points.push_back(point_x); //최적 경로에서 가져온 좌표(X, Y) 및 속도를 x_points, y_points, v_points에 추가
    y_points.push_back(point_y); //이렇게 하면 차량이 따라가야 할 주행 경로가 최적 경로를 기반으로 업데이트됨.
    v_points.push_back(velocity);
    //최종적으로 x_points, y_points, v_points에는 최적의 주행 경로를 따라가도록 업데이트된 이동 좌표 및 속도 데이터가 저장
  } 


//여기서부터 내일(25년 2월 13일 시작예정)

}
/**
 * @brief 장애물 데이터 설정 함수 (스타터 코드 원본 유지)
 */
/*
이 함수 set_obst()는 장애물(Obstacle) 데이터를 설정하는 역할 수행
즉, x_points와 y_points로 주어진 장애물의 좌표들을 받아서, obstacles 벡터에 State 형식으로 저장하는 기능을 함.

이 함수는 main.cpp에서 장애물 데이터를 받아 obstacles 벡터에 저장하는 역할을 함다
이후 다른 함수들이 이 obstacles 벡터를 사용하여 장애물을 회피하거나 경로를 계획하는 데 활용할 수 있슴다
*/

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag){

	for( int i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i]; //x_points: 장애물의 x 좌표 리스트 ,x_points에 저장된 장애물의 좌표를 State 객체로 변환,obstacles 벡터에 추가하여 장애물 리스트를 업데이트
		obstacle.location.y = y_points[i]; //y_points: 장애물의 y 좌표 리스트
		obstacles.push_back(obstacle); //obstacles: 장애물의 상태(State)를 저장할 벡터 (참조로 전달)
	}
	obst_flag = true; //obst_flag: 장애물이 존재하는지 여부를 나타내는 플래그 (참조로 전달)
                    //장애물이 설정되었음을 나타내는 obst_flag를 true로 변경
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
  cout << "starting server" << endl; //"starting server"를 출력하여 프로그램이 시작됨을 알림
  uWS::Hub h;  // WebSocket 서버 객체를 생성하여 클라이언트와 통신을 담당함  → 자율주행 시뮬레이터와 실시간 데이터 송수신을 가능하게 함

  double new_delta_time;
  int i = 0;


   // PID 제어기 데이터 저장 파일 초기화 (steer_pid_data.txt, throttle_pid_data.txt 두 개의 파일을 초기화)
   //→ PID(비례-적분-미분) 제어기의 조향 및 가속 데이터 저장
  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc); //std::ofstream::trunc → 기존 파일 내용을 지우고 새로 생성
                                                                                   //즉, 자율주행이 시작될 때마다 기존 PID 로그 데이터를 초기화하는 과정
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();


  // 시간 측정을 위한 변수 설정
  time_t prev_timer; //prev_timer, timer → 실행 시간 측정을 위한 변수
  time_t timer;
  time(&prev_timer); //time(&prev_timer); → 프로그램 실행 시작 시간을 prev_timer에 저장
                     //→ 이후 new_delta_time을 계산하여 각 루프에서 경과 시간을 추적하는 데 사용됨


  // initialize pid steer
  /**
  * TODO (Step 1): create pid (pid_steer) for steer command and initialize values
  **/


  // initialize pid throttle
  /**
  * TODO (Step 1): create pid (pid_throttle) for throttle command and initialize values
  **/


 // TODO (Step 1): PID 제어기 생성 및 초기화
  PID pid_steer = PID(); //→ 조향(steer)용 PID 컨트롤러 생성
  pid_steer.Init(KP_STEER, KI_STEER, KD_STEER, MAX_STEER, MIN_STEER); //mithul12의 코드응용

  PID pid_throttle = PID(); // → 가속(throttle)용 PID 컨트롤러 생성
  pid_throttle.Init(KP_THROTTLE, KI_THROTTLE, KD_THROTTLE, MAX_THROTTLE, MIN_THROTTLE); //mithul12의 코드응용
//이제 TODO (Step 1)를 반영하여 PID 제어기가 생성되었으며, 이후 코드에서 이를 활용하여 조향과 가속을 조절할 수 있음.






/*
아래 코드는 WebSocket 이벤트 핸들러(h.onMessage()) 내부에서 실행되며, 차량의 주행 데이터를 받아 **경로를 계획(path_planner 호출)**하는 역할 수행
즉, 자율주행 차량과 서버 간의 실시간 데이터 송수신 및 처리를 담당
*/
  //path_planner() 함수는 WebSocket 이벤트 핸들러(h.onMessage())에서 호출
  //uWS::WebSocket<uWS::SERVER> ws → WebSocket 객체, 데이터를 송수신하는 역할
  //핵심: WebSocket을 통해 차량의 데이터를 수신할 때, 이 코드 블록이 실행됨.
  h.onMessage([&pid_steer, &pid_throttle, &new_delta_time, &timer, &prev_timer, //[&pid_steer, &pid_throttle, &new_delta_time, ...] → 람다 캡처 리스트 (외부 변수를 사용할 수 있도록 함)
    &i, &prev_timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)  //char *data, size_t length, uWS::OpCode opCode → 클라이언트에서 보낸 데이터
  {
        auto s = hasData(data);  // JSON 데이터 확인
                                 //hasData(data) → 데이터가 null이 아닌지 확인 (없으면 빈 문자열 반환)

        if (s != "") {

          auto data = json::parse(s); // JSON 데이터 파싱
                                      //json::parse(s) → 문자열을 JSON 형식으로 변환
                                      //핵심: 차량의 데이터를 JSON 형태로 변환하여 코드에서 사용할 수 있도록 함.


          // create file to save values
          // PID 제어 데이터를 저장할 파일 열기
          fstream file_steer; //fstream을 사용하여 파일을 OPEN
          file_steer.open("steer_pid_data.txt"); //steer_pid_data.txt → 조향(steering) PID 제어값 저장
          fstream file_throttle; //fstream을 사용하여 파일을 OPEN
          file_throttle.open("throttle_pid_data.txt"); //throttle_pid_data.txt → 가속(throttle) PID 제어값 저장

          
          // 차량의 주행 데이터 받아오기
          vector<double> x_points = data["traj_x"]; //x_points, y_points → 차량이 이동하는 경로 좌표 리스트
          vector<double> y_points = data["traj_y"];
          vector<double> v_points = data["traj_v"]; //v_points → 경로 상의 속도 값 리스트
          double yaw = data["yaw"]; //yaw → 차량의 현재 방향(각도)
          double velocity = data["velocity"]; //velocity → 차량의 현재 속도
          double sim_time = data["time"]; //sim_time → 현재 시뮬레이션 시간
          double waypoint_x = data["waypoint_x"]; //waypoint_x, waypoint_y, waypoint_t → 목표 지점의 위치(x, y)와 방향(yaw)
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"]; //is_junction → 현재 위치가 교차로인지 여부
          string tl_state = data["tl_state"]; //tl_state → 신호등 상태(예: "red", "green" 등)

          double x_position = data["location_x"]; //x_position, y_position, z_position → 차량의 현재 위치
          double y_position = data["location_y"];
          double z_position = data["location_z"];
          //여기까지 --- 핵심: 차량의 현재 상태와 목표 위치를 서버에서 받아옴.

          // 장애물 데이터 처리
          if(!have_obst){
          	vector<double> x_obst = data["obst_x"]; //data["obst_x"], data["obst_y"] → 장애물의 x, y 좌표 목록
          	vector<double> y_obst = data["obst_y"];
          	set_obst(x_obst, y_obst, obstacles, have_obst); //장애물 데이터를 obstacles 리스트에 저장
                                                          //have_obst = true; 로 설정하여 한 번만 실행되도록 함
          } //여기까지 --핵심: 장애물이 감지되었을 경우 한 번만 장애물 목록을 저장함.
         

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
          vector< vector<double> > spirals_x; //spirals_x, spirals_y, spirals_v → 생성된 후보 경로(spirals)의 x, y 좌표 및 속도 값
          vector< vector<double> > spirals_y;
          vector< vector<double> > spirals_v;
          vector<int> best_spirals; //best_spirals → 최적 경로의 인덱스

          //결과적으로 path_planner()를 통해 차량의 이동 경로를 생성하고 최적의 경로를 선택함
          path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);



          // Save time and compute delta time
            // 시간 갱신 및 델타 타임 계산
          time(&timer);   //time(&timer); → 현재 시간을 가져옴. 


          new_delta_time = difftime(timer, prev_timer); //difftime(timer, prev_timer); → 이전 타임스탬프와 비교하여 경과 시간(델타 타임)을 계산.
          prev_timer = timer; //prev_timer를 최신 시간으로 갱신

          /* 
          여기까지의 코드는 WebSocket을 통해 차량의 주행 데이터와 장애물 정보를 받아오고, 이를 기반으로 path_planner()를 호출하여 차량의 주행 경로를 생성하는 역할
          경로 생성 후, 시간을 갱신하여 PID 컨트롤러의 업데이트에 사용할 델타 타임을 계산
          */





          //이제 여기서부터의 코드는 차량의 조향(steering)을 PID 제어기를 통해 조정하는 역할
          //그런데, 스타터 코드에서는 조향 값을 계산하고 적용하는 부분이 주석 처리되어 있음.
          ////////////////////////////////////////
          // Steering control
          ////////////////////////////////////////

          /**
          * TODO (step 3): uncomment these lines
          **/
//           // Update the delta time with the previous command
           pid_steer.UpdateDeltaTime(new_delta_time);  //PID 제어기의 delta time(시간 간격)을 업데이트하는 부분.
                                                         //이 코드가 주석 처리된 이유는 아직 PID 제어를 완전히 활성화하지 않았기 때문.
                                                         //추후 구현할 때 주석을 해제해야 함
                                                         //464,467행에 pid제어를 적용했기에 주석해제


           // 🚗 TODO (Step 3): 조향(steer) 제어 적용
          // Compute steer error
        
        double steer_output = 0.0;
        //double steer_output; //mithul12의 코드 적용인줄 알았는데 원래 있던 스타터 코드, mithul12가 위치만 바꾼것, 아래의 중복코드는 주석처리
                            //steer_output : PID 제어기를 통해 계산된 조향 값
        
        //double error_steer; //스타터 코드 원본
        double error_steer = 0.0;  // 조향 오차 (나중에 계산할 것) ,error_steer : 조향 오차값을 저장하는 변수 
                                   // 0으로 초기화 이유 --> 실제 조향 오차 값을 계산하지 않고, 임시로 0으로 설정
                                   //오차 계산하는 코드가 아직 구현되지 않았기 때문.
                                   //아래의 3줄로 오차 계산하는 코드가 구현됨 그러므로 
 
        //아래 3줄의 코드 ,mithul12의 코드응용                             
        int closest_point_index = find_closest_point(x_position, y_position, x_points, y_points); //현재 차량 위치(x_position, y_position)와 가장 가까운 주행 경로의 포인트를 찾음.
                                                                                              //즉, 차량이 도로 상에서 가장 가까운 위치를 기준으로 방향을 조정할 수 있도록 함.
        double angle = angle_between_points(x_position, y_position, x_points[closest_point_index], y_points[closest_point_index]); 
                       //현재 차량 위치에서 가장 가까운 경로 포인트까지의 방향(각도)을 계산. 
                       //즉, 차량이 향해야 하는 목표 방향(waypoint 방향)을 구함.
        error_steer = normalize_angle(angle - yaw); //목표 방향(waypoint)과 현재 차량 방향(yaw) 간의 차이를 계산.
                                                    // 즉, 차량이 회전해야 할 정도를 나타내는 값이 error_steer.                       
                                   


        //double steer_output;  //steer_output : PID 제어기를 통해 계산된 조향 값
          //여기까지 TODO (Step 3)로 선언만 되어 있지만, 실제로는 오차 계산 후 pid_steer로 보낼 값을 저장해야 함.
          //여기까지 mithul12의 코드응용




          /**
          * TODO (step 3): compute the steer error (error_steer) from the position and the desired trajectory
           TODO (Step 3): 조향 오차(error_steer)를 계산하여 PID 입력으로 전달
          **/
//           error_steer = 0;  //주석 처리된 조향 오차(error_steer) 계산
                               //조향 오차는 보통 차량의 현재 진행 방향과 목표 방향(waypoint) 차이를 기반으로 계산함.
                              //현재는 오차 계산 코드가 없어서 error_steer = 0;로 남겨둠.

          /**
          * TODO (step 3): uncomment these lines
          **/
//           // Compute control to apply
           pid_steer.UpdateError(error_steer);   //PID 제어기 적용 코드,PID 제어기에 조향 오차를 입력하고, 조향 값을 계산하는 부분.
                                                  //UpdateError(error_steer) : PID 컨트롤러가 오차값을 기반으로 내부 상태 업데이트.
           steer_output = pid_steer.TotalError(); //TotalError() : 최종 조향 값 출력.
                                                    //주석처리 이유 : 조향 오차(error_steer) 계산 코드가 구현되지 않았기 때문에 아직 활성화할 수 없음
                                                    //error_steer = 0;으로만 설정한 상태라, PID 제어기를 실행해도 제대로 동작하지 않음,이제 mithul12의 코드를 602행에서 응용 , 그래서 주석해제함

//          
// 
           // Save data                                    //  PID 값 저장 코드,PID 조향 오차와 결과 값을 파일(steer_pid_data.txt)에 저장.
                                                                 //로그 데이터를 남기기 위한 코드지만, 조향 값이 실제로 계산되지 않으므로 주석 처리됨.
                                                                 //조향값이 steer control에서 구현되었기에 주석해제함
           file_steer.seekg(std::ios::beg);
           for(int j=0; j < i - 1; ++j) {
               file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
           }
           file_steer  << i ;
           file_steer  << " " << error_steer;
           file_steer  << " " << steer_output << endl;
           //이 부분은 pid_steer.UpdateError(error_steer);와 steer_output = pid_steer.TotalError();가 활성화되었기 때문에 주석해제 함

           

// 여기까지 : TODO (Step 3)에서 변수 선언 (error_steer, steer_output)
//조향 오차(error_steer) 계산이 구현되지 않아 PID 적용 코드가 주석 처리됨
//PID 제어기(pid_steer.UpdateError())를 호출하려면 먼저 오차 계산 로직을 추가해야 함
//현재는 단순히 변수만 정의하고, PID 제어기 로직을 나중에 추가할 예정
// 결론: 현재는 PID 적용 전 상태! 오차(error_steer) 계산이 구현되면 주석을 해제하고 활성화해야 함! 🚗

      



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
           pid_throttle.UpdateDeltaTime(new_delta_time); //PID 제어기의 내부 시간(delta time)을 업데이트하는 코드.
                                                           //하지만 현재 PID 제어기 코드가 완성되지 않았기 때문에 비활성화(주석 처리)됨.
                                                           //나중에 속도 오차 계산이 구현되면, 주석 해제해서 활성화해야 함.
                                                           //464행에 pid_steer.Init(KP_STEER, KI_STEER, KD_STEER, MAX_STEER, MIN_STEER);
                                                           //467행에 pid_throttle.Init(KP_THROTTLE, KI_THROTTLE, KD_THROTTLE, MAX_THROTTLE, MIN_THROTTLE); 
                                                           //위의 464,467행이 구현되었기에 주석해제
                                                          
           // 🚗 TODO (Step 2): 가속(throttle) 및 브레이크 제어 적용
          // Compute error of speed
          //double error_throttle; //스타터 코드 원본 
          double error_throttle = 0.0;  // 속도 오차 (나중에 계산할 것) ,error_throttle : 현재 속도와 목표 속도 간 오차 값을 저장하는 변수
                                      //현재 속도 오차를 계산하는 코드가 없어서 error_throttle = 0; 으로 설정
                                      //error_throttle = 0; → 🚨 임시 값 (추후 속도 오차 계산을 구현해야 함)
                                      //실제 속도 오차 계산 코드가 추가되면, 이 부분에서 오차를 정확하게 계산해야 함.
          /**
          * TODO (step 2): compute the throttle error (error_throttle) from the position and the desired speed
          **/
          // modify the following line for step 2
          // error_throttle = 0; //이거는 초기화인듯..일단 주석처리
          error_throttle = v_points[closest_point_index] - velocity; //mithul12의 코드응용 , 이게 속도 오차 계산하는 코드임

          //double throttle_output;
          double throttle_output = 0.0;
          double brake_output;

          //* TODO (Step 2): 속도 오차(error_throttle)를 계산하여 PID 입력으로 전달
          /**
          * TODO (step 2): uncomment these lines
          **/
//           // Compute control to apply
             pid_throttle.UpdateError(error_throttle); //PID 제어기에 속도 오차(error_throttle) 값을 전달해서 내부 상태 업데이트.
             double throttle = pid_throttle.TotalError(); //PID 계산을 통해 최종 throttle 값(가속 값) 반환.
                                                        //error_throttle이 아직 정확한 값을 계산하지 않기 때문에 주석처리됨
                                                        //속도 오차를 계산하는 코드가 추가되면, 이 부분의 주석을 해제해야 함.
                                                        //두 코드 모두 주석해제함 (위에 687행에서 속도오차 구현 즉,error_throttle = v_points[closest_point_index] - velocity; 이 코드 구현되었기 떄문에

//           
// 
//           //여기서부터 -- 브레이크 및 가속도 설정 코드 (주석 처리됨)
//           //throttle_output(가속도)과 brake_output(브레이크 출력)을 설정하는 코드.
//           // Adapt the negative throttle to break
            //error_throttle = v_points[closest_point_index] - velocity; 이 코드가 구현되었기에 주석해제

             if (throttle > 0.0) {                      //throttle(가속도) 값이 양수(>0) → throttle_output에 적용 (가속)
                                                        //throttle(가속도) 값이 음수(<0) → brake_output에 적용 (감속)
                                                        //현재 throttle 값을 계산하는 코드가 없음 → 제대로 작동하지 않음. 그래서 아직 주석처리
                                                        //속도 오차(error_throttle) 계산이 추가되면 주석을 해제해서 활성화해야 함. (위에 687행에서 속도오차 구현했기에)주석해제
              throttle_output = throttle;
             brake_output = 0;
             } else {
             throttle_output = 0;
             brake_output = -throttle;
           }  //여기까지 브레이크 및 가속도 설정 코드 (원래 스타터코드에서 주석 처리되었었는데 해제함)
             //여기서부터 PID 값 저장 코드,  error_throttle, brake_output, throttle_output 값을 throttle_pid_data.txt 파일에 저장하는 코드.
             //현재 제대로 된 throttle 값을 계산하지 않기 때문에 주석 처리됨.
             //PID 적용 코드가 완성되면, 주석을 해제하고 PID 데이터를 저장할 수 있음.



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
//error_throttle(속도 오차) 계산이 아직 구현되지 않음 → 임시로 0 설정 --> (위에 682행에서 속도오차 구현했기에)
//PID 제어기(pid_throttle.UpdateError()) 적용 부분이 주석 처리됨 → 속도 오차 계산 코드 추가 후 해제해야 함
//throttle_output과 brake_output 설정 코드도 PID 적용 후 활성화해야 함
//현재는 PID 적용 전 상태! 🚗 속도 오차 계산이 구현되면 주석을 해제해야 함





//여기서부터 마지막 부분: WebSocket 통신을 통한 제어 명령 전송 및 서버 관리
//이 부분은 자율주행 차량이 생성한 경로 및 제어 신호(steer, throttle, brake)를 WebSocket을 통해 송신하고, 서버와 클라이언트 간의 연결을 관리하는 역할
          // Send control
          // JSON 데이터 생성 및 전송
          //제어 신호(brake, throttle, steer)를 JSON 객체에 저장
          json msgJson;
          msgJson["brake"] = brake_output; //brake_output : 브레이크 값
          msgJson["throttle"] = throttle_output; //throttle_output : 가속도 값
          msgJson["steer"] = steer_output; //steer_output : 조향(steering) 값
          //차량이 WebSocket을 통해 서버에서 데이터를 받아 이 값들을 사용해 실제로 움직이게 됨

          

          // 유효성 검사 추가 (NaN체크 :steer_output이나 throttle_output 값이 계산 중에 NaN (Not a Number)이 될 가능성이 있음. 
          // 이는 나눗셈에서 0으로 나누거나 정의되지 않은 연산이 발생했을 때 발생할 수 있음 )
          //에러 대응: if (isnan(steer_output) || isnan(throttle_output))를 통해 NaN 여부를 확인하고, 감지되면 값을 0으로 초기화
          //name 'steer' is not defined 에러가 발생 --> 이는 steer_output 값이 정의되지 않거나 잘못된 데이터를 전달했을 가능성을 나타냄. 따라서 NaN 체크는 필요
          if (isnan(steer_output) || isnan(throttle_output)) {
            cout << "Error: NaN detected in steer or throttle output" << endl;
            steer_output = 0.0;
            throttle_output = 0.0;
          }

          // 디버깅용 출력
          //name 'steer' is not defined 에러는 WebSocket으로 전송되는 JSON 데이터에서 steer 필드가 빠졌거나 잘못 전달되었음을 의미. 
          // 디버깅 출력으로 steer_output 값이 제대로 전달되는지 확인 가능
          cout << "Steer Output: " << steer_output << ", Throttle Output: " << throttle_output << endl;




          //경로 데이터를 JSON 객체에 저장
          msgJson["trajectory_x"] = x_points; //trajectory_x, trajectory_y : 차량의 이동 경로(x, y 좌표)
          msgJson["trajectory_y"] = y_points;
          msgJson["trajectory_v"] = v_points; //trajectory_v : 각 지점에서의 속도 정보
          // 자율주행 차량이 이 데이터를 활용해 경로를 따라 주행함.


         //생성된 주행 경로(spiral paths)를 JSON 객체에 저장
          msgJson["spirals_x"] = spirals_x; //spirals_x, spirals_y : 생성된 경로(spiral path)들의 x, y 좌표
          msgJson["spirals_y"] = spirals_y;
          msgJson["spirals_v"] = spirals_v; //spirals_v : 각 경로에서의 속도 정보
          msgJson["spiral_idx"] = best_spirals; //spiral_idx : 선택된 최적의 경로 인덱스(best spiral)
          //여러 개의 후보 경로 중 최적의 경로를 선택하고, 해당 데이터를 송신함.

          //현재 차량이 수행 중인 행동 상태(maneuver)를 JSON 객체에 저장
          //예를 들어 STOPPED, FOLLOW_LANE, DECELERATE_TO_STOP 등의 상태가 있을 수 있음.
          // 이 정보는 차량이 현재 어떤 동작을 수행하는지를 클라이언트에서 모니터링하는 데 활용됨
          msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();




          //  min point threshold before doing the update
          // for high update rate use 19 for slow update rate use 4
             // 주기적인 업데이트
          msgJson["update_point_thresh"] = 16; //차량이 새로운 경로를 업데이트하기 전에 최소한 16개의 포인트를 지나야 함.
                                               //값이 클수록 업데이트 빈도가 낮아지고, 값이 작으면 더 자주 업데이트됨.
                                               //업데이트 빈도를 조정해 시스템 부하를 관리하는 역할.



          auto msg = msgJson.dump(); //JSON 데이터를 문자열로 변환하여 WebSocket을 통해 전송할 준비.
                                      // WebSocket에서 문자열로 변환된 JSON 데이터를 사용해 송신함.

          i = i + 1;
          file_steer.close(); //파일(steer_pid_data.txt, throttle_pid_data.txt)을 닫음.
          file_throttle.close();
          //파일이 계속 열려있는 것을 방지하고, 새로운 데이터를 저장할 수 있도록 준비.



      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT); //WebSocket을 통해 JSON 데이터를 클라이언트로 전송
                                                            //ws.send() 함수는 데이터를 msg.data()(문자열) 형식으로 전송.
                                                            // 결과적으로 차량의 상태(위치, 속도, 조향 등)를 실시간으로 클라이언트에 보냄.

    }

  });


  //여기서부터 클라이언트(WebSocket) 연결 및 해제 관리
  // 클라이언트 연결 이벤트
  //WebSocket 클라이언트가 연결되었을 때 실행되는 코드.
  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
      cout << "Connected!!!" << endl; //cout << "Connected!!!" << endl; → 연결 성공 메시지 출력.
    }); 



 // 클라이언트 연결 해제 이벤트
 //클라이언트가 연결을 끊었을 때 실행되는 코드.
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
      ws.close(); //ws.close(); → WebSocket 연결을 닫음.
      cout << "Disconnected" << endl; //cout << "Disconnected" << endl; → 연결 해제 메시지 출력.
    }); //클라이언트가 WebSocket 서버에서 연결을 종료하면 이를 감지함.





    // WebSocket 서버 실행
  int port = 4567; //WebSocket 서버를 실행하고 포트 4567에서 클라이언트 연결을 기다림
  if (h.listen("0.0.0.0", port)) //h.listen("0.0.0.0", port) : 모든 네트워크 인터페이스에서 4567번 포트를 통해 서버를 실행.
    {
      cout << "Listening to port " << port << endl;
      h.run(); //h.run(); : WebSocket 서버 실행.
    }
  else
    {
      cerr << "Failed to listen to port" << endl; //실행 실패 시(if 문 실패) → "Failed to listen to port" 출력 후 프로그램 종료.
      return -1;
    } //최종적으로 WebSocket 서버가 실행되고, 클라이언트와의 통신을 시작함.
//여기까지 차량의 제어 신호(brake, throttle, steer)와 경로 데이터를 JSON 형식으로 변환하여 WebSocket을 통해 전송.
//클라이언트가 연결되거나 해제될 때 이를 감지하고, WebSocket 통신을 관리.
// update_point_thresh를 통해 경로 업데이트 빈도를 조절.
//WebSocket 서버를 포트 4567번에서 실행하여 클라이언트의 요청을 처리.
}



/*
 1. pid_steer.UpdateDeltaTime(new_delta_time); (576행) 주석 해제 이유
🔹 추가된 코드:

pid_steer.Init(KP_STEER, KI_STEER, KD_STEER, MAX_STEER, MIN_STEER); (464행)
pid_throttle.Init(KP_THROTTLE, KI_THROTTLE, KD_THROTTLE, MAX_THROTTLE, MIN_THROTTLE); (467행)
🔹 설명:

Init() 함수를 통해 PID 컨트롤러가 올바르게 초기화되었기 때문에 주석 해제 가능해짐.
📌 2. pid_steer.UpdateError(error_steer); / steer_output = pid_steer.TotalError(); (623, 625행) 주석 해제 이유
🔹 추가된 코드:

error_steer = normalize_angle(angle - yaw); (602행)
int closest_point_index = find_closest_point(x_position, y_position, x_points, y_points); (600행)
double angle = angle_between_points(x_position, y_position, x_points[closest_point_index], y_points[closest_point_index]); (601행)
🔹 설명:

조향 오차(error_steer) 계산이 구현되었기 때문에 PID 컨트롤러가 정상적으로 동작할 수 있음.
find_closest_point()와 angle_between_points()를 통해 조향 오차가 계산되므로 주석 해제 가능.
📌 3. file_steer 데이터 저장 코드 (631~635행) 주석 해제 이유
🔹 추가된 코드:

pid_steer.UpdateError(error_steer); (623행)
steer_output = pid_steer.TotalError(); (625행)
🔹 설명:

조향 제어값이 계산되었으므로 steer_pid_data.txt에 저장하는 코드가 정상적으로 동작할 수 있음.
📌 4. pid_throttle.UpdateDeltaTime(new_delta_time); (668행) 주석 해제 이유
🔹 추가된 코드:

pid_throttle.Init(KP_THROTTLE, KI_THROTTLE, KD_THROTTLE, MAX_THROTTLE, MIN_THROTTLE); (467행)
🔹 설명:

PID 컨트롤러가 초기화되었기 때문에 delta time 업데이트가 가능해짐.
📌 5. pid_throttle.UpdateError(error_throttle); / double throttle = pid_throttle.TotalError(); (702, 703행) 주석 해제 이유
🔹 추가된 코드:

error_throttle = v_points[closest_point_index] - velocity; (687행)
🔹 설명:

속도 오차(error_throttle) 계산이 구현되었기 때문에 PID 컨트롤러가 정상 동작 가능
📌 6. throttle_output 및 brake_output 설정 코드 (718~722행) 주석 해제 이유
🔹 추가된 코드:

error_throttle = v_points[closest_point_index] - velocity; (687행)
double throttle = pid_throttle.TotalError(); (703행)
🔹 설명:

속도 오차 및 PID 계산 결과가 나오므로 가속/감속 조정이 가능해짐.
📌 7. file_throttle 데이터 저장 코드 (727~733행) 주석 해제 이유
🔹 추가된 코드:

pid_throttle.UpdateError(error_throttle); (702행)
double throttle = pid_throttle.TotalError(); (703행)
throttle_output 및 brake_output 설정 코드 (718~722행)
🔹 설명:

PID 가속/감속 값이 정상적으로 계산되었으므로 throttle_pid_data.txt 파일에 저장 가능.
🚀 최종 결론:
✅ PID 초기화(Init()), 조향/속도 오차 계산(error_steer, error_throttle)이 추가되었기 때문에, 관련 PID 제어 코드와 로그 저장 코드가 정상적으로 주석 해제될 수 있었음! 🎯





*/