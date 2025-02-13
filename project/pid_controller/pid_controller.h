/*
pid_controller.cpp 내부의 UpdateError(), TotalError(), UpdateDeltaTime() 함수들은 pid_controller.h에 정의된 함수를 구현하는 것.
main.cpp는 pid_controller.h를 include하여 PID 객체를 생성하고, 그 객체를 통해 pid_controller.cpp 내부의 함수들을 호출함.
결과적으로 pid_controller.h가 다른 파일(main.cpp)과 pid_controller.cpp를 연결하는 역할을 함.

*/


/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

   /**
   * TODO: Create the PID class
   **/

    /*
       Errors (PID 오차 값)
       * - main.cpp에서 PID 객체를 통해 UpdateError()를 호출할 때 갱신됨
      * - pid_controller.cpp 내부에서 사용되어 제어 입력을 계산하는 데 활용됨
     
       */

    double prev_cte_;   // 이전 CTE (Cross Track Error, 차량이 목표 경로에서 벗어난 정도) - pid_controller.cpp에서 사용됨
    double diff_cte_;   // CTE의 변화율 (미분 항) - pid_controller.cpp에서 사용됨
    double int_cte_;     // 누적 CTE (적분 항) - pid_controller.cpp에서 사용됨
  

    /*
    
    Coefficients (PID 계수)
    * - main.cpp에서 Init() 함수를 통해 전달됨
    * - pid_controller.cpp에서 TotalError() 계산에 사용됨
    */
    double Kp_; // 비례 계수 (Proportional gain) - main.cpp에서 전달됨
    double Ki_; // 적분 계수 (Integral gain) - main.cpp에서 전달됨
    double Kd_; // 미분 계수 (Derivative gain) - main.cpp에서 전달됨

    /*
    
     Output limits (출력 제한 값)
     * - main.cpp에서 Init() 함수를 통해 설정됨
    * - pid_controller.cpp에서 TotalError() 호출 시 적용됨
    */
    double output_lim_min_; // 최소 출력 값 - main.cpp에서 전달됨
    double output_lim_max_; // 최대 출력 값 - main.cpp에서 전달됨
  
    /*
    
    Delta time (델타 타임 - 시간 간격)
    * - main.cpp에서 UpdateDeltaTime() 함수를 통해 주기적으로 업데이트됨
    * - pid_controller.cpp에서 미분 항 (D-term) 계산에 사용됨
    */
    double dt_; // 시간 변화량 (PID 업데이트를 위한 간격) - main.cpp에서 UpdateDeltaTime()을 통해 변경됨

    /*
    * Constructor (생성자)
     * - main.cpp에서 PID 객체를 생성할 때 자동 호출됨
    */
    PID();

    /*
    Destructor (소멸자)
     * - 객체가 소멸될 때 호출되지만 별도 메모리 해제 작업은 필요 없음
    */
    virtual ~PID();

    /*
   
      Initialize PID (PID 초기화)
    * - PID 계수(Kp, Ki, Kd) 및 출력 제한값을 설정
     * - main.cpp에서 호출되며, PID 계수(Kp, Ki, Kd) 및 출력 제한값을 설정

    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    Update the PID error variables given cross track error (PID 오차 업데이트)
    * - 주어진 CTE를 기반으로 P, I, D 요소를 갱신
    * - main.cpp에서 주기적으로 호출됨
    * - 전달된 CTE 값을 기반으로 P, I, D 값을 업데이트함
      
    */
    void UpdateError(double cte);

    /*   
      Calculate the total PID error (PID 총 오차 계산)
    * - P, I, D 요소를 이용하여 최종적인 제어 출력을 반환
     * - pid_controller.cpp에서 TotalError()가 호출되며, 제어 신호를 반환함
    * - main.cpp에서 이 값을 사용하여 차량의 조향/가속도를 조정함
 
    
    */
  
  
    double TotalError();
  
    /*
    
     * Update the delta time (델타 타임 업데이트)
    * - 새로운 시간 간격을 설정하여 PID 업데이트에 반영
     * - main.cpp에서 주기적으로 호출되며, 새로운 시간 간격을 설정하여 PID 업데이트에 반영
        "main.cpp에서 이 값을 사용하여 차량의 조향/가속도를 조정함"

    */
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H


