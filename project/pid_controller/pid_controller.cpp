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

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

/*
* PID 클래스 생성자
* - 초기화 작업 없음 (Init 함수에서 설정)

* - 객체가 생성될 때 자동으로 실행되는 함수
* - 별도로 초기화할 값이 없으므로 빈 상태로 둠
*/
PID::PID() {}

/*
* PID 클래스 소멸자

* - 객체가 메모리에서 삭제될 때 자동으로 실행됨
* - 별도의 메모리 해제가 필요하지 않으므로 빈 상태로 둠
*/
PID::~PID() {}


/*
* PID 컨트롤러 초기화
* - PID 계수(Kp, Ki, Kd) 및 출력 제한값을 설정, 즉 * - 제어 계수(Kp, Ki, Kd)와 출력 제한값을 설정
 

*/

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   // this->Kp_는 이 클래스의 멤버 변수 Kp_를 의미
    // 외부에서 전달된 Kpi 값을 Kp_에 저장

   this->Kp_ = Kpi;  // 비례 계수 설정
   this->Ki_ = Kii;  // 적분 계수 설정
   this->Kd_ = Kdi;  // 미분 계수 설정
   this->output_lim_max_ = output_lim_maxi;  // 최대 출력 제한
   this->output_lim_min_ = output_lim_mini;  // 최소 출력 제한

   // 오차 값 초기화
   this->prev_cte_ = 0.0; // 이전 CTE 값 (초기에는 0)
   this->diff_cte_ = 0.0;  // CTE 변화량 (초기에는 0)
   this->int_cte_ = 0.0;  // 누적 CTE (초기에는 0)
   this->dt_ = 1.0;  // 기본값 (필요시 UpdateDeltaTime에서 갱신)
                     // 기본적인 시간 간격 값 (필요시 UpdateDeltaTime에서 변경됨)

}

/*
* PID 오차 업데이트
* - 주어진 CTE(Cross Track Error)를 기반으로 오차 값 업데이트
* - 현재 주어진 CTE(Cross Track Error)를 이용해 P, I, D 값 업데이트
*/
void PID::UpdateError(double cte) {
   // dt_가 0이면 0으로 나누는 오류가 발생하므로 방어 코드 추가
   /**
   * TODO: Update PID errors based on cte.
   **/
   //this->diff_cte_ = (cte - this->prev_cte_) / this->dt_; // 미분 오차 계산

   if (dt_ > 0) { 
      this->diff_cte_ = (cte - this->prev_cte_) / this->dt_; // 미분 항 (현재 CTE와 이전 CTE 차이를 시간 간격으로 나눔)
  } else {
      this->diff_cte_ = 0.0;  // 0으로 나누는 오류 방지
                              // dt_가 0일 경우, 미분 계산을 하지 않고 0으로 설정
  }
   this->prev_cte_ = cte; //  현재 CTE를 이전 CTE 값으로 저장 (다음 계산을 위해 필요)
   this->int_cte_ += cte * this->dt_; // 적분 오차 누적 , 즉,// 적분 항 (누적 CTE에 현재 CTE 값을 더함)
}

/*
* PID 총 오차 계산
* - PID 공식에 따라 최종 제어 출력을 계산하여 반환

* - P, I, D 항을 기반으로 최종적인 제어 출력값을 반환
* - 출력값을 설정된 최소/최대 범위로 제한
*/
double PID::TotalError() {
    // PID 제어 공식 적용
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double control = this->Kp_ * this->prev_cte_  // 비례 항, 즉, P (비례) 항: 현재 CTE에 비례 계수를 곱한 값
                  + this->Kd_ * this->diff_cte_  // 미분 항, 즉, D (미분) 항: CTE 변화량에 미분 계수를 곱한 값
                  + this->Ki_ * this->int_cte_; // 적분 항, 즉,  I (적분) 항: 누적 CTE에 적분 계수를 곱한 값

   // 출력 제한 적용
   //return max(this->output_lim_min_, min(control, this->output_lim_max_)); -// 이 코드로 if else를 사용하지 않고 아래 코드 대체가능, 다만, 초보자는 max()와 min()의 동작을 한번 더 생각해봐야 함
   if (control < this->output_lim_min_) {
      control = this->output_lim_min_;
  } else if (control > this->output_lim_max_) {
      control = this->output_lim_max_;
  }
    return control;
}

/*
* 델타 타임 업데이트
* - PID 컨트롤러가 시간 간격을 조정할 수 있도록 업데이트
* - 시간 간격을 조정하여 PID 제어 계산에 반영
*/


double PID::UpdateDeltaTime(double new_delta_time) {
   //"main.cpp에서 주기적으로 호출되며, 새로운 시간 간격을 설정"
   
   /**
   * TODO: Update the delta time with new value
   */
   // this->dt_ = new_delta_time; // 기존 코드
   this->dt_ = (new_delta_time > 0) ? new_delta_time : 1.0;  // dt_가 0이 되지 않도록 보호
   return this->dt_;
}