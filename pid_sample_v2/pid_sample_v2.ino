#include <stdio.h>
#include <Arduino.h>



/*
This code is created by aratahorie


*/


void pinInterruptR1();   //エンコーダー割り込み関数定義
void pinInterruptF1();

const int PINNUM_A1 = 22;//エンコーダーピン番号
const int PINNUM_B1 = 2;

volatile long int now_count_1 = 0;  //エンコーダーカウント用グローバル変数

void setup(){
  Serial.begin(115200); //initialize Serial 

  attachInterrupt(digitalPinToInterrupt(PINNUM_B1),pinInterruptR1,RISING);  //割り込みセットアップ
  attachInterrupt(digitalPinToInterrupt(PINNUM_B1),pinInterruptF1,FALLING);

}


void loop(){

  float goal_pos = 180; // absolute degree
  float goal_vel = 0;

  float p_gain_pos = 0;
  float i_gain_pos = 0;
  float d_gain_pos = 0;
  
  float p_gain_vel = 0;
  float i_gain_vel = 0;
  float d_gain_vel = 0;

  while(1){

    run_pid_pos(goal_pos,p_gain_pos,i_gain_pos,d_gain_pos);

  //run_pid_vel(goal_vel,p_gain_vel,i_gain_vel,d_gain_vel);

  }
}

void pinInterruptR1(){
  if(digitalRead(PINNUM_A1)==0){
    now_count_1--;
  }
  else {
    now_count_1++;
  }
}

void pinInterruptF1(){
  if(digitalRead(PINNUM_A1)==1){
    now_count_1--;
  }
  else {
    now_count_1++;
  }
}

float calc_feedback_pos(float enc_res,float pulse_count){
  float now_pos = (pulse_count / enc_res) * 360.0;  // convert encoder count to degree position
  return now_pos;
}

float calc_feedback_vel(float enc_res,int pulse_count,int diff_time){

  static int past_count = 0;
  
  float diff_count_sec = ((pulse_count - past_count)/diff_time)*1000;
  float now_vel = (diff_count_sec/enc_res); 


  past_count = pulse_count;

}



int calc_pid_pos(float now_pos,float goal_pos,
                 int diff_time,
                 float p_gain,float i_gain,float d_gain){



  float now_pos_error = 0;
  float pid_p = 0;
  float pid_i = 0;
  float pid_d = 0;
  int pid = 0;

  static float integral_error = 0;  // Accumulated error value
  static float past_pos_error = 0;  // position error at past time
  const int limit_pwm = 255;


  float pid_time = diff_time * 0.001;

  now_pos_error = goal_pos - now_pos;    

  integral_error = (now_pos_error + past_pos_error) * (pid_time/2);  //calculate total error

  pid_p = p_gain * now_pos_error;
  pid_i = i_gain * integral_error;  
  pid_d = d_gain * (now_pos_error - past_pos_error) / pid_time;

  past_pos_error = now_pos_error;    //update position error

  pid = pid_p + pid_i + pid_d;  

  if(pid > limit_pwm){
    pid = limit_pwm;
  }

  return pid;
}

int calc_pid_vel(float now_vel,float goal_vel,
                  float diff_time, 
                  float p_gain,float i_gain,float d_gain){

  float dt = diff_time / 1000;

  float dt_pid = 0;

  float now_error = goal_vel - now_vel;

  const int limit_pwm = 255;

  static float past_pid = 0;
  static float past_error = 0;
  static float pid_i = 0;
  static float pwm_output = 0;

  pid_i += i_gain * ((now_error + past_error)*(dt/2));

  float pid_p = p_gain * now_error;
  
  dt_pid = ((pid_p + pid_i) - past_pid)/dt;

  past_pid = pid_p + pid_i;

  past_error = now_error;

  pwm_output += dt_pid;

  if(pwm_output > limit_pwm){
    pwm_output = limit_pwm;
  }

  return pwm_output;
  
}

void write_motor(int now_pid){

  int out_power = 0;
  int dir_out = 0;
  
  const int dir_pin = 28;  //direction pin number
  const int pwm_pin = 4; // pwm pin number

/*
pwm value range from 0 to 255.
decide the pwm value and the dir value.
*/

  if(now_pid>0){   
    dir_out = 1;
    out_power =  now_pid;
  }
  else if(now_pid<0){
    dir_out = 0;
    out_power = -now_pid;
  }

  digitalWrite(dir_pin,dir_out);
  analogWrite(pwm_pin,out_power);
}


//pidで角度を制御する
void run_pid_pos(float goal_pos,
                 float p_gain,float i_gain,float d_gain){

  const int pid_period = 30;
  const int enc_res = 512.00;

  static int last_time = 0;

  int now_pwm = 0; 
  int diff_time = 0;

  float now_pos = 0;

  int now_time = millis();

  if(diff_time > pid_period){
    now_pos = calc_feedback_pos(enc_res,now_count_1);
    now_pwm = calc_pid_pos(now_pos,goal_pos,diff_time,p_gain,i_gain,d_gain);

    write_motor(now_pwm);
    last_time = now_time;
  }
  diff_time = now_time - last_time;
}

//pidで速度を制御する
void run_pid_vel(float goal_vel,
                 float p_gain,float i_gain,float d_gain){


  const int pid_period = 30;
  const int enc_res = 512.00;

  static int last_time = 0;

  int now_pwm = 0; 
  int now_vel = 0;

  int diff_time = 0;

  int now_time = millis();

  if(diff_time > pid_period){
    now_vel = calc_feedback_vel(enc_res,now_count_1,diff_time);
    now_pwm = calc_pid_vel(now_vel,goal_vel,diff_time,p_gain,i_gain,d_gain);
    write_motor(now_pwm);
    last_time = now_time;
  }

  diff_time = now_time - last_time;
  
}


