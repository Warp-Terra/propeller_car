#include "propeller.h"
#include "simpletools.h"
#include <unistd.h> 


#define LFSensor_0 0
#define LFSensor_1 1
#define LFSensor_2 2
#define LFSensor_3 3
#define LFSensor_4 4
#define Trig_PIN_left 15
#define Echo_PIN_left 14
#define Trig_PIN_front 12
#define Echo_PIN_front 13


int watch_left();
int watch_front();
int custom_round(double number);


int RightMotorDirPin1 = 10;
int RightMotorDirPin2 = 9;
int LeftMotorDirPin1  = 6;
int LeftMotorDirPin2  = 7;
int speedPinL = 5;
int speedPinR = 8;

void pwm_speed(int pin, int speed);
void stop_Stop();
void forward(float t_ms);
void cw_rotate(float t_ms);
void ccw_rotate(float t_ms);
unsigned int sensor[5];
volatile unsigned int sensor_read = 0;

void go_advance(int t);
void go_left(int t);
void go_right(int t);

int read_sensor_values();
void auto_tracking();
void auto_tracking_S();
void obstacle_in_S();

void light_LED26(int t);
void light_LED27(int t);
void off_light_LED26();
void off_light_LED27();
void light_LED27_count(int t);
void light_LED26_park();

void go_advance_S(int t);

unsigned int intersection_number = 0;//record number of intersection
unsigned int intersection_number_A = 0;//record number of intersection in line A
unsigned int intersection_number_B = 0;//record number of intersection in line A
unsigned int intersection_distance = 0;//record number of intersection when pick up point detected
unsigned int pick_up = 0;//indicate we have pick up obstacle
unsigned int finish_A = 0;// record we finish in line A
unsigned int finish_B = 0;//record we finish in line B
int double_check = 0;




int main() {
  
    DIRA &= ~((1 << LFSensor_0) | (1 << LFSensor_1) | (1 << LFSensor_2) | (1 << LFSensor_3) | (1 << LFSensor_4)); // input pin for ultrasonic sensor 
    DIRA |= 1 << Trig_PIN_left; // inputTrig_PIN
    DIRA &= ~(1 << Echo_PIN_left); // output Echo_PIN
    DIRA |= 1 << Trig_PIN_front; // output Trig_PIN
    DIRA &= ~(1 << Echo_PIN_front); // output Echo_PIN
    
    int check_distance = watch_front();
    while(1){
      if(intersection_number >= 5){
        //go_advance(7);
        auto_tracking();
        int check_front = watch_front();          
        if(check_front < 30){
          pause(2000);//to indicate we find pick up point
          pick_up = 1;
          light_LED26(1);
          //LED
          go_advance(13);
          //pause(500);
          go_right(33);
          //intersection distance
        }else{
          //pause(500);
          go_advance(13);
          //pause(500);
          go_right(33);
        }
        while(1){
          intersection_number_B = 0;
          auto_tracking_S();
          //pause(500);
          intersection_number_A += 1;
          int check_left = watch_left();
          if(intersection_number_A < 4){
            if(check_left < 30){
              pause(2000);//to indicate we find pick up point
              pick_up = 1;
              light_LED26(1);
              //LED
              go_advance(13);
            }else{
              go_advance(13);
              if(pick_up == 1){
                intersection_distance += 1;
              }                
            } 
          }else{
            if(check_left < 30){
              pause(2000);
              pick_up = 1;
              light_LED26(1);
              //LED
              go_advance(13);
              //pause(500);
              go_right(33);
              finish_A = 1;
            }else{
              go_advance(13);
              //pause(500);
              go_right(33);
              finish_A = 1;
              if(pick_up == 1){
                intersection_distance += 1;
              }                
            }
          if(finish_A == 1){
            auto_tracking();
            //pause(500);
            if(pick_up == 1){
              intersection_distance += 1;
            }              
            go_advance(13);
            //pause(500);
            auto_tracking();
            if(pick_up == 1){
              intersection_distance += 1;
            }
            check_front = watch_front();
            if(check_front < 30){
              pause(2000);
              light_LED26(1);
              light_LED27(intersection_distance);
              exit(0);
              //LED
              //go_advance(13);
              //pause(500);
              //go_right(33);
              //pause(500);
            }else{
              //pause(500);
              go_advance(13);
              //pause(500);
              go_right(33);
              //pause(500);
            }
            while(1){
              intersection_number_A = 0;
              auto_tracking_S();
              //pause(500);
              if(pick_up == 1){
                intersection_distance += 1;
              }                
              intersection_number_B += 1;
              int check_left = watch_left();
              if(intersection_number_B < 4){
                if(check_left < 30){
                  pause(2000);
                  light_LED26_park();
                  light_LED27_count(intersection_distance);
                  //LED
                  print("find drop off place, job over!");
                  exit(0);
                }else{
                  go_advance(13);
                }
              }else{
                if(check_left < 30){
                  pause(2000);
                  light_LED26_park();
                  light_LED27_count(intersection_distance);
                  //LED
                  print("find drop off place, job over!");
                  exit(0);
                }else{
                  go_advance(13);
                  pause(500);
                  go_right(33);
                  pause(500);
                  auto_tracking();
                  pause(500);
                  go_advance(13);
                  auto_tracking();
                  finish_B = 1;
                }                                        
              }
              if(finish_B == 1){
                pick_up = 0;
                intersection_distance = 0;
                break;
              }                                                                                        
            }                                                              
          }                                                                   
        }                          
      }          
      //print("job is over");
      //exit(0);
      }else{  
        if (double_check == 0){    
          auto_tracking();
          //pause(500);
          intersection_number += 1;
          print("intersection number is %d \n",intersection_number);
          if(intersection_number > 4){
            go_advance(13);
            //pause(500);
            go_right(33);
          }
          if(intersection_number <= 4){
            check_distance = watch_front();
            if(check_distance > 30){
              print("no obstacle, move on!");
              go_advance(13);
              //pause(500);
              auto_tracking();
              //pause(500);
              intersection_number += 1;
              print("intersection number is %d \n",intersection_number);
              if(intersection_number >= 5){
                go_advance(13);
                go_right(33);
              }                
            }
          }             
        }
        check_distance = watch_front(); 
        if(check_distance < 30){
          print("have obstacle, turn left!");
          //double_check = 0;
          if(double_check == 0){
          go_advance(13);          
          //pause(500);
          go_left(33);
          }    
          if(double_check == 1){
            go_left(37);
          }                  
          //pause(500);
          auto_tracking();
          //pause(500);
          go_advance(13);
          //pause(500);
          go_right(33);
          //pause(500);
          auto_tracking_S();
          //pause(500);
          go_advance(13);
          //pause(500);
          go_right(33);
          //pause(500);
          auto_tracking();
          //pause(500);
          double_check = 0;
          intersection_number += 1;
          if(intersection_number <= 4){
            go_advance(13);
            //pause(500);
            go_left(33);
            //pause(500);
            check_distance = watch_front();
            if(check_distance < 30){
              double_check = 1;
              
            }
          } 
        }                     
      }              
    }      
     
    return 0;
}





void go_advance_S(int t){
  for(int i = 0; i < t; i++){
    forward(25);
    print("go advance \n");
  }
}  

void go_advance(int t){
  for(int i = 0; i < t; i++){
    forward(15);
    print("go advance \n");
  }
}

void go_right(int t){
  for(int i = 0; i < t; i++){
    cw_rotate(8);//10
    print("go right \n");
  }
}

void go_left(int t){
  for(int i = 0; i < t; i++){
    ccw_rotate(8);//10
    print("go left \n");
  }
}             
      
void forward(float t_ms) {

    high(RightMotorDirPin1);
    low(RightMotorDirPin2);
    
    high(LeftMotorDirPin1);
    low(LeftMotorDirPin2);
        
    high(speedPinL);
    high(speedPinR);
    
    pause(10);
    low(speedPinL); 
    low(speedPinR);   
    pause(t_ms);
}

void ccw_rotate(float t_ms){
    high(RightMotorDirPin1);
    low(RightMotorDirPin2);
    
    high(LeftMotorDirPin2);
    low(LeftMotorDirPin1);
        
    high(speedPinL);
    high(speedPinR);
    
    pause(10);
    low(speedPinL); 
    low(speedPinR);   
    pause(t_ms);

}

void cw_rotate(float t_ms){
    high(RightMotorDirPin2);
    low(RightMotorDirPin1);
    
    high(LeftMotorDirPin1);
    low(LeftMotorDirPin2);
        
    high(speedPinL);
    high(speedPinR);
    
    pause(10);
    low(speedPinL); 
    low(speedPinR);   
    pause(t_ms);
}

int read_sensor_values() {
    sensor[0] = (INA >> LFSensor_0) & 1;
    sensor[1] = (INA >> LFSensor_1) & 1;
    sensor[2] = (INA >> LFSensor_2) & 1;
    sensor[3] = (INA >> LFSensor_3) & 1;
    sensor[4] = (INA >> LFSensor_4) & 1;
    
    int val = 0;
    
    for (int i = 0; i < 5; i++){
      if (sensor[i] == 1){
          int temp = pow(2,i);
          val += temp;      
        } 
    }
   return val;
}

void auto_tracking() {
    int sensorval;
    
    while (1) {
        sensorval = read_sensor_values();
        print("auto_tracking \n");
        if (sensorval == 0b11011 || sensorval == 0b11011 || sensorval == 0b11001 || sensorval == 0b10011) {
            go_advance(1);
            print("forward \n");
        } else if (sensorval == 0b11101 || sensorval == 0b11110 || sensorval == 0b11100 || sensorval == 0b11000) {
            go_left(2);
            print("anti-clockwise \n");
        } else if (sensorval == 0b10111 || sensorval == 0b01111 || sensorval == 0b00111 || sensorval == 0b00011) {
            go_right(2);
            print("clockwise \n");
        } else if (sensorval == 0b00000 || sensorval == 0b00001 || sensorval == 0b10000) {
          go_advance(1);
            //print("stop \n");
            light_LED27(1);
            off_light_LED27();
            break;
        }
        
    }
}

void auto_tracking_S() {
    int sensorval;
    int check_distance = watch_front();
    while (1) {
      int check_distance = watch_front();
      if(check_distance > 30){
        sensorval = read_sensor_values();
        print("auto_tracking \n");
        if (sensorval == 0b11011 || sensorval == 0b11011 || sensorval == 0b11001 || sensorval == 0b10011) {
            go_advance_S(1);
            print("forward \n");
        } else if (sensorval == 0b11101 || sensorval == 0b11110 || sensorval == 0b11100 || sensorval == 0b11000) {
            go_left(2);
            print("anti-clockwise \n");
        } else if (sensorval == 0b10111 || sensorval == 0b01111 || sensorval == 0b00111 || sensorval == 0b00011) {
            go_right(2);
            print("clockwise \n");
        } else if (sensorval == 0b00000 || sensorval == 0b00001 || sensorval == 0b10000) {
            print("stop \n");
            light_LED27(1);
            off_light_LED27();
            go_advance_S(1);
            break;
        }
      }else{
        light_LED26(1);
        off_light_LED26();
      }                
    }
}


int watch_left() {
    long echo_distance;

    OUTA &= ~(1 << Trig_PIN_left); 
    usleep(5);               

    OUTA |= 1 << Trig_PIN_left; 
    usleep(15);            

    OUTA &= ~(1 << Trig_PIN_left);

    //wait PWM start
    while (((INA >> Echo_PIN_left) & 1) == 0) {
    }

    uint32_t start_time = CNT;

    // wait PWM stop
    while (((INA >> Echo_PIN_left) & 1) == 1) {
    }

    uint32_t end_time = CNT;
    echo_distance = (end_time - start_time) / (CLKFREQ / 1000000); 
    echo_distance = echo_distance * 0.01657; 

    return custom_round(echo_distance);
}

int watch_front() {
    long echo_distance;

    OUTA &= ~(1 << Trig_PIN_front); 
    usleep(5);               

    OUTA |= 1 << Trig_PIN_front; 
    usleep(15);            

    OUTA &= ~(1 << Trig_PIN_front); 

    
    while (((INA >> Echo_PIN_front) & 1) == 0) {
    }

    uint32_t start_time = CNT;

    
    while (((INA >> Echo_PIN_front) & 1) == 1) {
    }

    uint32_t end_time = CNT;
    echo_distance = (end_time - start_time) / (CLKFREQ / 1000000); 
    echo_distance = echo_distance * 0.01657; 

    return custom_round(echo_distance);
}

int custom_round(double number) {
    if (number >= 0) {
        return (int)(number + 0.5);
    } else {
        return (int)(number - 0.5);
    }
}

void light_LED26(int t){
  for(int i = 0; i < t; i++){
    high(26);
    pause(500);
    low(26);
    pause(500);
  }    
}

void off_light_LED26(){
  OUTA &= ~(1 << 26);
}  

void off_light_LED27(){
  OUTA &= ~(1 << 27);
}  

void light_LED27(int t){
  for(int i = 0; i < t; i++){
    high(27);
    pause(50);
    low(27);
    pause(50);
    OUTA &= ~(1 << 27);
  }    
}

void light_LED27_count(int t){
  for(int i = 0; i < t; i++){
    high(27);
    pause(500);
    low(27);
    pause(500);
    OUTA &= ~(1 << 27);
  }  
}        

void light_LED26_park(){
  high(26);
}  