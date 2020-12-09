#include <Romi32U4.h>
#include "Behaviors.h"

//sensors
#include "Speed_controller.h"
#include "Median_filter.h"
#include "IMU.h"

//behaviors
#include "Wall_following.h"
#include "Position_estimation.h"

//sensors
Romi32U4ButtonA buttonA;

//motor-speed controller
SpeedController robot;
WallFollowingController wallFollow;
Position position;

//sensors
IMU_sensor LSM6;
Romi32U4ButtonA buttonA;

//median filter
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;

void Behaviors::Init(void)
{
    LSM6.Init();
    med_x.Init();
    med_y.Init();
    med_z.Init();
    position.Init();
    robot.Init();
}

void Behaviors::Stop(void)
{
    robot.Stop();
}

boolean Behaviors::DetectCollision(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    unsigned long now = millis();
    Serial.print(now);
    Serial.print('\t');
    Serial.print(data[0]);
    Serial.print('\t');
    Serial.print(data[1]);
    Serial.print('\t');
    Serial.print(data[2]);
    Serial.println();
    if((abs(data[0]) > threshold) || (abs(data[1]) > threshold)) return 1;
    else return 0;
}

void Behaviors::Run(void)
{
    switch (robot_state)
    {
    case IDLE:
        if(buttonA.getSingleDebouncedRelease()){ 
            robot_state = DRIVE; 
            robot.Stop();
            delay(1000); //delay 1 s          
        } 
        else { 
            robot_state = IDLE;
            robot.Stop(); 
        }   
        break;
    
    case DRIVE:
        if(DetectCollision){
            robot.Straight(-25,2); //reverse when collided
            robot_state = COLLISION;
            robot.Stop();
        }
        else {
            robot_state = DRIVE;
            robot.Run(50, 50);
        }
        break;

    case COLLISION:
        if(buttonA.getSingleDebouncedRelease()){
            delay(1000); //delay 1s after pressing button
            robot_state = TURN;
            robot.Stop();
        }
        else{
            robot_state = COLLISION;
            robot.Stop();
        }
        break;
    
    case TURN:
        if(robot.Turn(90, 0)){
            robot_state = WALLFOLLOW;
            robot.Stop();
            position.Stop(); //reset position to 0,0,0
        }
        break;
    
    case WALLFOLLOW:
        if(position.ReadY() < -0.92){ 
            robot_state = IDLE;
            robot.Stop(); //stop when hit the end of 10 cm
        }
        else{
            robot_state = WALLFOLLOW;
            int speed = wallFollow.Process(28); //maintain 28cm from wall
            robot.Run(50 + speed, 50 - speed);
            position.UpdatePose(50 + speed, 50 - speed);
        }
        break;
    };
}