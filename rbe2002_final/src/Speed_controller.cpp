#include <Romi32U4.h>
#include "Encoders.h"
#include  "Speed_controller.h"
#include "Position_estimation.h"

Romi32U4Motors motors;
Encoder MagneticEncoder; 
Position odometry;

float time_track = 0;

void SpeedController::Init(void)
{
    MagneticEncoder.Init();
    odometry.Init();
}

void SpeedController::Run(float target_velocity_left, float target_velocity_right)
{
    if(MagneticEncoder.UpdateEncoderCounts()){
        time_track = time_track + 50/1000.0;
        float e_left = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
        float e_right = target_velocity_right - MagneticEncoder.ReadVelocityRight();
        
        E_left += e_left;
        E_right += e_right;

        float u_left = Kp*e_left + Ki*E_left;
        float u_right = Kp*e_right + Ki*E_right;

        motors.setEfforts(u_left,u_right);
        //motors.setEfforts(30,30);
        //odometry.UpdatePose(target_velocity_left,target_velocity_right);
    }
}

boolean SpeedController::MoveToPosition(float target_x, float target_y)
{
    do
    {    
        //assignment
        float current_x = odometry.ReadX();
        float current_y = odometry.ReadY();
        float current_theta = odometry.ReadTheta();
        error_distance = sqrt(pow(target_x - current_x, 2)+pow(target_y - current_y, 2));
        float error_theta = atan2(target_y - current_y, target_x - current_x) - current_theta;
        if(error_theta > (PI/180)*185) error_theta -= 2*PI;
        else if(error_theta < -(PI/180)*185) error_theta += 2*PI;

        E_distance += error_distance;
        E_theta += error_theta;
        
        float constrained_velocity_left = constrain((Kp2*error_distance - Kp2*error_theta), -50, 50);
        float constrained_velocity_right = constrain((Kp2*error_distance + Kp2*error_theta), -50, 50);

        
        Run(constrained_velocity_left,constrained_velocity_right);
        Serial.print(target_x);
        Serial.print('\t');
        Serial.print(odometry.ReadX());
        Serial.print('\t');
        Serial.print(target_y);
        Serial.print('\t');
        Serial.print(odometry.ReadY());
        Serial.print('\t');
        Serial.print(MagneticEncoder.ReadVelocityLeft());
        Serial.print('\t');
        Serial.print(MagneticEncoder.ReadVelocityRight());
        Serial.print('\t');
        Serial.print(error_distance);
        Serial.print('\t');
        Serial.print(error_theta);
        Serial.print('\t');
        Serial.print(constrained_velocity_left);
        Serial.print('\t');
        Serial.print(constrained_velocity_right);
        Serial.println("");

    } while (error_distance >= 0.03); //define a distance criteria that lets the robot know that it reached the waypoint.
    return 1;
}

boolean SpeedController::Turn(int degree, int direction)
{
    motors.setEfforts(0, 0);
    float turns = counts*degree;
    int count_turn = MagneticEncoder.ReadEncoderCountLeft();

    while(abs(abs(count_turn) - abs(MagneticEncoder.ReadEncoderCountLeft())) <= turns)
    {
        if(!direction) Run(50,-50);
        else Run(-50,50);
    }
    motors.setEfforts(0, 0);
    return 1;
}

boolean SpeedController::Straight(int target_velocity, int time)
{
    motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        float velocity_left = MagneticEncoder.ReadVelocityLeft();
        float velocity_right = MagneticEncoder.ReadVelocityRight();
        //float output_velocity_left = constrain(target_velocity, velocity_left - 87, velocity_left + 87);
        //float output_velocity_right = constrain(target_velocity, velocity_right - 87, velocity_right + 87);
        //Run(output_velocity_left,output_velocity_right);
        Run(target_velocity,target_velocity);
        /*Serial.print(millis()-now);
        Serial.print('\t');
        Serial.print(velocity_left);
        Serial.print('\t');
        Serial.print(velocity_right);
        Serial.println("");*/
    }
    motors.setEfforts(0, 0);
    return 1;
}

boolean SpeedController::Curved(int target_velocity_left, int target_velocity_right, int time) //in mm/s and s
{
    motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity_left,target_velocity_right);
    }
    motors.setEfforts(0, 0);
    return 1;
}

void SpeedController::Stop()
{
    motors.setEfforts(0,0);
    odometry.Stop();
    time_track = 0;
}