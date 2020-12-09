#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

class Behaviors{
    private:
        int threshold = 275;
        //int threshold_pick_up = 1290;
        int threshold_pick_up = 1100;
        int data[3] = {0};
        enum ROBOT_STATE {IDLE, DRIVE, COLLISION, TURN, WALLFOLLOW, RAMP};
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE
         
    public:
        void Init(void);
        void Stop(void);
        void Run(void);
        boolean DetectCollision(void);
        double lengthCourse = 0.3;
};
#endif