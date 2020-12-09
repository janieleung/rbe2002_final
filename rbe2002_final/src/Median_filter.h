#ifndef MEDIAN_FILTER //header guard: if the header file is not included,
#define MEDIAN_FILTER //then include, and define this header file

#include <Romi32U4.h> //include the Romi32U4.h header file from library

class MedianFilter{
    private:
        int array[5] = {0}; //initialize the element at index 5 = 0
        
        
    public:
        void Sort(int, int); //The .cpp includes the Sort() function, declaring it public 
        void Init(void);     //The .cpp includes the Init() function, declaring it public 
        int Filter(int);     //The .cpp includes the Filter() function, declaring it public 
};

#endif               //if the header file is defined, then ignore the line in between #ifndef and #endif