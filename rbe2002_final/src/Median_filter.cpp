#include "Median_filter.h"

void MedianFilter::Init(void)
{
    // no initialization required, but good practice
}

void MedianFilter::Sort(int index_a, int index_b)
{
    if(array[index_a] < array[index_b]){    //Compare the element at index A and index B
        int temp = array[index_a];          //if index B > index A, set a temporary variable to index A
        array[index_a] = array[index_b];    //Arrange element at index B to index A at, and the element
        array[index_b] = temp;              //at index A to index B, sorting the element in a 
                                            //descending order
    }
}

int MedianFilter::Filter(int measurement)
{
    array[0] = measurement;                 //setting the element at index 0 to the value given
    for(int i = 4; i > 0; i--)              //loop through the array
     array[i] = array[i-1];                 //moving all the elements to one less in the array index
    
    Sort(0,1);                              //sort element at index 0 and index 1 in descending order
    Sort(3,4);                              //sort element at index 3 and index 4 in descending order
    Sort(0,2);                              //sort element at index 0 and index 2 in descending order
    Sort(1,2);                              //sort element at index 1 and index 2 in descending order
    Sort(0,3);                              //sort element at index 0 and index 3 in descending order
    Sort(2,3);                              //sort element at index 2 and index 3 in descending order
    Sort(1,4);                              //sort element at index 1 and index 4 in descending order
    Sort(1,2);                              //sort element at index 1 and index 2 in descending order
    Sort(3,4);                              //sort element at index 3 and index 4 in descending order

    return array[2];                        //return the median
}