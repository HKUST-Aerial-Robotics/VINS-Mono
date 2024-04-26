#include "parameters_.h"

double toSec(headerMsg header_time){
    return static_cast<double>(header_time.stamp.sec + (header_time.stamp.nanosec /  1.0e9));
}