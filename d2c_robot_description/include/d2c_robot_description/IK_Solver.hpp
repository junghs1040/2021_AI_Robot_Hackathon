#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>




class Robot 
{
    public:

        std::vector<double> Leg_IK(std::vector<double> final_value);

        

    private:
        std::vector<double> final_value;
        std::vector<double> link_length;       

};



#endif