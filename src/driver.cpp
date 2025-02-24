#include "driver.hpp"
#include<iostream>
#include<vector>
#include<eigen3/Eigen/Dense>
#include<cmath>


driver::diff_drive::diff_drive(){}

driver::diff_drive::diff_drive(float wheel_sep, float wheel_rad, float distPerTick) 
   : whl_sep{wheel_sep},
     whl_rad{wheel_rad},
     distancePerTick{distPerTick}
{}

void driver::diff_drive::setwhl_rad(float wheel_rad){whl_rad = wheel_rad;}

void driver::diff_drive::setwhl_sep(float wheel_sep){whl_sep = wheel_sep;}

Eigen::VectorXf driver::diff_drive::getPose(){

    return posVector;


}

void driver::diff_drive::computePose(int prevLeft_tks, int currentLeft_tks, int prevRight_tks, int currentRight_tks){

    if(prevLeft_tks == 0 && currentLeft_tks == 0 && prevRight_tks == 0 && currentRight_tks == 0){

         d_l = 0.0;
         d_r = 0.0;
         d = 0.0;
         theta = 0.0;

    }

         // calculate distance traveled based on encoder ticks
         d_l = float((currentLeft_tks - prevLeft_tks) * distancePerTick);
         d_r = float((currentRight_tks - prevRight_tks) * distancePerTick);
         d = (d_l + d_r)/2.0;
         theta = float((d_l - d_r)/whl_sep);


    // change in distance in the x and y in the local coordinate

    dx = d * std::cos(theta);
    dy = d * std::sin(theta);

    // distance traveld in world coordinate

    Eigen::Vector3f pose = {(dx * std::cos(theta) - dy * std::sin(theta)), (dx * std::sin(theta) + dy * std::cos(theta)), theta}; 

    posVector = pose;


 }


 void driver::diff_drive::computeVelocity(Eigen::Vector3f pos, double duration){


    velocityVector = 1/duration * pos;

 }

 Eigen::VectorXf driver::diff_drive::getVelocity(){

    return velocityVector;

 }

 Eigen::Vector3f driver::diff_drive::getlocalPos(){

   localpos = {dx, dy, theta};

   return localpos;

 }