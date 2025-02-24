#ifndef DRIVER_HPP
#define  DRIVER_HPP

#include<iostream>
#include<vector>
#include<eigen3/Eigen/Dense>
#include<cmath>

class driverInterface{



public:

    virtual  Eigen::VectorXf getPose() = 0;
    virtual Eigen::VectorXf getVelocity() = 0;


};


namespace driver{

    class diff_drive : public driverInterface{


        public:

            diff_drive();
            diff_drive(float wheel_sep, float wheel_rad, float distPerTick);
            Eigen::VectorXf getPose() override;
            Eigen::VectorXf getVelocity() override;
            void computeVelocity(Eigen::Vector3f pos, double duration);
            void computePose(int prevLeft_tks, int currentLeft_tks, int prevRight_tks, int currentRight_tks);
            void setwhl_sep(float wheel_sep);
            void setwhl_rad(float wheel_rad);
            Eigen::Vector3f getlocalPos();


        private:

            float d_l = 0.0f;
            float d_r = 0.0f;
            float d = 0.0f;
            float dx = 0.0f;
            float dy = 0.0f;
            float theta = 0.0f;
            float delta_d = 0.0f;
            float delta_theta = 0.0f;
            float whl_sep;
            float whl_rad;
            float distancePerTick;
            Eigen::VectorXf posVector;
            Eigen::VectorXf velocityVector;
            Eigen::Vector3f localpos;
            


    };

    // class ackerman : driverInterface{


    //     // public:

    //     //     ackerman();



    // };



}


#endif