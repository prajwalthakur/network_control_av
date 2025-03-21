#pragma once
#include "robot_base.hpp"

//parameter class for the abc robot


        // self.mu = 1.0489
        // self.C_Sf = 21.92/1.0489
        // self.C_Sr = 21.92/1.0489
        // self.lf = 0.3048*3.793293
        // self.lr = 0.3048*4.667707
        // self.h = 0.3048*2.01355
        // self.m = 4.4482216152605/0.3048*74.91452
        // self.I = 4.4482216152605*0.3048*1321.416

        // #steering constraints
        // self.s_min = -1.066  #minimum steering angle [rad]
        // self.s_max = 1.066  #maximum steering angle [rad]
        // self.sv_min = -0.4  #minimum steering velocity [rad/s]
        // self.sv_max = 0.4  #maximum steering velocity [rad/s]

        // #longitudinal constraints
        // self.v_min = -13.6  #minimum velocity [m/s]
        // self.v_max = 50.8  #minimum velocity [m/s]
        // self.v_switch = 7.319  #switching velocity [m/s]
        // self.a_max = 11.5  #maximum absolute acceleration [m/s^2]








class ABC_PARAM{
    protected:
        int NX=6;  
        int NU=2;
        float fine_time_step_ = 0.001;
        float mu = 1.0489;
        float C_Sf = 21.92/1.0489;
        float C_Sr = 21.92/1.0489;
        float lf  = 0.3048*3.793293;
        float lr = 0.3048*4.667707;
        float h = 0.3048*2.01355;
        float m = 4.4482216152605/0.3048*74.91452;
        float I = 4.4482216152605*0.3048*1321.416;
        //steering constraints
        float s_min = -1.066;  //minimum steering angle [rad]
        float s_max = 1.066; //maximum steering angle [rad]
        float sv_min = -0.4; //minimum steering velocity [rad/s]
        float sv_max = 0.4;  //maximum steering velocity [rad/s]
        // #longitudinal constraints
        float v_min = -13.6;  //minimum velocity [m/s]
        float v_max = 50.8;  //minimum velocity [m/s]
        float v_switch = 7.319;  //switching velocity [m/s]
        float a_max = 11.5;  //maximum absolute acceleration [m/s^2]
        float a_min = 0.0;
};

// states of the abc robot
struct StateStruct{
    float x;
    float y;
    float front_steer;
    float v_x;
    float yaw;
    float yaw_dot;
    float slip_angle; // slip angle
};

// input of the abc robot
struct InputStruct{
        float steer_dot;
        float acc_x;
};


class RobotAbc: private RobotBase , private ABC_PARAM{
    private:
        
        StateVector _kin_dynamics(const StateStruct&, const InputStruct&);
        StateVector _st_dynamics(const StateStruct&, const InputStruct&);
        StateVector _dynamics(const StateVector&, const InputVector&);
        InputStruct _controlConstraints(InputStruct&);
        StateVector rk4Integrator(const StateVector& , const InputVector& ,float ) override;
        StateVector StateToVector(const StateStruct & ) const;
        StateStruct VectorToState(const StateVector &) const;
        InputVector InputToVector(const InputStruct &) const;
        InputStruct VectorToInput(const InputVector &) const;
    public:
        RobotAbc(float);
        void initialize() override;
        void simStep(const InputVector&) override;
        StateVector getState() override;
        StateVector getControl() override;


};


