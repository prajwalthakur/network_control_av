#pragma once
#include "robot_base.hpp"

class ABC_PARAM{
    protected:
        int NX=7;  
        int NU=2;
        double fine_time_step_ = 0.001;
        double mu = 1.0489;
        double C_Sf = 21.92/1.0489;
        double C_Sr = 21.92/1.0489;
        double lf  = 0.3048*3.793293;
        double lr = 0.3048*4.667707;
        double h = 0.3048*2.01355;
        double m = 4.4482216152605/0.3048*74.91452;
        double Iz = 4.4482216152605*0.3048*1321.416;
        //steering constraints
        double s_min = -1.066;  //minimum steering angle [rad]
        double s_max = 1.066; //maximum steering angle [rad]
        double sv_min = -0.4; //minimum steering velocity [rad/s]
        double sv_max = 0.4;  //maximum steering velocity [rad/s]
        // #longitudinal constraints
        double v_min = -13.6;  //minimum velocity [m/s]
        double v_max = 50.8;  //minimum velocity [m/s]
        double v_switch = 7.319;  //switching velocity [m/s]
        double a_max = 11.5;  //maximum absolute acceleration [m/s^2]
        double a_min = 0.0;
        double g =  9.8;
};

// states of the abc robot
struct StateStruct{
    double x;
    double y;
    double front_steer;
    double v_x;
    double yaw;
    double yaw_dot;
    double slip_angle; // slip angle
};

// input of the abc robot
struct InputStruct{
        double steer_dot;
        double acc_x;
};


class RobotAbc: public RobotBase , private ABC_PARAM{
    private:
        StateStruct state_struct;
        InputStruct input_struct;
        StateVector _kin_dynamics(const StateStruct&, const InputStruct&);
        StateVector _st_dynamics(const StateStruct&, const InputStruct&);
        StateVector _dynamics(const StateVector&, const ControlVector&);
        void _controlConstraints(InputStruct&);
        void _stateConstraints(StateStruct&);
        StateVector _rk4Integrator(const StateVector& , const ControlVector& ,double );
        StateVector _stateToVector(const StateStruct & ) const;
        StateStruct _vectorToState(const StateVector &) const;
        ControlVector _inputToVector(const InputStruct &) const;
        InputStruct _vectorToInput(const ControlVector &) const;
    public:
        RobotAbc();
        void initialize(double) override;
        void initialize(double ctrl_dt, StateVector &st) override;
        void simStep(const ControlVector &) override;
        StateVector getState() const override;
        ControlVector getControl() const override;
        virtual void printState() const override;
        virtual void printControl() const override;
        StateStruct getStateStruct() const ;
        InputStruct getControlStruct() const ;
};


