#pragma once
#include "robot_base.hpp"

class ABC_PARAM{
    protected:
        int NX=7;  
        int NU=2;
        double fine_time_step_ = 0.001;
        double mu = 1.0489;
        double C_Sf = 4.718;
        double C_Sr = 5.4562;
        double lf  = 0.15875;
        double lr =  0.17145;
        double h =  0.074;
        double m = 3.74;
        double Iz = 0.04712;
        //steering constraints
        double s_min = -0.4189;  //minimum steering angle [rad]
        double s_max = 0.4189; //maximum steering angle [rad]
        double sv_min = -3.2; //minimum steering velocity [rad/s]
        double sv_max = 3.2;  //maximum steering velocity [rad/s]
        // #longitudinal constraints
        double v_min = -5.0;  //minimum velocity [m/s]
        double v_max = 5.0;  //minimum velocity [m/s]
        double v_switch = 7.319;  //switching velocity [m/s]
        double a_max = 5.0;  //maximum absolute acceleration [m/s^2]
        double a_min = -5.0;
        double g =  9.8;
        // params (dict, default={'mu': 1.0489, 'C_Sf':, 'C_Sr':, 'lf': 0.15875, 'lr': 0.17145, 'h': 0.074, 'm': 3.74, 'I': 0.04712, 's_min': -0.4189, 's_max': 0.4189, 'sv_min': -3.2, 'sv_max': 3.2, 'v_switch':7.319, 'a_max': 9.51, 'v_min':-5.0, 'v_max': 20.0, 'width': 0.31, 'length': 0.58}):
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
        ControlVector PID(double , double , double , double , double , double , double ,double );

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


