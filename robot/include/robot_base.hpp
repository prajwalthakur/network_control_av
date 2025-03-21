#pragma once
#include <Eigen/Dense>
typedef Eigen::VectorXd StateVector;
typedef Eigen::VectorXd InputVector;
class RobotBase{
    protected:
        StateVector robot_state;
        InputVector robot_control;
        float   ctrl_dt;
        void _setInput(const InputVector&);
        void _setState(const StateVector&);

    public:
        virtual ~RobotBase()=default;

        // defining interface (pure virtual function, base class needs to define these classes)
        virtual void initialize()=0;
        virtual void simStep(const InputVector&)=0;
        virtual StateVector getState()=0;
        virtual StateVector getControl()=0;
        virtual StateVector rk4Integrator(const StateVector& , const InputVector& ,float )=0;

        
};