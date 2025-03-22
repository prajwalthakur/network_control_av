#pragma once
#include<iostream>
#include <Eigen/Dense>
typedef Eigen::VectorXd StateVector;
typedef Eigen::VectorXd InputVector;
class RobotBase
    {
        protected:
            StateVector robot_state;
            InputVector robot_control;
            double   ctrl_dt;
            void _setInput(const InputVector&);
            void _setState(const StateVector&);

        public:
            virtual ~RobotBase()=default;

            // defining interface (pure virtual function, base class needs to define these classes)
            virtual void initialize(double)=0;
            virtual void simStep(const InputVector&)=0;
            virtual StateVector getState()const =0;
            virtual InputVector getControl()const =0;   
            // virtual StateType getStateStruct() const = 0;
            // virtual InputType getControlStruct() const = 0;
    };


