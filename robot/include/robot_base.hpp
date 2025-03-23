#pragma once
#include<iostream>
#include <Eigen/Dense>
typedef Eigen::VectorXd StateVector;
typedef Eigen::VectorXd ControlVector;
class RobotBase
    {
        protected:
            StateVector robot_state;
            ControlVector robot_control;
            double   ctrl_dt;
            void _setInput(const ControlVector&);
            void _setState(const StateVector&);

        public:
            virtual ~RobotBase()=default;

            // defining interface (pure virtual function, base class needs to define these classes)
            virtual void initialize(double)=0;
            virtual void initialize(double , StateVector& )=0;
            virtual void simStep(const ControlVector&)=0;
            virtual StateVector getState()const =0;
            virtual ControlVector getControl()const =0;
            virtual void printState() const =   0;
            virtual void printControl() const = 0;
            // virtual StateType getStateStruct() const = 0;
            // virtual InputType getControlStruct() const = 0;
    };


