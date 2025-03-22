#include "robot_base.hpp"
void RobotBase::_setInput(const InputVector& control_input){
    this->robot_control = control_input;
}

void RobotBase::_setState(const StateVector& state){

    this->robot_state = state;

}