#pragma once
#include "robot_abc.hpp"
#include <memory>
#include <string>

//IO interface to the robot , robotBase is the Base Class, which is inherited by the different robot currently only one Avialable "ABC"
class RobotFactory 
    {
    public:
        static std::shared_ptr<RobotBase> createRobot(const std::string& type){
            if(type=="ABC"){
                return std::make_shared<RobotAbc>();
            }
        }

    };