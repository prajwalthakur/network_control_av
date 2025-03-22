#pragma once
#include "robot_abc.hpp"
#include <memory>
#include <string>

class RobotFactory 
    {
    public:
        static std::shared_ptr<RobotBase> createRobot(const std::string& type){
            if(type=="ABC"){
                return std::make_shared<RobotAbc>();
            }
        }

    };