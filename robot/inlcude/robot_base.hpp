#pragma once
// #include <Eigen/Dense>
class RobotBase{

    public:
        virtual ~RobotBase()=default;

        // defining interface (pure virtual function, base class needs to define these classes)
        virtual void initialize()=0;
        virtual void simStep()=0;

};