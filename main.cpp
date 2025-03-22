#include <iostream>
#include <robot_factory.hpp>
using namespace std;
int main() {
    
    auto robot = RobotFactory::createRobot("ABC");
    double ctrl_dt = 0.01;
    robot->initialize(ctrl_dt); 
    std::cout << "Hello, ddsd!" << std::endl;
    //StateVector st = robot->getState();
    InputVector vt(2); 
    vt << 0.0, 2.0;  
    for(int i=0;i<(int)5/0.01;++i){
        robot->simStep(vt);
    cout<<"state at "<<i<<" ";
    cout<<robot->getState()<<endl;
    }
    return 0;
}
