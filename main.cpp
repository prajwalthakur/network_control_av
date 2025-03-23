#include <iostream>
#include <robot_factory.hpp>
#include <simulator.hpp>
using namespace std;
int main() {
    int width = 800;
    int height = 600;

    // CSV file containing waypoints
    std::string file_name = "/home/prajwal/projects/network_control_av/e7_floor5_square.csv";

    // Create the simulator window
    Simulator simulator(width, height, file_name);
    auto robot = RobotFactory::createRobot("ABC");
    double ctrl_dt = 0.01;
    StateVector st(7);
    st << -0.1769055, -7.8900953, 0.0, 0.0, 0.0, 0.0, 0.0;
    robot->initialize(ctrl_dt,st); 
    std::cout << "Hello, ddsd!" << std::endl;
    //StateVector st = robot->getState();
    ControlVector vt(2); 
    vt << -3.0, 2.0;  
    for(int i=0;i<(int)10/0.01;++i){
        robot->simStep(vt);
        StateVector st = robot->getState();
        simulator.run(st[0],st[1]);
        sf::sleep(sf::milliseconds(16)); 
        cout<<"state at "<<i<<" "<<endl;
        robot->printState();
        cout<<endl;
    }
    return 0;
}
