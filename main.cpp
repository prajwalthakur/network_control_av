

#include <iostream>
#include <robot_factory.hpp>
#include <simulator.hpp>
#include  <pure_pursuit.hpp>
#include <server.hpp>
#include <client.hpp>
using namespace std;
int main() {
    int width = 800;
    int height = 600;

    // CSV file containing waypoints
    std::string file_name = "/home/prajwal/projects/network_control_av/maps/e7_floor5_square.csv";
    
    // Create the simulator window
    Simulator simulator(width, height, file_name);
    //auto robot = RobotFactory::createRobot("ABC");
    double ctrl_dt = 0.02;
    StateVector st(7);
    //st << 15.933784860951967, -4.176775967937121, 0.0, 0.0, -M_PI_2, 0.0, 0.0;
    st << -0.1769055,-7.8900953, 0.0, 0.0, M_PI_2, 0.0, 0.0;
    
    //robot->initialize(ctrl_dt,st); 
    std::cout << "Hello, ddsd!" << std::endl;
    //StateVector st = robot->getState();
    ControlVector ct(2); 
    ct << 0.0,0.0;  
    TCPServer server(8080, file_name,st);
    RobotClient client("127.0.0.1",8080,"ABC", st, ct) ;
    std::thread server_thread([&server, &client]() {
        server.start();
        client.start();
    });
    std::cout << "Server started on port 8080" << std::endl;
    for(int i=0;i<(int)100/ctrl_dt;++i){
        //robot->simStep(ct);
        StateVector st = server.getState();
        simulator.run(st[0],st[1],st[4]);
        sf::sleep(sf::milliseconds(16)); 
        cout<<"state at "<<i<<" "<<st(0)<<st(1) << endl;
        //robot->printState();
        //cout<<"ctrl: ref-str" << ct(0) << " ref-speed " <<ct(1);
        cout<<"----------";
        cout<<endl;
    }
    return 0;
}




















// #include <iostream>
// #include <robot_factory.hpp>
// #include <simulator.hpp>
// #include  <pure_pursuit.hpp>
// using namespace std;
// int main() {
//     int width = 800;
//     int height = 600;

//     // CSV file containing waypoints
//     std::string file_name = "/home/prajwal/projects/network_control_av/maps/Sakhir_centerline.csv";

//     // Create the simulator window
//     Simulator simulator(width, height, file_name);
//     auto robot = RobotFactory::createRobot("ABC");
//     double ctrl_dt = 0.02;
//     StateVector st(7);
//     st << 15.933784860951967, -4.176775967937121, 0.0, 0.0, -M_PI_2, 0.0, 0.0;
//     robot->initialize(ctrl_dt,st); 
//     std::cout << "Hello, ddsd!" << std::endl;
//     //StateVector st = robot->getState();
//     ControlVector ct(2); 
//     ct << -3.0, 2.0;  
//     auto pure_pur = std::make_shared<PurePursuit>(file_name,st);

//     for(int i=0;i<(int)100/ctrl_dt;++i){
//         robot->simStep(ct);
//         StateVector st = robot->getState();
//         simulator.run(st[0],st[1]);
//         ct = pure_pur->computeControl(st,ct);
//         sf::sleep(sf::milliseconds(16)); 
//         cout<<"state at "<<i<<" "<<endl;
//         robot->printState();
//         cout<<"ctrl: ref-str" << ct(0) << " ref-speed " <<ct(1);
//         cout<<"----------";
//         cout<<endl;
//     }
//     return 0;
// }
