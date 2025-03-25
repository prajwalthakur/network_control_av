#include <client.hpp>

static std::vector<double> eigenToBuffer(const Eigen::VectorXd& vec){
    return std::vector<double>(vec.data(),vec.data()+vec.size());
}

static Eigen::VectorXd bufferToEigen(const std::vector<double>& buffer,std::size_t size){
    
    Eigen::VectorXd vec = Eigen::VectorXd(Eigen::Map<const Eigen::VectorXd>(buffer.data(),buffer.size()));
    return vec;
}




RobotClient::RobotClient(const std::string& host,int port,const std::string& robot_name,StateVector init_state,ControlVector init_control)
            :socket(io_service),
            state_timer(io_service),
            ctrl_execute_timer(io_service),
            running(false),
            robot_name(robot_name),
            init_state(init_state),current_control(init_control)
            {

                robot = RobotFactory::createRobot(robot_name);
                state_feedback_dt = 0.02;
                dynamics_update_dt = 0.05;
                robot->initialize(dynamics_update_dt,init_state);
                current_robot_state.resize(7);
                current_robot_state = robot->getState();
                boost::asio::ip::tcp::endpoint end_point(boost::asio::ip::address::from_string(host),port); // connect to ip address defined by host on port 
                socket.connect(end_point);
                std::cout << "Connected to server at "<< host << ":" << port <<std::endl;
               
        
            }

RobotClient::~RobotClient() {

    stop();
}

void RobotClient::start(){

    running  = true;
    client_thread  = std::thread([this](){
            sendState();
            recieveControl();
            executeControl();
            io_service.run();
        });

}


void RobotClient::sendState(){

    if(!running) return;

    current_robot_state = robot->getState();
    // {
    //     std::lock_guard<std::mutex> lock(state_mutex);
    //     sim->run(current_robot_state(0),current_robot_state(1));
    // }
        
    
    
    auto buffer = std::make_shared<std::vector<double>>(eigenToBuffer(current_robot_state));
    boost::asio::async_write(socket,boost::asio::buffer(*buffer),[this, buffer](const boost::system::error_code& error, std::size_t bytes_transferred){
        if(!error){std::cout<< " Send state to server "<<current_robot_state(0)<<" "<<current_robot_state(1)<<std::endl;}
        else{std::cerr<<" state send errorrrrrrr"<< error.message()<<std::endl;}
    });
    state_timer.expires_from_now(std::chrono::milliseconds(static_cast<int>(state_feedback_dt * 1000)));
    state_timer.async_wait([this](const boost::system::error_code& error){
        if(!error){sendState();}
    });
}

void RobotClient::executeControl(){

    if(!running) return;

    ControlVector ctrl = current_control;
    robot->simStep(ctrl);
    ctrl_execute_timer.expires_from_now(std::chrono::milliseconds(static_cast<int>(dynamics_update_dt * 1000)));
    ctrl_execute_timer.async_wait([this](const boost::system::error_code& error){
        if(!error){executeControl();}
    });
}




void RobotClient::recieveControl(){
    auto buffer = std::make_shared<std::vector<double>>(2);
    socket.async_read_some(boost::asio::buffer(*buffer),
        [this, buffer](const boost::system::error_code& error, std::size_t bytes_transferred){
            if(!error){
                current_control = bufferToEigen(*buffer,bytes_transferred/sizeof(double));

            std::cout << "Received control: [" << current_control(0) << ", " << current_control(1) << "]" << std::endl;
            recieveControl();
            }
            else {
                std::cerr << " Control recieve error" <<error.message()<<std::endl;
            }

        });
}

void RobotClient::stop() {
    running = false;

    if (socket.is_open()) {
        std::cout << "Closing client socket." << std::endl;
        socket.close();
    }

    //socket.close();
    io_service.stop();

    if (client_thread.joinable()) {
        client_thread.join();
    }
}


StateVector RobotClient::getState(){
    std::lock_guard<std::mutex> lock(state_mutex);
    return current_robot_state;
}