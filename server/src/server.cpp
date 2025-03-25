#include <server.hpp>

static std::vector<double> eigenToBuffer(const Eigen::VectorXd& vec){
    return std::vector<double>(vec.data(),vec.data()+vec.size());
}

// Convert std::vector<double> to Eigen::VectorXd
static Eigen::VectorXd bufferToEigen(const std::vector<double>& buffer, std::size_t size) {
    return Eigen::Map<const Eigen::VectorXd>(buffer.data(), size);
}

// static std::VectorXd bufferToEigen(const vector<)

//Opens a listening socket  the port.
TCPServer::TCPServer(int port,std::string& file_name):port(port),acceptor(io_service,boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(),port)),
        socket(io_service), control_timer(io_service){

            // robot = RobotFactory::createRobot(robot_name);
            // ctrl_dt = 0.01;
            current_robot_state.resize(7);
            current_control.resize(2); 
            current_robot_state << 15.933784860951967, -4.176775967937121, 0.0, 0.0, -M_PI_2, 0.0, 0.0;
            current_control << -3.0, 2.0;  
            Controller = std::make_shared<PurePursuit>(file_name,current_robot_state);
            control_dt = 0.05;
            std::cout<<" server initialized"<<std::endl; 

        }

TCPServer::~TCPServer() {

    stop();

}


void TCPServer::start(){
    running = true;
    server_thread = std::thread([this]()
        {
            acceptConnection();
            readState();
            controlLoop();
            io_service.run();
        }
    );

}

void TCPServer::acceptConnection(){
    acceptor.async_accept(socket,
        [this](const boost::system::error_code& error){
            if(!error){
                std::cout << "Client connected"<<std::endl;
                acceptConnection();
            }
        });
}

// // Send Eigen::VectorXd over socket
void TCPServer::sendControl(const Eigen::VectorXd& control_vector){
    std::shared_ptr<std::vector<double>> buffer;
    {    
    //Acquires the mutex at the beginning of the scope.


    std::lock_guard<std::mutex> lock(state_control_mutex);
    // shared pointer is necessary since, callback (lambda) might be called later when after exiting this sendState
    buffer = std::make_shared<std::vector<double>>(eigenToBuffer(control_vector));
    //Releases the mutex automatically at the end of the scope.

    }

    boost::asio::async_write(socket,boost::asio::buffer(*buffer),
        [this,buffer](const boost::system::error_code& error,std::size_t bytes_transferred)
            {

                if(!error){
                    std::cout<<"sent state vector to client"<<std::endl;
                    }
            }
        );
}
// Send Eigen::VectorXd over socket
// void TCPServer::sendControl(const Eigen::VectorXd& control_vector){
//     std::ostringstream oss;
//     std::shared_ptr<std::string> buffer;
    
//     {
//         std::lock_guard<std::mutex> lock(state_control_mutex);

//         // Convert to string
//         oss << control_vector(0) << " " << control_vector(1) << "\n";

//         // Store the string in a shared_ptr
//         buffer = std::make_shared<std::string>(oss.str());
//     }

//     // Send the buffer using boost::asio
//     boost::asio::async_write(socket, boost::asio::buffer(*buffer),
//         [this, buffer](const boost::system::error_code& error, std::size_t bytes_transferred) {
//             if (!error) {
//                 std::cout << "Sent control vector to client: " << *buffer;
//             } else {
//                 std::cerr << "Send failed: " << error.message() << std::endl;
//             }
//         });
// }

// read Eigen::VectorXd over socket
void TCPServer::readState()
{  
    auto buffer = std::make_shared<std::vector<double>>(7);
    socket.async_read_some(boost::asio::buffer(*buffer), 
        [this,buffer](const boost::system::error_code& error, std::size_t bytes_transferred){
            if(!error){
                std::lock_guard<std::mutex> lock(state_control_mutex);
                current_robot_state = bufferToEigen(*buffer, bytes_transferred/sizeof(double));
                std::cout<<"Recieved state vector:\n"<< current_robot_state.transpose()<<std::endl;
                readState();
            }});


}


void TCPServer::stop(){
    socket.close();
    if(server_thread.joinable()) server_thread.join();
}

void TCPServer::controlLoop(){
    if(!running) return;
    {
        std::lock_guard<std::mutex> lock(state_control_mutex);
        current_control = Controller->computeControl(current_robot_state,current_control);
    }
    
    sendControl(current_control);
    control_timer.expires_from_now(std::chrono::milliseconds(50));
    control_timer.async_wait([this](const boost::system::error_code& error){
        if(!error){controlLoop();}
        else{std::cerr<<"Time Error"<< error.message()<< std::endl;} });
}
