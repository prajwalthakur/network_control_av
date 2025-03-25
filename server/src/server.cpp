#include <server.hpp>

static std::vector<double> eigenToBuffer(const Eigen::VectorXd& vec){
    return std::vector<double>(vec.data(),vec.data()+vec.size());
}


static Eigen::VectorXd bufferToEigen(const std::vector<double>& buffer,std::size_t size){
    
    Eigen::VectorXd vec = Eigen::VectorXd(Eigen::Map<const Eigen::VectorXd>(buffer.data(),buffer.size()));
    return vec;
}



//Opens a listening socket  the port.
TCPServer::TCPServer(int port,std::string& file_name,StateVector& init_state):
                port(port),
                acceptor(io_service,boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(),port)),
                socket(io_service), 
                control_timer(io_service),
                current_robot_state(init_state){

            control_dt = 0.05; // 1/(rate at which we need to call the pure pursuit)
            Controller = std::make_shared<PurePursuit>(file_name,current_robot_state);
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
            controlLoop();
            io_service.run();
        }
    );
}

void TCPServer::stop(){
    socket.close();
    if(server_thread.joinable()) server_thread.join();
}

void TCPServer::acceptConnection(){
    //ensure that the socket object remains alive until the lambda (its callback) is executed.
    auto new_socket = std::make_shared<boost::asio::ip::tcp::socket>(io_service);
    acceptor.async_accept(*new_socket,
        [this, new_socket](const boost::system::error_code& error){
            if(!error){
                std::cout << "Client connected" << std::endl;
                //  new_socket for read/write operations.
                socket = std::move(*new_socket); // if you want to assign it to a member
                readState(); // once connection established, read the messages in the socket
            } else {
                std::cerr << "Accept error: " << error.message() << std::endl;
            }
        });
}



//Send vector over socket
void TCPServer::sendControl(const Eigen::VectorXd& control_vector){
    std::shared_ptr<std::vector<double>> buffer;
    // shared pointer is necessary since, callback (lambda) might be called later when after exiting this sendState
    buffer = std::make_shared<std::vector<double>>(eigenToBuffer(control_vector));
    
    boost::asio::async_write(socket,boost::asio::buffer(*buffer),
        [this,buffer](const boost::system::error_code& error,std::size_t bytes_transferred)
            {

                if(!error){
                    std::cout<<"sent control vector to client"<<std::endl;
                    }
                else{
                    std::cerr<<"Error at TCPServer::sendControl" << error.message() <<std::endl;

                }
            }
        );
}

// read the message sent by the client, handle errors like end of file, connection reset by clients
void TCPServer::readState(){
    if (!socket.is_open()){
        std::cerr << "Socket is closed. Waiting for a new connection." << std::endl;
        acceptConnection();
        return; // Prevent further operations on a closed socket.
    }

    auto buffer = std::make_shared<std::vector<double>>(7);
    socket.async_read_some(boost::asio::buffer(*buffer),
        [this, buffer](const boost::system::error_code& error, std::size_t bytes_transferred){
            if(!error){
                //std::lock_guard<std::mutex> lock(state_control_mutex);
                current_robot_state = bufferToEigen(*buffer, bytes_transferred/sizeof(double));
                std::cout << "Received state vector:\n" << current_robot_state.transpose() << std::endl;
                readState();
            } else {
                std::cerr << "Read error: " << error.message() << std::endl;
                if (error == boost::asio::error::eof ||
                    error == boost::asio::error::connection_reset) {
                    std::cerr << "Connection closed by client." << std::endl;
                    socket.close();
                    acceptConnection();
                }
            }
        });
}




// run the loop at fixed hz
void TCPServer::controlLoop(){
    if(!running) return;
    {
        std::lock_guard<std::mutex> lock(state_control_mutex);
        current_control = Controller->computeControl(current_robot_state,current_control);
    }
    
    sendControl(current_control);
    control_timer.expires_from_now(std::chrono::milliseconds(static_cast<int>(control_dt * 1000)));
    control_timer.async_wait([this](const boost::system::error_code& error){
        if(!error){controlLoop();}
        else{std::cerr<<" Error in TCPServer::controlLoop() "<< error.message()<< std::endl;} });
}


// helper function for visualization
StateVector TCPServer::getState(){
    std::lock_guard<std::mutex> lock(state_control_mutex);
    return current_robot_state;
}



