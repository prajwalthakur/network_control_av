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
TCPServer::TCPServer(int port):port(port),acceptor(io_service,boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(),port)),
        socket(io_service){}

TCPServer::~TCPServer() {

    stop();

}


void TCPServer::start(){
    running = true;
    server_thread = std::thread([this]()
        {
            acceptConnection();
            io_service.run();
        }
    );

}

void TCPServer::acceptConnection(){
    acceptor.async_accept(socket,
        [this](const boost::system::error_code& error){
            if(!error){
                std::cout << "Client connected"<<std::endl;
                readState();
                acceptConnection();
            }
        });
}

// Send Eigen::VectorXd over socket
void TCPServer::sendControl(const Eigen::VectorXd& control_vector){
    // shared pointer is necessary since, callback (lambda) might be called later when after exiting this sendState
    auto buffer = std::make_shared<std::vector<double>>(eigenToBuffer(control_vector));
    boost::asio::async_write(socket,boost::asio::buffer(*buffer),
        [this,buffer](const boost::system::error_code& error,std::size_t bytes_transferred)
            {

                if(!error){
                    std::cout<<"sent state vector to client"<<std::endl;
                    }
            }
        );
}


// read Eigen::VectorXd over socket
void TCPServer::readState()
{  
    auto buffer = std::make_shared<std::vector<double>>(7);
    socket.async_read_some(boost::asio::buffer(*buffer), 
        [this,buffer](const boost::system::error_code& error, std::size_t bytes_transferred){
            if(!error){
                Eigen::VectorXd robot_state = bufferToEigen(*buffer, bytes_transferred);
                std::cout<<"Recieved Control vector:\n"<<ControlVector.transpose()<<std::endl;
                readState();
            }});


}


void TCPServer::stop(){
    socket.close();
    if(server_thread.joinable()) server_thread.join();
}
