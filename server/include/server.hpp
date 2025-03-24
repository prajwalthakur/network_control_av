#pragma once
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <boost/asio.hpp>
#include <Eigen/Dense>
class TCPServer{
    private:
        void acceptConnection();
        int port;
        std::atomic<bool> running;
        

        std::thread server_thread;
        boost::asio::io_service io_service;
        boost::asio::ip::tcp::acceptor acceptor;
        boost::asio::ip::tcp::socket socket;
        std::mutex data_mutex;
        Eigen::VectorXd StateVector;
        Eigen::VectorXd ControlVector;
    public: 
        TCPServer(int);
        ~TCPServer();
        void start();
        void stop();
        void sendControl(const Eigen::VectorXd&);
        void readState();
};