#pragma once
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <boost/asio.hpp>
#include <Eigen/Dense>
#include <robot_factory.hpp>
#include <pure_pursuit.hpp>
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
        StateVector current_robot_state;
        ControlVector current_control;
        //std::shared_ptr<RobotBase> robot;
        //double ctrl_dt;
         Eigen::VectorXd initial_st;
         boost::asio::steady_timer control_timer; //  Timer for control loop
         std::shared_ptr<PurePursuit> Controller ;
         double control_dt; // 1/(rate to call pure pursuit)
         std::mutex state_control_mutex;
         //void handleClient(std::shared_ptr<boost::asio::ip::tcp::socket> );
    public: 
        TCPServer(int,std::string&,StateVector&);
        ~TCPServer();
        void start();
        void stop();
        void sendControl(const Eigen::VectorXd&);
        void controlLoop();
        void readState();
        StateVector getState();
};