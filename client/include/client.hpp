#pragma once
#include <iostream>
#include <thread>
#include <atomic>
#include <boost/asio.hpp>
#include <Eigen/Dense>
#include <robot_factory.hpp>
#include <vector>
#include <simulator.hpp>
class RobotClient {

    private:
        //void connect();
        void sendState();
        void recieveControl();
        void executeControl();
        boost::asio::io_service io_service;
        boost::asio::ip::tcp::socket socket;
        boost::asio::steady_timer state_timer;
        boost::asio::steady_timer ctrl_execute_timer;
        std::thread client_thread;
        std::atomic<bool> running;
        StateVector current_robot_state;
        ControlVector current_control;
        std::mutex state_mutex;
        std::shared_ptr<RobotBase> robot;
        std::string robot_name;
        StateVector init_state;
        double state_feedback_dt;
        double dynamics_update_dt;
        Simulator* sim;


    public:
        RobotClient(const std::string& host, int port, const std::string& robot_name, StateVector init_state, ControlVector init_control);
        ~RobotClient();
        void start();
        void stop();
        StateVector getState();

};