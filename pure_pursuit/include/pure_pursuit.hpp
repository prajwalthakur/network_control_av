#pragma once
#include <iostream>
#include <robot_base.hpp>
#include <vector>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <math.h>
typedef Eigen::Matrix<double, Eigen::Dynamic, 2 > EigWaypoint;


struct coords{
    double x=0;
    double y=0;
};

// pure pursuit parameters
class PurePursuitParam{
    protected:
        double min_lookahead =  0.9;
        double max_lookahead = 2.0; 
        double lookahead_ratio =  8.0; 
        double K_p = 0.4;
        double steering_limit = 25.0;
        double velocity_percentage = 1.0 ;
        int index_speed = 3;
        int index_yaw = 4;
        double lf  = 0.3048*3.793293;
        double lr = 0.3048*4.667707;
};


class PurePursuit: protected PurePursuitParam{
    private:
        StateVector current_state;
        ControlVector control;
        std::vector<coords> waypoints;
        EigWaypoint eig_waypoints;
        void _load_waypoints(const std::string&);
        void _constructEigenWaypoints();
        void _insertInEigen(EigWaypoint& ,int , std::vector<double>to_insert);
        double _computeLookAheadDist(Eigen::Vector3d&,double,Eigen::VectorXd&);
        Eigen::MatrixXd _transformWaypoints(const EigWaypoint&, const Eigen::Vector3d&);
        int num_dim = 2;
        int num_control = 2;
        int num_states = 7;
        int current_closest_idx = 0;
        double ref_speed = 1.0;
    public:
        PurePursuit(const std::string&,const StateVector&, const ControlVector&  );
        ControlVector computeControl(const StateVector&,const ControlVector&);  
};
