#pragma once
#include <robot_base.hpp>
#include <vector>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <math.h>
typedef Eigen::Matrix<double, Eigen::Dynamic, 3 > EigWaypoint;
struct coords{
    double x=0;
    double y=0;
};

class PurePursuitParam{
    protected:
        double min_lookahead =  0.5;
        double max_lookahead = 3.0; 
        double lookahead_ratio =  8.0; 
        double K_p = 0.5;
        double steering_limit = 25.0;
        double velocity_percentage = 1.0 ;
        int index_speed = 3;
};


class PurePursuit: protected PurePursuitParam{
    private:
        StateVector st;
        ControlVector control;
        std::vector<coords> waypoints;
        EigWaypoint eig_waypoints;
        void _load_waypoints(const std::string&);
        void _constructEigenWaypoints();
        void _insertInEigen(EigWaypoint& ,int , std::vector<double>to_insert);
        int num_dim =3 ;
        int num_control = 2;
        int num_states = 7;
    public:
        PurePursuit(const std::string& );
        ControlVector computeControl(const StateVector&,const ControlVector&);  

};
