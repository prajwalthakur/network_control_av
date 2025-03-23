#include "pure_pursuit.hpp"

PurePursuit::PurePursuit(const std::string& file_name, const StateVector& init_state){
    st.resize(num_states);
    control.resize(num_control);
    _load_waypoints(file_name);
    Eigen::Vector3d curr_pose;
    curr_pose<<init_state(0),init_state(1),init_state(index_yaw);
    Eigen::MatrixXd transformed_wp = _transformWaypoints(eig_waypoints, curr_pose);
    int num_rows = transformed_wp.rows();
    // find the indices from the previous closest indices to the next 100 
    int curr_idx = this->current_closest_idx;
    Eigen::VectorXi indices = Eigen::VectorXi::LinSpaced(num_rows, 0,num_rows-1);
    // find the euclid dist from the 
    Eigen::VectorXd distances = transformed_wp(indices, Eigen::all).rowwise().norm();
    Eigen::Index min_index;
    double min_distance = distances.minCoeff(&min_index);
    this->current_closest_idx = indices(min_index);
}

void PurePursuit::_load_waypoints(const std::string& file_name)
{
    std::ifstream file(file_name);
    
    if(!file){
        std::cerr<<"Failed to open file " << file_name<<std::endl;
        return;
    }
    std::string line;
    int i=0;
    while(std::getline(file,line)){
        
        std::stringstream ss(line);
        std::string token;
        std::vector<float> v;
        while (std::getline(ss, token, ',')) {
               v.push_back(stof(token));
        }
        waypoints.emplace_back(coords{v[0], v[1]});
    }
    file.close();
    std::cout<<"Loaded "<< waypoints.size()<< "waypoints"<< std::endl;
    _constructEigenWaypoints();
}


void PurePursuit::_insertInEigen(EigWaypoint& eig_vector,int index, std::vector<double>to_insert)
{
    int num_cols = eig_vector.cols();
    for(int i=0;i<num_cols;++i){
        eig_vector(index,i) = to_insert[i];

    }


}

void PurePursuit::_constructEigenWaypoints(){

    int num_waypoints = waypoints.size();
    eig_waypoints.resize(num_waypoints,num_dim);
    std::vector<double> wp = {waypoints[0].x,waypoints[0].y};
    _insertInEigen(eig_waypoints,0,wp);
    for(int i=1;i<num_waypoints;++i){
        auto diff_x = waypoints[i].x - waypoints[i-1].x;
        auto diff_y = waypoints[i].y - waypoints[i-1].y;
        auto yaw = atan2(diff_y,diff_x);
        wp = {waypoints[i].x,waypoints[i].y};
        _insertInEigen(eig_waypoints,i,wp);
    }
}

    // state_vector(0) = state_struct.x;
    // state_vector(1) = state_struct.y;
    // state_vector(2) = state_struct.front_steer;
    // state_vector(3) = state_struct.v_x;
    // state_vector(4) = state_struct.yaw;
    // state_vector(5) = state_struct.yaw_dot;
    // state_vector(6) = state_struct.slip_angle;
    Eigen::MatrixXd PurePursuit::_transformWaypoints(const EigWaypoint& waypoints, const Eigen::Vector3d& current_pose){
    // Eigen::MatrixXd shifted = waypoints - current_pose(Eigen::seqN(0, 2)).transpose();
    Eigen::MatrixXd shifted = waypoints.rowwise() - current_pose.head<2>().transpose();
    Eigen::Matrix2d R;
    double theta = current_pose(2);
    R<< cos(theta), sin(theta), -sin(theta),cos(theta);
    return (R*shifted.transpose()).transpose();  // Nx2 
}

double PurePursuit::_computeLookAheadDist(Eigen::Vector3d& current_pose,double estimated_look_ahead,Eigen::VectorXd& look_ahead_point){
    std::cout<<"in";
    Eigen::MatrixXd transformed_wp = _transformWaypoints(eig_waypoints, current_pose);
    int num_rows = transformed_wp.rows();
    // find the indices from the previous closest indices to the next 100 
    int curr_idx = this->current_closest_idx;
    Eigen::VectorXi indices = Eigen::VectorXi::LinSpaced(30, 0,29)
                                .unaryExpr([num_rows, curr_idx](int x) { 
                                    return (x + curr_idx) % num_rows; 
                                });
    // find the euclid dist from the 
    Eigen::VectorXd distances = transformed_wp(indices, Eigen::all).rowwise().norm();
    Eigen::Index min_index;
    double min_distance = distances.minCoeff(&min_index);
    this->current_closest_idx = indices(min_index);
    //find the the lookahead point from the this->current_closest_idx
    Eigen::VectorXd diff_distance = (distances.array() - estimated_look_ahead).abs();
    double min_distance_to_actual_lookhead = diff_distance.minCoeff(&min_index);
    double look_ahead_dist = diff_distance(min_index);
    look_ahead_point = transformed_wp.row(indices(min_index));;
    return look_ahead_dist;

}






ControlVector PurePursuit::computeControl(const StateVector& st, const ControlVector& prev_control){
    ControlVector ctrl(2);
    Eigen::Vector3d current_position;
    current_position<<st(0),st(1),st(index_yaw);
    auto v_x = st(index_speed);
    
    double lookahead = std::min(std::max(min_lookahead, max_lookahead * v_x / lookahead_ratio), max_lookahead);
    Eigen::VectorXd look_ahead_point;
    double look_ahead_dist = _computeLookAheadDist( current_position, lookahead, look_ahead_point);
    double alpha = atan2(look_ahead_point(1),look_ahead_point(0));
    ctrl(0) =  atan(2*(lf+lr)*sin(alpha)/(look_ahead_dist));
    ctrl(1) = 1.1;
    return  ctrl;
}
