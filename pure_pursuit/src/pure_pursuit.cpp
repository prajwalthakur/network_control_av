#include "pure_pursuit.hpp"

PurePursuit::PurePursuit(const std::string& file_name){
    st(num_states);
    control(num_control);
    _load_waypoints(file_name);
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
    std::vector<double> wp = {waypoints[0].x,waypoints[0].y,0.0};
    _insertInEigen(eig_waypoints,0,wp);
    for(int i=1;i<num_waypoints;++i){
        auto diff_x = waypoints[i].x - waypoints[i-1].x;
        auto diff_y = waypoints[i].y - waypoints[i-1].y;
        auto yaw = atan2(diff_y,diff_x);
        wp = {waypoints[0].x,waypoints[0].y,yaw};
        _insertInEigen(eig_waypoints,0,wp);
    }
}

    // state_vector(0) = state_struct.x;
    // state_vector(1) = state_struct.y;
    // state_vector(2) = state_struct.front_steer;
    // state_vector(3) = state_struct.v_x;
    // state_vector(4) = state_struct.yaw;
    // state_vector(5) = state_struct.yaw_dot;
    // state_vector(6) = state_struct.slip_angle;





ControlVector PurePursuit::computeControl(const StateVector& st, const ControlVector& prev_control){
    ControlVector ctrl(2);
    auto v_x = st(index_speed);
    
    double lookahead = std::min(std::max(min_lookahead, max_lookahead * v_x / lookahead_ratio), max_lookahead);

    return  ctrl;
}
