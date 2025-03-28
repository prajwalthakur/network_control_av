#include "robot_abc.hpp"

//helper function to bound the states and control
static auto clip_fxn(auto val, auto min_val, auto max_val){
    if(val<min_val){return min_val;}
    else if(val>=min_val && val<=max_val){return val;}
    else{
        return max_val;
    }
}

// class constructor
RobotAbc::RobotAbc(){
    this->robot_state.resize(NX);
    this->robot_control.resize(NU);
}

void RobotAbc::initialize(double ctrl_dt){

    this->ctrl_dt = ctrl_dt;
    std::cout<<"robot ABC initialized"<<std::endl;

}

void RobotAbc::initialize(double ctrl_dt, StateVector& st){

    this->ctrl_dt = ctrl_dt;
    this->robot_state = st;
    std::cout<<"robot ABC initialized"<<std::endl;

}

// function to bound the control
void RobotAbc::_controlConstraints(InputStruct& input){

    input.acc_x = clip_fxn(input.acc_x, a_min, a_max );
    input.steer_dot = clip_fxn(input.steer_dot, sv_min, sv_max );
}
//function to bound the states 
void RobotAbc::_stateConstraints(StateStruct& st){

    st.yaw = atan2(sin(st.yaw),cos(st.yaw));  // -pi to pi range
    st.slip_angle = atan2(sin(st.slip_angle),cos(st.slip_angle)); //slip angle -pi/2 to pi/2
    st.v_x = clip_fxn(st.v_x,v_min,v_max);
    st.front_steer = clip_fxn(st.front_steer,s_min,s_max);
}

// kinematic based dynamics for very low sppeed (currently for less than 0.3 m/sec)
// ref: https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/blob/master/vehicleModels_commonRoad.pdf
//ref: https://github.com/f1tenth/f1tenth_gym
StateVector RobotAbc::_kin_dynamics(const StateStruct& st, const InputStruct& u){

    auto lwb = lf+lr;
    auto x_dot = st.v_x*cos(st.yaw);
    auto y_dot = st.v_x*sin(st.yaw);
    auto steer_f_dot = u.steer_dot;
    auto v_x_dot = u.acc_x;
    auto yaw_dot  = (st.v_x/lwb)*tan(st.front_steer);
    auto yaw_ddot = (u.acc_x / lwb) * (std::tan(st.yaw)) + (st.v_x / (lwb * std::pow(std::cos(st.front_steer), 2))) *u.steer_dot;
    auto slip_angle_dot = 0.0;
    StateVector stv(NX);
    stv<<x_dot,y_dot,steer_f_dot,v_x_dot,yaw_dot,yaw_ddot,slip_angle_dot;   
    return stv;
}


// single track dynamics of a robot, which considers the forces, frictions
// ref: https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/blob/master/vehicleModels_commonRoad.pdf
//ref: https://github.com/f1tenth/f1tenth_gym
StateVector RobotAbc::_st_dynamics(const StateStruct& st, const InputStruct& u){
    auto lwb = lf + lr;
    auto x_dot = st.v_x*std::cos(st.yaw+st.slip_angle);
    auto y_dot  = st.v_x*std::sin(st.yaw+st.slip_angle);
    auto steer_f_dot = u.steer_dot;
    auto v_x_dot = u.acc_x; 
    auto yaw_dot  = st.yaw_dot;
    auto yaw_ddot = (-mu*m/(st.v_x*Iz*(lf+lr)))*(std::pow(lf,2)*C_Sf*(g*lr-u.acc_x*h) + std::pow(lr,2)*C_Sr*(g*lf + u.steer_dot*h))*st.yaw_dot \
                    + (mu*m/(Iz*(lf+lr)))*(lr*C_Sr*(g*lf+u.acc_x*h)-lf*C_Sf*(g*lr-u.acc_x*h))*st.slip_angle \
                    + (mu*m/(Iz*(lf+lr)))*lf*C_Sf*(g*lr-u.acc_x*h)*st.front_steer;
    auto slip_angle_dot =  (mu/(st.v_x*(lf+lr)))*( 
                            C_Sf*(g*lr-u.acc_x*h)*st.front_steer \
                            - (C_Sr*(g*lf+u.acc_x*h) + C_Sf*(g*lr-u.acc_x*h))*st.slip_angle \
                            + ( C_Sr*(g*lf+u.acc_x*h)*lr -C_Sf*(g*lr - u.acc_x*h) )*(st.yaw_dot/(st.v_x)) 
                            ) - st.yaw_dot;
    StateVector stv(NX);
    stv<<x_dot,y_dot,steer_f_dot,v_x_dot,yaw_dot,yaw_ddot,slip_angle_dot;   
    return  stv;
}


//accepts the steering speed and acceleration and calls the dynamics update
StateVector RobotAbc::_dynamics(const StateVector& st, const ControlVector& u){
        StateStruct state = _vectorToState(st);
        InputStruct input = _vectorToInput(u);
        _controlConstraints(input);
        StateVector stv;
        if((double)abs(state.v_x)<0.3){
            stv = _kin_dynamics(state, input);
        }
        else{

            stv = _st_dynamics(state,input);
        }
        return stv;
}


// PID for converting speed and steering angle reference to the acceleration and steering angle speed
//ref : https://github.com/f1tenth/f1tenth_gym
ControlVector RobotAbc::PID(double speed, double steer, double current_speed, double current_steer, double max_sv, double max_a, \
 double max_v,double min_v)
 {
    double sv = 0.0;
    double accl = 0.0;

    // Steering
    double steer_diff = steer - current_steer;
    if (std::fabs(steer_diff) > 1e-4) {
        sv = (steer_diff / std::fabs(steer_diff)) * max_sv;
    } else {
        sv = 0.0;
    }

    // Velocity difference
    double vel_diff = speed - current_speed;

    // If currently moving forward
    if (current_speed > 0.0) {
        if (vel_diff > 0) {
            // Accelerating
            double kp = 10.0 * max_a / max_v;
            accl = kp * vel_diff;
        } else {
            // Braking
            double kp = 10.0 * max_a / -min_v;
            accl = kp * vel_diff;
        }
    }
    // If currently moving backward
    else {
        if (vel_diff > 0) {
            // Braking
            double kp = 2.0 * max_a / max_v;
            accl = kp * vel_diff;
        } else {
            // Accelerating
            double kp = 2.0 * max_a / -min_v;
            accl = kp * vel_diff;
        }
    }
    ControlVector ctrl;
    ctrl.resize(NU);
    ctrl<<sv,accl;
    return ctrl;
}

// accepts ControlVector (spped and steering reference) and call the internal PID controller to convert the reference to speed and steering reference
// integration through RK4, to get the next state of the car
// fine_time_step is for finer integration step , currently ctrl_dt =  fine_time_step  0.001
void RobotAbc::simStep(const ControlVector& u){
    StateVector x_next = this->robot_state;
    ControlVector ctrl = PID(u(1), u(0),x_next(3), x_next(2), sv_max, a_max, v_max, v_min);
    const int integration_steps = (int)(ctrl_dt/this->fine_time_step_);
    for(int i=0;i<integration_steps;i++){
        x_next=this->_rk4Integrator(x_next,ctrl,fine_time_step_);
        }
    
    StateStruct next_state = _vectorToState(x_next);
     _stateConstraints(next_state);
    x_next = _stateToVector(next_state);
    this->RobotBase::_setState(x_next);
    this->RobotBase::_setInput(ctrl);
    std::cout<<"True states of the Robot"<<std::endl;
    printState();

}

// helper function to convert the state struct to the state-Eigen vector
StateVector RobotAbc::_stateToVector(const StateStruct & state_struct) const{
    StateVector state_vector;
    state_vector.resize(NX);
    state_vector(0) = state_struct.x;
    state_vector(1) = state_struct.y;
    state_vector(2) = state_struct.front_steer;
    state_vector(3) = state_struct.v_x;
    state_vector(4) = state_struct.yaw;
    state_vector(5) = state_struct.yaw_dot;
    state_vector(6) = state_struct.slip_angle;
    return state_vector;
}


// helper function to convert the state-Eigen-vector to the state-struct
StateStruct RobotAbc::_vectorToState(const StateVector & statevector) const{
    StateStruct st;
    st.x = statevector(0);
    st.y = statevector(1);
    st.front_steer = statevector(2);
    st.v_x = statevector(3);
    st.yaw = statevector(4);
    st.yaw_dot = statevector(5);
    st.slip_angle = statevector(6);
    return st;
}


// helper function to convert the Input struct to the Input-Eigen vector
ControlVector RobotAbc::_inputToVector(const InputStruct & input_struct) const{
    ControlVector input_vector;
    input_vector.resize(NU);
    input_vector(0) = input_struct.steer_dot;
    input_vector(1) = input_struct.acc_x;
    return input_vector;

}


// helper function to convert the Control-Eigen-vector to the control-struct
InputStruct RobotAbc::_vectorToInput(const ControlVector & ControlVector) const{
    InputStruct inpt;
    inpt.steer_dot = ControlVector(0);
    inpt.acc_x = ControlVector(1);
    return inpt;
}

// helper to print the states
void RobotAbc::printState() const{

StateStruct st = getStateStruct();

    std::cout << "StateStruct:\n"
            << " x: " << st.x << "\n"
            << " y: " << st.y << "\n"
            << " front_steer: " << st.front_steer << "\n"
            << " v_x: " << st.v_x << "\n"
            << " yaw: " << st.yaw << "\n"
            << " yaw_dot: " << st.yaw_dot << "\n"
            << " slip_angle: " << st.slip_angle << "\n";

}
// helper to print dynamics-control (acc,steering dot)
void RobotAbc::printControl() const{

    InputStruct input = getControlStruct();

    std::cout << "InputStruct:\n"
            << " steer_dot: " << input.steer_dot << "\n"
            << " acc_x: " << input.acc_x << "\n";

};

// rk4 integrator
StateVector RobotAbc::_rk4Integrator(const StateVector& x, const ControlVector& u, double ts) {
    StateVector k1 = _dynamics(x, u);
    StateVector k2 = _dynamics(x + (ts/2.)*k1,u);
    StateVector k3 = _dynamics(x + (ts/2.)*k2,u);
    StateVector k4 = _dynamics(x + (ts/2.)*k3,u);
    StateVector x_next = x + ts*(k1/6.+k2/3.+k3/3.+k4/6.);
    return x_next;
}


// getters for state, control state-struct, control-struct
StateVector RobotAbc::getState()const {

    return this->robot_state;

}
ControlVector RobotAbc::getControl() const{

    return this->robot_control;
}

StateStruct RobotAbc::getStateStruct()const {

    return _vectorToState(this->robot_state);

}


InputStruct RobotAbc::getControlStruct() const{

    return _vectorToInput(this->robot_control);
}

