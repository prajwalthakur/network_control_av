#include "robot_abc.hpp"


static auto clip_fxn(auto val, auto min_val, auto max_val){
    if(val<min_val){return min_val;}
    else if(val>=min_val && val<=max_val){return val;}
    else{
        return max_val;
    }
}





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


void RobotAbc::_controlConstraints(InputStruct& input){

    input.acc_x = clip_fxn(input.acc_x, a_min, a_max );
    input.steer_dot = clip_fxn(input.steer_dot, sv_min, sv_max );
}
void RobotAbc::_stateConstraints(StateStruct& st){

    st.yaw = atan2(sin(st.yaw),cos(st.yaw));  // -pi to pi range
    st.slip_angle = atan2(sin(st.slip_angle),cos(st.slip_angle)); //slip angle -pi/2 to pi/2
    st.v_x = clip_fxn(st.v_x,v_min,v_max);
    st.front_steer = clip_fxn(st.front_steer,s_min,s_max);
}


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



StateVector RobotAbc::_dynamics(const StateVector& st, const InputVector& u){
        StateStruct state = _vectorToState(st);
        InputStruct input = _vectorToInput(u);
        _controlConstraints(input);
        StateVector stv;
        if((double)abs(state.v_x)<0.5){
            stv = _kin_dynamics(state, input);
        }
        else{

            stv = _st_dynamics(state,input);
        }
        return stv;
}

StateVector RobotAbc::getState()const {

    return this->robot_state;

}
InputVector RobotAbc::getControl() const{

    return this->robot_control;
}

StateStruct RobotAbc::getStateStruct()const {

    return _vectorToState(this->robot_state);

}


InputStruct RobotAbc::getControlStruct() const{

    return _vectorToInput(this->robot_control);
}




StateVector RobotAbc::_rk4Integrator(const StateVector& x, const InputVector& u, double ts) {
    StateVector k1 = _dynamics(x, u);
    StateVector k2 = _dynamics(x + (ts/2.)*k1,u);
    StateVector k3 = _dynamics(x + (ts/2.)*k2,u);
    StateVector k4 = _dynamics(x + (ts/2.)*k3,u);
    StateVector x_next = x + ts*(k1/6.+k2/3.+k3/3.+k4/6.);
    return x_next;
}

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

InputVector RobotAbc::_inputToVector(const InputStruct & input_struct) const{
    InputVector input_vector;
    input_vector.resize(NU);
    input_vector(0) = input_struct.steer_dot;
    input_vector(1) = input_struct.acc_x;
    return input_vector;

}

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


InputStruct RobotAbc::_vectorToInput(const InputVector & inputvector) const{
    InputStruct inpt;
    inpt.steer_dot = inputvector(0);
    inpt.acc_x = inputvector(1);
    return inpt;
}



void RobotAbc::simStep(const InputVector& u){
    StateVector x_next = this->robot_state;
    const int integration_steps = (int)(ctrl_dt/this->fine_time_step_);
    for(int i=0;i<integration_steps;i++){
        x_next=this->_rk4Integrator(x_next,u,fine_time_step_);
        }
    
    StateStruct next_state = _vectorToState(x_next);
     _stateConstraints(next_state);
    x_next = _stateToVector(next_state);
    this->RobotBase::_setState(x_next);
    this->RobotBase::_setInput(u);

}