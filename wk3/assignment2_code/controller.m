function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

kp_y = 20;
kv_y = 10;
kp_z = 100;
kv_z = 20;
kp_phi = 1000;
kv_phi = 20;
phi = state.rot(1);
phi_dot = state.omega(1);

phi_c = (-1/params.gravity)*(des_state.acc(1) + ...
    kv_y*(des_state.vel(1) - state.vel(1)) + ...
    kp_y*(des_state.pos(1) - state.pos(1)));
phi_dot_c = 0;

u1 = params.mass*(params.gravity + kv_z*(des_state.vel(2) - state.vel(2)) ...
        + kp_z*(des_state.pos(2) - state.pos(2)) + des_state.acc(2));
u2 = params.Ixx*(kv_phi*(phi_dot_c - phi_dot) + kp_phi*(phi_c  - phi));

% FILL IN YOUR CODE HERE

end

