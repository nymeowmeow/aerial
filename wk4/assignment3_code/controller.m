function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
%Thrust
kp = [ 100; 100; 100];
kd = [20; 20; 20];
r_accel = des_state.acc + kp .*  (des_state.pos - state.pos) + ...
          kd .* (des_state.vel - state.vel);
% Thrust
F = params.mass *(params.gravity + r_accel(3));
if F < params.minF
    F = params.minF;
elseif F > params.maxF
    F = params.maxF;
end 
% Moment
M = zeros(3,1);

% =================== Your code ends here ===================
kp_angle = [ 100; 100; 100];
kd_angle = [5;5;5];

phi_des = (1/params.gravity)*(r_accel(1) * sin(des_state.yaw) - ...
             r_accel(2)*cos(des_state.yaw));
theta_des = (1/params.gravity)*(r_accel(1) * cos(des_state.yaw) + ...
             r_accel(2)*sin(des_state.yaw));
rot_des = [ phi_des; theta_des; des_state.yaw];
omega_des = [ 0; 0; des_state.yawdot];

M = kp_angle .* (rot_des - state.rot) + kd_angle .* (omega_des - state.omega);
end
