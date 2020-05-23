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

% u1 = 0;
% u2 = 0;

% FILL IN YOUR CODE HERE

% Quadrotor parameters
m = params.mass;
g = params.gravity;
Ixx = params.Ixx;
% L = params.arm_length;
% minF = params.minF;
% maxF = params.maxF;

% PD controller gains
kp_y = 0;
kv_y = 5;
kp_z = 70;
kv_z = 30;
kp_phi = 180;
kv_phi = 15;

u1 = m * (g + des_state.acc(2) + (kv_z * (des_state.vel(2) - state.vel(2))) + (kp_z * (des_state.pos(2) - state.pos(2))));
phi_c = (-1/g) * (des_state.acc(1) + (kv_y * (des_state.vel(1) - state.vel(1))) + (kp_y * (des_state.pos(1) - state.pos(1))));

% Neglect the second-order derivative of the rotation angle
u2 = Ixx * ((kv_phi * (-state.omega)) + (kp_phi * (phi_c - state.rot)));

% Display state
fprintf('%6s %6s %6s %6s %6s %6s\n', 'u1', 'u2', 'phi_c', 'y_T', 'yd_T', 'ydd_t');
fprintf('%6f %6f %6f %6f %6f %6f\n\n', u1, u2, phi_c, des_state.pos(1), des_state.vel(1), des_state.acc(1));

end

