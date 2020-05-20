function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters


% FILL IN YOUR CODE HERE

Kp = 55;
Kv = 10;

% Errors
e = s_des(1) - s(1);
e_d = s_des(2) - s(2);

u_input = params.mass * ((Kp * e) + (Kv * e_d) + params.gravity);

u = min(max(u_input, params.u_min), params.u_max);

fprintf('%6s %6s %6s %6s\n', 'h', 'u', 'u_inp', 'e');
fprintf('%6f %6f %6f %6f\n', s(1), u, u_input, e);
end

