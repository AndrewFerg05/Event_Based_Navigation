%%
clear all;
%% parameter set up 

R = 2.0;                % Ohms
L = 0.5;                % Henrys
Km = 0.1;               % torque constant
Kb = 0.1;               % back emf constant
Kf = 0.2;               % Nms
J = 0.02;               % kg.m^2/s^2

%% systems model set up 

h1 = tf(Km,[L R]);            % armature
h2 = tf(1,[J Kf]);            % eqn of motion

dcm = ss(h2) * [h1 , 1];      % w = h2 * (h1*Va + Td)
dcm = feedback(dcm,Kb,1,1);   % close back emf loop

% Extract transfer functions for each input-output path
dcm_tf = tf(dcm);            % Convert state-space model to transfer function form

zpk(dcm_tf)

%% system analysis 
bode(dcm);

step(dcm);


