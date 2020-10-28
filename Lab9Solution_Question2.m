%% Robotics
% Lab 9 - Question 2 - Static Torque

function Lab9Solution_Question2()

close all
clear all
clc

mdl_puma560                                                                 % Load the puma560 model      

tau_max = [97.6 186.4 89.4 24.2 20.1 21.3]';                                % Maximum joint torque of the Puma560

%% Zero position

q = zeros(1,6);                                                             % "Zero" position for the Puma 560
J = p560.jacob0(q);                                                         % Jacobian at this pose
g = p560.gravload(q)';                                                      % Torque due to gravity
m = 0;                                                                      % Initial weight on end-effector
OK = 1;
while OK
    m = m+0.1;                                                              % Increment weight
    w = [0 0 m*9.81 0 0 0]';                                                % Calculate wrench on end-effector
    tau = g + J'*w;                                                         % Calculate the torque needed to hold the weight
    for j = 1:6                                                             % Cylce through joints
        if abs(tau(j)) > tau_max(j)                                         % If torque limit exceeded...
            m = m-0.1;
            OK = 0;                                                         % Set flag to exit while loop                                
        end
    end
end
display(['maximum weight is ',num2str(m)]);                                 % Display maximum weight



%% For a given end-effector pose
T1 = [roty(pi/2) [0.7; 0; 0]; zeros(1,3) 1];                                % Desired end-effector transform
q0 = randperm(6,6);                                                         % Randomize initial guess for joints
q = p560.ikcon(T1,q0);                                                      % Inverse kinematics
T = p560.fkine(q);                                                          % Get the actual pose
norm(T1-T)                                                                  % Norm of pose error
g = p560.gravload(q)';                                                      % Get the gravity torque at this configuration
J = p560.jacob0(q);                                                         % Get the Jacobian at this pose
m = 0;                                                                      % Start the initial mass at 0kg
OK = 1;
while OK
    m = m+0.1;                                                              % Increment the payload weight
    w = [0 0 m*9.81 0 0 0]';                                                % Calculate the wrench
    tau = g + J'*w;                                                         % Calculate the static torque
    for j = 1:6
        if abs(tau(j)) > tau_max(j)                                         % Check if torque limits are exceeded
            m = m-0.1;
            OK = 0;
        end
    end
end
display(['maximum weight is ',num2str(m)]);                                 % Display maximum weight

%% Minimum distance
m = 40;
w = [0 0 m*9.81 0 0 0]';
x = 0.44;
T1 = [roty(pi/2) [x; 0; 0]; zeros(1,3) 1];                                  % Desired end-effector transform
q = p560.ikcon(T1,-ones(1,6));
g = p560.gravload(q)';
J = p560.jacob0(q);
tau = J'*w + g;

display('tau_max and abs(tau) are:');                                       % Display tau_max and tau
[tau_max abs(tau)]

p560.plot(q)