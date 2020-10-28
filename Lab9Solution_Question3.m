%% Robotics
% Lab 9 - Question 3 - Dynamic Torque

function Lab9Solution_Question3()

close all
clear all
clc

mdl_puma560    
qZero = zeros(1,6);

%%%%%%%%%% Variables to change %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time = 20;                                                                  % Total time to execute the motion
T1 = [[0 -1 0; 0 0 1; -1 0 0] [0;0.7;0];zeros(1,3) 1];                      % First pose
q1 = p560.ikcon(T1,qZero);                                                     % Inverse kinematics for 1st pose
T2 = [[0 0 1;0 -1 0; 1 0 0] [0.5;0;0.6];zeros(1,3) 1];                      % Second pose
q2 = p560.ikcon(T2,qZero);                                                     % Inverse kinematics for 2nd pose

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = 1/100;                                                                 % Set control frequency at 100Hz
steps = time/dt;                                                            % No. of steps along trajectory

% s = lspb(0,1,steps);                                                      % Generate trapezoidal velocity profile
% for i = 1:steps
%     q(i,:) = (1-s(i))*q1 + s(i)*q2;
% end
q = jtraj(q1,q2,steps);                                                     % Quintic polynomial profile

qd = zeros(steps,6);                                                        % Array of joint velocities
qdd = nan(steps,6);                                                         % Array of joint accelerations
tau = nan(steps,6);                                                         % Array of joint torques
mass = 21;                                                                  % Payload mass (kg)
p560.payload(mass,[0.1;0;0]);                                               % Set payload mass in Puma 560 model: offset 0.1m in x-direction

for i = 1:steps-1
    qdd(i,:) = (1/dt)^2 * (q(i+1,:) - q(i,:) - dt*qd(i,:));                 % Calculate joint acceleration to get to next set of joint angles
    M = p560.inertia(q(i,:));                                               % Calculate inertia matrix at this pose
    C = p560.coriolis(q(i,:),qd(i,:));                                      % Calculate coriolis matrix at this pose
    g = p560.gravload(q(i,:));                                              % Calculate gravity vector at this pose
    tau(i,:) = (M*qdd(i,:)' + C*qd(i,:)' + g')';                            % Calculate the joint torque needed
    for j = 1:6
        if abs(tau(i,j)) > tau_max(j)                                       % Check if torque exceeds limits
            tau(i,j) = sign(tau(i,j))*tau_max(j);                           % Cap joint torque if above limits
        end
    end
    qdd(i,:) = (inv(M)*(tau(i,:)' - C*qd(i,:)' - g'))';                     % Re-calculate acceleration based on actual torque
    q(i+1,:) = q(i,:) + dt*qd(i,:) + dt^2*qdd(i,:);                         % Update joint angles based on actual acceleration
    qd(i+1,:) = qd(i,:) + dt*qdd(i,:);                                      % Update the velocity for the next pose
end

t = 0:dt:(steps-1)*dt;                                                      % Generate time vector

%% Visulalisation and plotting of results

% Plot joint angles
figure(1)
for j = 1:6
    subplot(3,2,j)
    plot(t,q(:,j)','k','LineWidth',1);
    refline(0,p560.qlim(j,1));
    refline(0,p560.qlim(j,2));
    ylabel('Angle (rad)');
    box off
end

% Plot joint velocities
figure(2)
for j = 1:6
    subplot(3,2,j)
    plot(t,qd(:,j)*30/pi,'k','LineWidth',1);
    refline(0,0);
    ylabel('Velocity (RPM)');
    box off
end

% Plot joint acceleration
figure(3)
for j = 1:6
    subplot(3,2,j)
    plot(t,qdd(:,j),'k','LineWidth',1);
    ylabel('rad/s/s');
    refline(0,0)
    box off
end

% Plot joint torques
figure(4)
for j = 1:6
    subplot(3,2,j)
    plot(t,tau(:,j),'k','LineWidth',1);
    refline(0,tau_max(j));
    refline(0,-tau_max(j));
    ylabel('Nm');
    box off
end

% figure(6)
% p560.plot(q,'fps',steps)

