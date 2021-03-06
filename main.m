clf;
close all;
clear all;

% Creates a log of the command window and clears any previous logs
dfile ='CommandWindowLog';
% if exist(dfile, 'file') ; delete(dfile); end
diary(dfile)
diary on

hold on;

% Orient the plot/workspace viewport

% azimuth, az, is the horizontal rotation about the z-axis as measured in degrees from the negative y-axis.
% Positive values indicate counterclockwise rotation of the viewpoint
az = -45;
% el is the vertical elevation of the viewpoint in degrees.
% Positive values of elevation correspond to moving above the object; negative values correspond to moving below the object.
el = 15;

view(az,el);

%% Place the robot base according to your student number.
%  For an 8 digit student number: xxxxyyyy, the base must be [x.xxx,y.yyy,1]meters.
%  So, for student number: 10052821,  robot.base = transl( [1.005, 2.821, 1]);

mdl_puma560;
robot = p560;

% Student number = 12876785
baseLocation = [1.287,6.785,1];
robot.base = transl(baseLocation);

axis([-2+baseLocation(1) 2+baseLocation(1) -2+baseLocation(2) 2+baseLocation(2) -1.5 2.5]);
robot.teach();
startQ = [0,0,0,0,0,0];
robot.animate(startQ);


%% Show calculations to determine the tool offset as a homogeneous transform matrix to attach the blasting nozzle (shown in the figure) to the robot (4)

blastStreamAngle = deg2rad(45);
toolOffset = [0,0,(200*tan(blastStreamAngle))/1000]; % Tool offset in meters

toolTransform = transl(toolOffset);
robot.tool=toolTransform;
disp("Robot Tool Transform");
disp(robot.tool);

% robotTransform = robot.fkine(robot.getpos);
% Transform = robotTransform*toolTransform;

%% Choose a location for the drum based upon your personalised robot location

drumPosition = baseLocation;
drumPosition(1) = drumPosition(1)-0.75;
drumPosition(3) = drumPosition(3)-1.5;

drum = LoadObject("Drum.ply",drumPosition,0);

%% In your report, show (a) the robot base transform, (b) the drum transform and (c) the transform between the robot base and the drum

robotBaseTransform = robot.base;
disp("Robot Base Transform");
disp(robotBaseTransform);

drumTransform = transl(drumPosition);
disp("Drum Transform");
disp(drumTransform);

robotDrumTransform = robotBaseTransform * drumTransform;
disp("Transform Between Base and Drum");
disp(robotDrumTransform);

%% Show how you used an inverse kinematic solver to find a starting pose that points the nozzle at one corner of the white window on the drum (2).
%  Remember you can use “Tools->Data Tips” in the Matlab figure window to find the [x,y,z] of the corners of the white window.

% Window corner point from data tips = [1.983,6.84,0.591]

drumOriginToCorner1=[0.3856,0.0550,0.5910];
windowCorner1 = drumPosition+drumOriginToCorner1;

drumOriginToCorner2=[0.1964,0.0550,0.5910];
windowCorner2 = drumPosition+drumOriginToCorner2;

robot.animate(deg2rad([0,170,-35,0,0,0])); %initial guess for ikcon

gritBlastHeight = 0.3;

% windowCorner = [0.8,6.84,0.591];
startPose = windowCorner1;
startPose(3) = startPose(3)+gritBlastHeight;
% startPose(1) = startPose(1)+0.2;
robotPose = CalculateQ(robot,startPose);
robot.animate(robotPose);

% robotStartTransform = robot.fkine(robot.getpos);
endPoint = windowCorner2;
endPoint(3) = endPoint(3)+gritBlastHeight;
% endPoint(1) = endPoint(1) - 0.2;

velocity = 0.1; % Below overload velocity of "approx" 0.6 m/s

% Control allignment of end effector along trajectory
rpy=tr2rpy(robot.fkine(robot.getpos));
rpy(3)=rpy(3)+pi; % Allign end effector z axis
rpy(1)=rpy(1)-pi/4 % Allgin end effector so that the blast stream is parallel to the gravity vector
axis = -rpy; % Move along x axis

launching = false;
overload = true; % True only works if the path is long enough or timeStep is small. Otherwise the number of steps can approach zero

velocity = 0.1; % set the velocity to make sure it is intially below the overload velocity

% Calaculate the time to animate the robot through 1 tejectory step.
% Varies based on computer speed
% So Use the timeStep as the time step for RMRC, so that the animation velocities match the calculated values.
animationTime = TimeStepCalculator(robot);
timeStep = 0.03;
animationStep = round(animationTime/timeStep,0,'decimals');

% Slowly allgin end effector so that the blast stream is parallel to the gravity
% vector before grit blasting
moveDistance = -0.005;
startPose(1) = startPose(1)+moveDistance;
allignmentVelocity = abs(moveDistance/30);
allignmentTimeStep=0.3;
plotResults = false;
allignmentOverload = false;
[q,qd,qdd,allignmentVelocity,robotOverload] = DynamicTorque(robot,startPose,allignmentVelocity,axis,allignmentTimeStep,launching,allignmentOverload,plotResults);

if robotOverload == false
    gritBlast = false;
    AnimateTrajectory(robot,q,gritBlast,gritBlastHeight,animationStep);
end

% Move robot along straight grit blasting trajectory
plotResults = true;
[q,qd,qdd,velocity,robotOverload] = DynamicTorque(robot,endPoint,velocity,axis,timeStep,launching,overload,plotResults);

% figure(1); % Switch back to figure 1 tab

pause(1); % Give the computer/simulation time to catch up so that the animation time is accurate

if robotOverload == false
    % Start timer
    tic;
    
    gritBlast = true;
    AnimateTrajectory(robot,q,gritBlast,gritBlastHeight,animationStep);
    
    % Stop timer and return time passed since tic was called
    timePassed = toc;
    approxAnimationVelocityError = norm(endPoint-startPose)/timePassed;
    
    disp(['Time passed: ',num2str(timePassed),' seconds']);
    disp(['Approximated animation velocity: ',num2str(approxAnimationVelocityError),' m/s']);
    disp(['Approximated animation velocity error vs programmed velocity: ',num2str(approxAnimationVelocityError-velocity),' m/s']);
end

% Delete previous trajectory plot
% delete(trajectoryPlot);
% trajectoryPlot = [];

function [timeStep] = TimeStepCalculator (robot)

trajectory = robot.getpos;

% Start timer
tic;

for trajStep = 1:size(trajectory,1)
    Q = trajectory(trajStep,:);
    
    % Animate robot through a fraction of the total movement
    robot.animate(Q);
    
    drawnow();
end

% Stop timer and return time passed since tic was called
timeStep = toc;

end

function AnimateTrajectory (robot,trajectory,gritBlast,gritBlastHeight,animationStep)

blastStreamPlot = [];
figure(1);

% Iterate the robot arms through their movement
for trajStep = 1:animationStep:size(trajectory,1)
    
    Q = trajectory(trajStep,:);
    
    % Turn on grit blaster (show grit blasting line indicator)
    if gritBlast == true
        %   calculate end effector position using fkine
        fkine = robot.fkine(robot.getpos());
        endEffectorPosition = fkine(1:3,4);
        
        for i=0.01:0.01:gritBlastHeight
            index = round(i * 100,0,'decimals');
            blastStream(index,:) = [endEffectorPosition(1),endEffectorPosition(2),endEffectorPosition(3)-i];
        end
        
        blastStreamPlot = plot3(blastStream(:,1),blastStream(:,2),blastStream(:,3),'Color', [0.5, 0.5, 0.5], 'Marker', '.','LineWidth',1);
        
    end
    
    % Animate robot through a fraction of the total movement
    robot.animate(Q);
    
    drawnow();
    
    % Delete previous trajectory plot
    delete(blastStreamPlot);
    blastStreamPlot = [];
end

end

function [endQ] = CalculateQ(robot,point)

% End tranformation for the end effector
endTransform = transl(point)*troty(pi);

% endTransform = transl(point)*rotationTransformY*rotationTransformX;

% Use endTransform to find the joint orientations for the end position
endQ = robot.ikcon(endTransform,robot.getpos);

endTransformCheck = robot.fkine(endQ);

% Difference (absolute positive magnitude) between the specified end Transform and actual
% end Tansform calculated by ikcon (inverse kinematics)
transformError = abs(endTransformCheck) - abs(endTransform);

% Position error
translationError = transformError(1:3,4)';

% Rotation error
rotationError = rad2deg(tr2rpy(transformError(1:3,1:3)));

% Display errors
disp('Transformation Error: ')
disp(transformError);

disp('Translation Error: ')
disp(translationError);

disp('Rotation Error (degrees): ')
disp(rotationError);

end

function [qMatrix,trajectoryPlot]=ResolveMotionRateControlCalculateTrajectory(robot,endPoint,velocity,axis,steps,timeStep,launching)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modified from 41013 Robotics week 9 material
% "Lab9Solution_Question1.m"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculate distance to point to scale the number of steps for
% each trajectory

% % Transform = robotTransform*toolTransform;
robotTransform = robot.fkine(robot.getpos);
startPoint = robotTransform(1:3,4)';
distanceToEndPoint = norm(endPoint-startPoint);

t = 10;             % Total time (s)
deltaT = timeStep;      % Control frequency
% steps = t/deltaT;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector

% 1.2) Allocate array data
m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,6);       % Array for joint anglesR
qdot = zeros(steps,6);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error

% 1.3) Set up trajectory, initial pose
s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = (1-s(i))*startPoint(1) + s(i)*endPoint(1); % Points in x
    x(2,i) = (1-s(i))*startPoint(2) + s(i)*endPoint(2); % Points in y
    if launching == true
        x(3,i) = startPoint(3) + sqrt((hypot(startPoint(2),startPoint(1)))^2-((hypot(x(1,i),x(2,i))))^2);% Points in z
    elseif launching == false
        x(3,i) = (1-s(i))*startPoint(3) + s(i)*endPoint(3); % Points in z
        %     x(3,i) = endPoint(3) + 0.2*sin(i*delta); % Points in z
    end
    
    theta(1,i) = axis(2);                                        % Roll angle % pi/2 alligns y to trajectory
    theta(2,i) = axis(1);                                        % Pitch angle   % pi/2 alligns x to trajectory
    theta(3,i) = axis(3);                                        % Yaw angle % pi/2 alligns z to trajectory
    
end

q0 = zeros(1,6);                                                            % Initial guess for joint angles

% robotTransform * trotx(deg2rad(axis(2)))*troty(deg2rad(axis(1)))*trotz(deg2rad(axis(3)));
qMatrix(1,:) = robot.ikcon(robotTransform,robot.getpos);                                            % Solve joint angles to achieve first waypoint

% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    T = robot.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(abs(Rd*Ra'));                                            % Convert rotation matrix to RPY angles. Gives error as rotation matrix times its transform should equal the indentity matrix
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = robot.jacob0(qMatrix(i,:));                                          % Get Jacobian at current joint state. Jacob gives with respect to base!!!
    m(i) = sqrt(abs(det(J*J')));
    if m(i) < epsilon                                                       % If manipulability is less than given threshold (epsilon was set at top)
        lambda = (1 - m(i)/epsilon)*5E-2; % If manipubility is insufficient then damping will be < 0
    else
        lambda = 0; % If manipubility is sufficient then damping will be = 0
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the         vector)
    for j = 1:6                                                             % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.qlim(j,1)                     % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
    angleError(:,i) = deltaTheta;                                           % For plotting
end

Transfrom = robot.fkine(qMatrix(end,:));
endPosition = Transfrom(1:3,4)';

error = endPosition-endPoint;
errorMax = max(abs(error));

trajectoryPlot = plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1);
% Display errors
disp('End Point Translation Error: ')
disp(error);

disp('End Point Maximum Translation Error: ')
disp(errorMax);

disp('End Point Rotation Error (degrees): ')
disp(rad2deg(angleError(:,end))');

end

function [objectMesh_h] = LoadObject(objectName, position, orientation)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modified from 41013 Robotics week 4 material
% "PuttingSimulatedObjectsIntoTheEnvironment.m"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Loads object into environment in specified position and orientation

[f,v,data] = plyread(objectName,'tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
objectMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

% Get vertex count
objectVertexCount = size(v,1);

% Move center point to origin
midPoint = sum(v)/objectVertexCount;
objectVerts = v - repmat(midPoint,objectVertexCount,1);

innacuracyOffset = 0.006518;

position = [position(1)+midPoint(1)*cos(orientation),position(2)+midPoint(1)*sin(orientation),position(3)+midPoint(3)];

% Move forwards (facing in -y direction)
forwardTR = makehgtform('translate',position-innacuracyOffset);

% % Random rotate about Z
rotateTR = makehgtform('zrotate',orientation);

objectPose = eye(4);

% Move the pose forward and a slight and random rotation
objectPose = objectPose * forwardTR *rotateTR;
updatedPoints = [objectPose * [objectVerts,ones(objectVertexCount,1)]']';

% Now update the Vertices
objectMesh_h.Vertices = updatedPoints(:,1:3);
drawnow();

end

function [q,qd,qdd,velocity,robotOverload] = DynamicTorque(robot,endPoint,velocity,axis,timeStep,launching,overload,plotResults)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modified from 41013 Robotics week 10 material
% "Lab9Solution_Question3.m "
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

qZero = zeros(1,6);                                                         % Initial joint angle guess for ikcon
tau_max = [97.7 186.4 89.4 24.2 20.1 21.3]';                                % Maximum joint torque of the Puma560
qd_max = [8 10 10 5 5 5];
qdd_max = [10 12 12 8 8 8];

%%%%%%%%%% Variables to change %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robotTransform = robot.fkine(robot.getpos);
startPoint = robotTransform(1:3,4)';
distanceToEndPoint = norm(endPoint-startPoint);

% overloadPoint = zeros(1,2);

torqueLimit = false;
velocityLimit = false;
accelerationLimit = false;
breakOut = false;
time = distanceToEndPoint/velocity;

while torqueLimit == false && velocityLimit == false && accelerationLimit == false && breakOut == false
    
    overloadPoint = zeros(1,2);
    
    if overload == false
        breakOut = true;
    elseif overload == true
        time= time -timeStep;
        velocity = distanceToEndPoint/time;
    end
    
    % time = 0.2;                                                                  % Total time to execute the motion
    T1 = [[0 -1 0; 0 0 1; -1 0 0] [0;0.7;0];zeros(1,3) 1];                      % First pose
    q1 = robot.ikcon(T1,qZero);                                                     % Inverse kinematics for 1st pose
    T2 = [[0 0 1;0 -1 0; 1 0 0] [0.5;0;0.6];zeros(1,3) 1];                      % Second pose
    q2 = robot.ikcon(T2,qZero);                                                     % Inverse kinematics for 2nd pose
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    dt = timeStep;                                                                 % Set control frequency at 100Hz
    steps = round(time/dt,0,'decimals');                                           % No. of steps along trajectory
    
    % s = lspb(0,1,steps);                                                      % Generate trapezoidal velocity profile
    % for i = 1:steps
    %     q(i,:) = (1-s(i))*q1 + s(i)*q2;
    % end
    % q = jtraj(q1,q2,steps);                                                     % Quintic polynomial profile
    [q,trajectoryPlot] = ResolveMotionRateControlCalculateTrajectory(robot,endPoint,velocity,axis,steps,timeStep,launching);
    
    qd = zeros(steps,6);                                                        % Array of joint velocities
    qdd = nan(steps,6);                                                         % Array of joint accelerations
    tau = nan(steps,6);                                                         % Array of joint torques
    mass = -19.2148;                                                           % Payload mass (kg)
    robot.payload(mass,[0;0;0]);                                               % Set payload mass in Puma 560 model: no offset
    
    
    % Execute equations from week 9 lectures on dynamics
    for i = 1:steps-1
        
        % Discrete time propagation
        % q(t) = q(t-1) + dt*qd(t-1) + (dt^2)*qdd(t-1)
        qdd(i,:) = (1/dt)^2 * (q(i+1,:) - q(i,:) - dt*qd(i,:));                 % Calculate joint acceleration to get to next set of joint angles
        M = robot.inertia(q(i,:));                                               % Calculate inertia matrix at this pose
        % Coriolis (C) is either a matrix*vector = vector or just a straight up vector
        C = robot.coriolis(q(i,:),qd(i,:));                                      % Calculate coriolis matrix at this pose
        g = robot.gravload(q(i,:));                                              % Calculate gravity vector at this pose
        % tau = M*qdd + C*qd + g
        tau(i,:) = (M*qdd(i,:)' + C*qd(i,:)' + g')';                            % Calculate the joint torque needed
        for j = 1:6
            if abs(tau(i,j)) > tau_max(j)                                     % Check if torque exceeds limits
                %                 tau(i,j) = sign(tau(i,j))*tau_max(j);       % Cap joint torque if above limits
                if torqueLimit == false
                    overloadPoint = [i,j];
                end
                torqueLimit = true;
            end
        end
        for j = 1:6
            if abs(qd(i,j)) > qd_max(j)                                       % Check if velocity exceeds limits
                %                 qd(i,j) = sign(qd(i,j))*qd_max(j);                           % Cap velocity if above limits
                if velocityLimit == false
                    overloadPoint = [i,j];
                end
                velocityLimit = true;
                
            end
        end
        for j = 1:6
            if abs(qdd(i,j)) > qdd_max(j)                                       % Check if acceleration exceeds limits
                %                 qdd(i,j) = sign(qdd(i,j))*qdd_max(j);                           % Cap acceleration if above limits
                if accelerationLimit == false
                    overloadPoint = [i,j];
                end
                accelerationLimit = true;
                
            end
        end
        
        % Make sure these calculation are below the above for loops.
        % So that they can recaculate after any values are normalised
        if accelerationLimit == false
            qdd(i,:) = (inv(M)*(tau(i,:)' - C*qd(i,:)' - g'))';                     % Re-calculate acceleration based on actual torque
        end
        if torqueLimit == false
            q(i+1,:) = q(i,:) + dt*qd(i,:) + dt^2*qdd(i,:);                         % Update joint angles based on actual acceleration
        end
        if velocityLimit == false
            qd(i+1,:) = qd(i,:) + dt*qdd(i,:);                                      % Update the velocity for the next pose
            %
        end
        
        
    end
    
    t = 0:dt:(steps-1)*dt;                                                      % Generate time vector
    
end
%% Visulalisation and plotting of results

robotOverload = false;

if overload == true || accelerationLimit == true || velocityLimit == true || torqueLimit == true
    
    disp(['Overload velocity: ',num2str(velocity),' m/s']);
    disp('ROBOT OVERLOADED: INSTRUCTION NOT POSSIBLE. REDUCE MOVEMENT VELOCITY');
    
    if velocityLimit==true
        disp(['Velocity overload at ',num2str(t(overloadPoint(1))),' seconds, in joint ', num2str(overloadPoint(2)),', at ',num2str(qdd(overloadPoint(1),overloadPoint(2))), ' rad/s']);
    elseif accelerationLimit==true
        disp(['Acceleration overload at ',num2str(t(overloadPoint(1))),' seconds, in joint ', num2str(overloadPoint(2)),', at ',num2str(qdd(overloadPoint(1),overloadPoint(2))), ' rad/s/s']);
    elseif torqueLimit==true
        disp(['Joint Torque overload at ',num2str(t(overloadPoint(1))),' seconds, in joint ', num2str(overloadPoint(2)),', at ',num2str(qdd(overloadPoint(1),overloadPoint(2))), ' Nm']);
    end
    
    disp(['Overload joint configuration at [',num2str(rad2deg(q(overloadPoint(1),:))),'] degrees']);
    
    endTransform = robot.fkine(robot.getpos);
    endPosition = endTransform(1:3,4)';
    disp(['Overload end Effector Position [',num2str(endPosition),'] meters']);
    
    robotOverload = true;
    
end

if plotResults == true
    
    % Plot joint angles
    figure('Name','Joint Angles','NumberTitle','off')
    for j = 1:6
        subplot(3,2,j)
        plot(t,q(:,j)','k','LineWidth',1);
        refline(0,robot.qlim(j,1));
        refline(0,robot.qlim(j,2));
        title(['Joint ', num2str(j)]);
        ylabel('Joint Angle (rad)');
        xlabel('time (s)');
        box off
    end
    
    % Plot joint velocities
    figure('Name','Joint Velocities','NumberTitle','off')
    for j = 1:6
        subplot(3,2,j)
        plot(t,qd(:,j),'k','LineWidth',1);
        refline(0,qd_max(j));
        refline(0,-qd_max(j));
        refline(0,0);
        title(['Joint ', num2str(j)]);
        ylabel('Velocity (rad/s)');
        xlabel('time (s)');
        if velocityLimit==true && j == overloadPoint(2)
            hold on
            plot(t(overloadPoint(1)),qdd(overloadPoint(1),overloadPoint(2)),'r.','MarkerSize',20);
            hold off
        end
        box off
        
    end
    % Plot joint acceleration
    figure('Name','Joint Accelerations','NumberTitle','off')
    for j = 1:6
        subplot(3,2,j)
        plot(t,qdd(:,j),'k','LineWidth',1);
        refline(0,qdd_max(j));
        refline(0,-qdd_max(j));
        title(['Joint ', num2str(j)]);
        ylabel('Acceleration (rad/s/s)');
        xlabel('time (s)');
        refline(0,0);
        if accelerationLimit==true && j == overloadPoint(2)
            hold on
            plot(t(overloadPoint(1)),qdd(overloadPoint(1),overloadPoint(2)),'r.','MarkerSize',20);
            hold off
        end
        box off
    end
    
    % Plot joint torques
    figure('Name','Joint Torques','NumberTitle','off')
    for j = 1:6
        subplot(3,2,j)
        plot(t,tau(:,j),'k','LineWidth',1);
        refline(0,tau_max(j));
        refline(0,-tau_max(j));
        title(['Joint ', num2str(j)]);
        ylabel('Joint Torque (Nm)');
        xlabel('time (s)');
        if torqueLimit==true && j == overloadPoint(2)
            hold on
            plot(t(overloadPoint(1)),qdd(overloadPoint(1),overloadPoint(2)),'r.','MarkerSize',20);
            hold off
        end
        box off
    end
    
    % figure(6)
    % robot.plot(q,'fps',steps)
    
end
end