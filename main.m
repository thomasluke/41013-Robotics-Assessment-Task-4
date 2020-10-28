clf;
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
%  So, for student number: 10052821,  p560.base = transl( [1.005, 2.821, 1]);

mdl_puma560;
robot = p560;

% Student number = 12876785
baseLocation = [1.287,6.785,1];
robot.base = robot.base * transl(baseLocation);

axis([-2+baseLocation(1) 2+baseLocation(1) -2+baseLocation(2) 2+baseLocation(2) -1.5 2.5]);
robot.teach();
startQ = [0,0,0,0,0,0];
robot.animate(startQ);


%% Show calculations to determine the tool offset as a homogeneous transform matrix to attach the blasting nozzle (shown in the figure) to the robot (4)

blastStreamAngle = deg2rad(45);
toolOffset = [0,0,(200*tan(blastStreamAngle))/1000]; % Tool offset in meters
toolOffset = [0,0,0.2]; % Tool offset in meters

toolTransform = transl(toolOffset);
robot.tool=toolTransform;

robotTransform = robot.fkine(robot.getpos);
Transform = robotTransform*toolTransform;

plot3(Transform(1,4),Transform(2,4),Transform(3,4),'.','Color','b','MarkerSize',10);
% Transform = robotTransform +;

%% Choose a location for the drum based upon your personalised robot location

drumPosition = baseLocation;
drumPosition(1) = drumPosition(1)-0.75;
drumPosition(3) = drumPosition(3)-1.5;

drum = LoadObject("Drum.ply",drumPosition,0);

%% In your report, show (a) the robot base transform, (b) the drum transform and (c) the transform between the robot base and the drum

disp("Robot Base Transform");
robotBaseTransform = robot.base

disp("Robot Drum Transform");
drumTransform = transl(drumPosition)

disp("Transform Between Base and Drum");
robotDrumTransform = robotBaseTransform * drumTransform

%% Show how you used an inverse kinematic solver to find a starting pose that points the nozzle at one corner of the white window on the drum (2).
%  Remember you can use “Tools->Data Tips” in the Matlab figure window to find the [x,y,z] of the corners of the white window.

% Window corner point from data tips = [1.983,6.84,0.591]

drumOriginToCorner1=[0.3856,0.0550,0.5910];
windowCorner1 = drumPosition+drumOriginToCorner1;

drumOriginToCorner2=[0.1964,0.0550,0.5910];
windowCorner2 = drumPosition+drumOriginToCorner2

robot.animate(deg2rad([0,170,-35,0,0,0])); %initial guess for ikcon

% windowCorner = [0.8,6.84,0.591];
startPose = windowCorner1;
startPose(3) = startPose(3)+0.3;
% startPose(3) = startPose(3);
robotPose = CalculateQ(robot,startPose);
robot.animate(robotPose);

robotStartTransform = robot.fkine(robot.getpos);
% startPoint = robotStartTransform(1:3,4)';
endPoint = windowCorner2;
endPoint(3) = endPoint(3)+0.3;
% endPoint(2) = endPoint(2) - 0.3;
velocity = 1;

% Control allignment of end effector along trajectory
rpy=tr2rpy(robot.fkine(robot.getpos));
rpy(3)=rpy(3)+pi;
axis = -rpy; % Move along x axis

launching = false;

[qMatrix,trajectoryPlot] = ResolveMotionRateControlCalculateTrajectory(robot,endPoint,velocity,axis,launching);

AnimateTrajectory(robot,qMatrix);

% Delete previous trajectory plot
delete(trajectoryPlot);
trajectoryPlot = [];

function AnimateTrajectory (robot,trajectory)

% Iterate the robot arms through their movement
for trajStep = 1:size(trajectory,1)
    
    Q = trajectory(trajStep,:);
    
    % calculate end effector position using fkine
    %     fkine = robot.fkine(robot.getpos());
    %     endEffectorPosition = fkine(1:3,4);
    
    %                 plot3(endEffectorPosition(1),endEffectorPosition(2),endEffectorPosition(3),'k.','LineWidth',1);
    
    % Animate robot through a fraction of the total movement
    robot.animate(Q);
    
    drawnow();
end
end

function [endQ] = CalculateQ(robot,point)

% % Calculating rotation of the end transform depending on object position
% if sign(point(2)) ~= 0
%     rotationTransformX = sign(point(1))*-trotx(pi/2);
% else
%     rotationTransformX = 1;
% end
%
% if sign(point(1)) ~= 0
%     rotationTransformY = sign(point(1))*troty(pi/2);
% else
%     rotationTransformY = 1;
% end

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

function [qMatrix,trajectoryPlot]=ResolveMotionRateControlCalculateTrajectory(robot,endPoint,velocity,axis,launching)

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

% Robotics
% Lab 9 - Question 1 - Resolved Motion Rate Control in 6DOF
% 1.1) Set parameters for the simulation
% mdl_puma560;        % Load robot model
% t = (distanceToEndPoint)/velocity;             % Total time (s)
% deltaT = 0.005;      % Control frequency
% steps = round(t/deltaT,0,'decimals');   % No. of steps for simulation
% delta = 2*pi/steps; % Small angle change
% epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
% W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector 1s for more weighting than 0.1 angular velocities

t = 10;             % Total time (s)
deltaT = 0.02;      % Control frequency
steps = t/deltaT;   % No. of steps for simulation
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

% % Modify the rotation of the end effector based on the object position
% if endPoint(1)>0.01
%     objectTransform = troty(-pi/2);
% elseif endPoint(1) <0.01
%     objectTransform = troty(pi/2);
% else
%     objectTransform = eye(4);
% end
%
% if endPoint(2)>0.01
%     objectTransform = objectTransform*trotx(-pi/2);
% elseif endPoint(2) <0.01
%     objectTransform = objectTransform*trotx(pi/2);
% end
%
% rpy = tr2rpy(objectTransform);

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
% Modified from 41013 Robotics week 4 material material
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