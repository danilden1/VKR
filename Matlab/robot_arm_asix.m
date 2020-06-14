
close all
%
eeName = 'Gripper';
numJoints = numel(robot.homeConfiguration);
ikInitGuess = robot.homeConfiguration;

point = [0.0 0.6 -0.2];
waypoints = point' + ... 
            [0 0 0 ; 0 0.1 0.1 ; 0.1 0.1 0 ; 0.1 0 -0.1 ; 0 0 0]';
         
% Euler Angles (Z Y X) relative to the home orientation       
orientations = [0     0    0;
                0  0    0; 
                0    0  0;
               0  0    0;
                0     0    0]';   
            
% Array of waypoint times
waypointTimes = 0:4:16;

% Trajectory sample time
ts = 0.2;
trajTimes = 0:ts:waypointTimes(end);
% Boundary conditions (for polynomial trajectories)
% Velocity (cubic and quintic)
waypointVels = 0.1 *[ 0  0  0 0 0 0;
                      0  0  0 0 0 0;
                      0  0  0 0 0 0;
                      0  0  0 0 0 0;
                      0  0  0 0 0 0]';

% Acceleration (quintic only)
waypointAccels = zeros(size(waypointVels));
% Acceleration times (trapezoidal only)
waypointAccelTimes = diff(waypointTimes)/4;


%%createWaypointData;

figure, hold on
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ko:','LineWidth',2);
title('Trajectory Waypoints'); 
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on
view([45 45]);
% Define IK solver

ik = robotics.InverseKinematics('RigidBodyTree',robot);
ikWeights = [1 1 1 1 1 1];
% Use a small sample time for this example, so the difference between joint
% and task space is clear due to evaluation of IK in task space trajectory.

ts = 0.02;
trajTimes = 0:ts:waypointTimes(end);
% Initialize matrices for plots
qTask = zeros(numJoints, numel(trajTimes)); % Derived joint angles in task space trajectory
posJoint = zeros(3,numel(trajTimes)); % Derived end effector positions in joint space trajectory

% Create and evaluate a task space trajectory
ikInitGuess = jointAnglesH;
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

disp('Running task space trajectory generation and evaluation...')
tic;

% Trajectory generation
[posTask,velTask,accelTask] = trapveltraj(waypoints,numel(trajTimes), ...
    'AccelTime',repmat(waypointAccelTimes,[3 1]), ...
    'EndTime',repmat(diff(waypointTimes),[3 1]));

% Trajectory evaluation

for idx = 1:numel(trajTimes) 
    % Solve IK
    tgtPose = trvec2tform(posTask(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;
    qTask(:,idx) = config;
end

taskTime = toc;
disp(['Task space trajectory time : ' num2str(taskTime) ' s']);

% Create and evaluate a joint space trajectory
ikInitGuess = jointAnglesH;
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

disp('Running joint space trajectory generation and evaluation...')
tic;

% Solve IK for all waypoints
numWaypoints = size(waypoints,2);
numJoints = numel(robot.homeConfiguration);
jointWaypoints = zeros(numJoints,numWaypoints);
for idx = 1:numWaypoints
    tgtPose = trvec2tform(waypoints(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    cfgDiff = config - ikInitGuess;
    jointWaypoints(:,idx) = config';    
end



[qJoint,qdJoint,qddJoint] = trapveltraj(jointWaypoints,numel(trajTimes), ...
    'AccelTime',repmat(waypointAccelTimes,[numJoints 1]), ... 
    'EndTime',repmat(diff(waypointTimes),[numJoints 1]));

[qJointc,qdJointc,qddJointc] =  cubicpolytraj(jointWaypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels);
        

[qJointq,qdJointq,qddJointq] = quinticpolytraj(jointWaypoints,waypointTimes,trajTimes, ... 
             'VelocityBoundaryCondition',waypointVels, ...
             'AccelerationBoundaryCondition',waypointAccels);

%Trajectory evaluation (only needed to find end effector position)
for idx = 1:numel(trajTimes)  
    eeTform = getTransform(robot,qJoint(:,idx)',eeName); 
    posJoint(:,idx) = tform2trvec(eeTform)'; 
end
posJointc = posJoint;
% for idx = 1:numel(trajTimes)  
%     eeTformc = getTransform(robot,qJointc(:,idx)',eeName); 
%     posJointc(:,idx) = tform2trvec(eeTformc)'; 
% end
% posJointq = posJoint;
% for idx = 1:numel(trajTimes)  
%     eeTformq = getTransform(robot,qJointq(:,idx)',eeName); 
%     posJointq(:,idx) = tform2trvec(eeTformq)'; 
% end


jointTime = toc;
disp(['Joint space trajectory time : ' num2str(jointTime) ' s']);
%%

% Compare trajectories in Cartesian space
close all
figure, hold on
plot3(posTask(1,:),posTask(2,:),posTask(3,:),'g-');
plot3(posJoint(1,:),posJoint(2,:),posJoint(3,:),'r--');
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ko','LineWidth',2);
title('Сравнение траекотрий'); 
xlabel('X [м]');
ylabel('Y [м]');
zlabel('Z [м]');
legend('Дек. П-во','П-во обобщ. коорд', 'Точки');
grid on
view([45 45]);
%%
% Compare joint angles
% Plot each joint trajectory
for idx = 1:numJoints - 1
    figure, hold on;
    plot(trajTimes,qTask(idx,:),'g-');
    plot(trajTimes,qJoint(idx,:),'r--');
    plot(trajTimes,qJointc(idx,:),'b:');
    plot(trajTimes,qJointq(idx,:),'m-.');
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Траектория ' num2str(idx) 'ого звена']); 
    xlabel('Время [с]');
    ylabel('Угол звена [рад]');
    legend('Дек. п-во ','Трепеции', 'Пол. 3й степени', 'Пол. 5й степени');
end
%%
%%qdTask = diff(qTask);
for idx = 1:numJoints - 1
    qdTask(1,idx) = 0;
end

for idx = 2:numel(trajTimes) 
    for j = 1:numJoints - 1
        qdTask(j,idx) =  (qTask(j,idx) - qTask(j,idx - 1)) * 10;
    end
end

for idx = 1:numJoints - 1
    figure, hold on;
    plot(trajTimes,qdJoint(idx,:),'r--');
    plot(trajTimes,qdJointc(idx,:),'b:');
    plot(trajTimes,qdJointq(idx,:),'m-.');
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Скорость ' num2str(idx) 'ого звена']); 
    xlabel('Время [с]');
    ylabel('Скорость звена [рад/с]');
    legend('Трепеции', 'Пол. 3й степени', 'Пол. 5й степени');
end

%%
for idx = 1:numJoints - 1
    figure, hold on;
    plot(trajTimes,qddJoint(idx,:),'r-');
    plot(trajTimes,qddJointc(idx,:),'b:');
    plot(trajTimes,qddJointq(idx,:),'m-.');
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Ускорение ' num2str(idx) 'ого звена']); 
    xlabel('Время [с]');
    ylabel('Ускорение звена [рад/с2]');
    legend('Трепеции', 'Пол. 3й степени', 'Пол. 5й степени');
end









