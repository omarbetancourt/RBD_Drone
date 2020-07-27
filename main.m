clc;
clear;
close all;

addpath('./lib');
%% DEFINE
R2D = 180/pi; % radians to degrees
D2R = pi/180;

%% INITIALIZING PARAMETERS regarding the drone
drone1_params = containers.Map({'mass', 'armLength', 'Ixx', 'Iyy', 'Izz'}, ...
    {1.25, 0.265, 0.0232, 0.0232, 0.0468}); % Similar to dictionary in python

drone1_initStates = [0, 8, .5, ...      % starting X,Y,Z position
                     0, 0, 0, ...        % dX, dY, dZ (velocities)
                     0, 0, 0, ...        % phi, theta, psi (euler angles)
                     0, 0, 0]';          % p, q, r (angular rates)

drone1_initInputs = [0, 0, 0, 0]';      % u1, u2, u3, u4 (T, M1, M2, M3)
% T = total thrust and M's are moments on each control axis
drone1_body = [ 0.265,  0,      0, 1; ... 
                0,      -0.265, 0, 1; ...
                -0.265, 0,      0, 1; ...
                0,      0.265,  0, 1; ...
                0,      0,      0, 1; ... %Center of Drone
                0,      0,   0.15, 1]';
            
drone1_gains = containers.Map(... %% PID gains 
    {'P_phi',  'I_phi',   'D_phi', ... %From attitude controler
    'P_theta', 'I_theta', 'D_theta', ... %From attitude controler
    'P_psi',   'I_psi',   'D_psi', ... %From attitude controler
    'P_x',     'I_x',     'D_x',...
    'P_y',     'I_y',     'D_y',...
    'P_z',     'I_z',     'D_z'},...
    {6,       0.0,       4, ...
     6,       0.0,       4, ...
     6,       0.0,       7, ...        
     13,       0.0,       8,...
     13,       0.0,       8,...
     13,       0.0,       8});        

simulationTime = 30000; %milliseconds
drone1 = Drone(drone1_params, drone1_initStates, drone1_initInputs, drone1_gains, simulationTime);

%% Init. 3D Fig.
fig1 = figure('pos', [0 200 800 800]);
h = gca;            
view(3); 

axis equal;
grid on;

xlim([0 8]);
ylim([0 8]);
zlim([0 6]);
xlabel('X[m]');
ylabel('Y[m]');
zlabel('Z[m]');

hold(gca, 'on');
drone1_state = drone1.GetState();
wHb = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3);
       0 0 0 1];

drone1_world = wHb * drone1_body;
drone1_atti = drone1_world(1:3, :);

fig1_ARM13 = plot3(gca, drone1_atti(1,[1 3]), drone1_atti(2, [1 3]), drone1_atti(3, [1 3]), '-ro', 'MarkerSize', 5);
fig1_ARM24 = plot3(gca, drone1_atti(1,[2 4]), drone1_atti(2, [2 4]), drone1_atti(3, [2 4]), '-bo', 'MarkerSize', 5);
fig1_payload = plot3(gca, drone1_atti(1,[5 6]), drone1_atti(2, [5 6]), drone1_atti(3, [5 6]), '-k', 'MarkerSize', 5);
fig1_shadow = plot3(gca, 0, 0, 0, 'xk', 'LineWidth', 3);

hold(gca, 'off');

%% Init. Data Fig.
fig2 = figure('pos', [800 550 800 450]);
subplot(2,3,1);
title('phi[deg]');
grid on;
hold on;
subplot(2,3,2)
title('theta[deg]');
grid on;
hold on;
subplot(2,3,3);
title('psi[deg]');
grid on;
hold on;
subplot(2,3,4);
title('x[m]');
grid on;
hold on;
subplot(2,3,5);
title('y[m]');
grid on;
hold on;
subplot(2,3,6);
title('zdot[m/s]');
grid on;
hold on;

% Desired position at 0,0,0 
commandSig(1) = 4;
commandSig(2) = 3;
commandSig(3) = 6;

myVideo = VideoWriter('myVideoFile'); %open video file
myVideo.FrameRate = 30;  %can adjust this, 5 - 10 works well for me
open(myVideo)
for i = 1:simulationTime
    drone1.PositionCtrl(commandSig);
    drone1.AttitudeCtrl();
    
    drone1.UpdateState();
    drone1_state = drone1.GetState();
    
    %% 3D plot
    if mod(simulationTime,50) == 0
        figure(1)
        wHb = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3); 0 0 0 1];
        drone1_world = wHb * drone1_body;
        drone1_atti = drone1_world(1:3, :);

        set(fig1_ARM13, ...
            'xData', drone1_atti(1,[1 3]), ...
            'yData', drone1_atti(2,[1 3]), ...
            'zData', drone1_atti(3,[1 3]));
        set(fig1_ARM24, ...
            'xData', drone1_atti(1,[2 4]), ...
            'yData', drone1_atti(2,[2 4]), ...
            'zData', drone1_atti(3,[2 4]));
        set(fig1_payload, ...
            'xData', drone1_atti(1,[5 6]), ...
            'yData', drone1_atti(2,[5 6]), ...
            'zData', drone1_atti(3,[5 6]));
        set(fig1_shadow, ...
            'xData', drone1_state(1), ...
            'yData', drone1_state(2), ...
            'zData', 0);
    pause(0.01) %Pause and grab frame
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
    end

%     figure(2)
%     subplot(2,3,1);
%         plot(i/100, drone1_state(7)*R2D, '.');
%     subplot(2,3,2)
%         plot(i/100, drone1_state(8)*R2D, '.');
%     subplot(2,3,3);
%         plot(i/100, drone1_state(9)*R2D, '.');
%     subplot(2,3,4);
%         plot(i/100, drone1_state(1), '.');
%     subplot(2,3,5);
%         plot(i/100, drone1_state(2), '.');
%     subplot(2,3,6);
%         plot(i/100, drone1_state(3), '.');
    if (drone1_state(3) <= 0)
        msgbox('Crashed', 'Error', 'error');
        break;
    end

end
close(myVideo)




