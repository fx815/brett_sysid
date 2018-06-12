%% load bag files: one bag for estimating the TF, one for validating that TF
clear;
close all
clc;
path(path,'../read_bags');
path(path,'../helper_functions');

bag_name1 = 'rp1_2018-05-21-15-40-31.bag';
bag_name2 = 'rp2_2018-05-21-15-45-20.bag';

bag1 = ros.Bag(bag_name1);
bag2 = ros.Bag(bag_name2);

%% read topics
imu_data1 = readImu(bag1, '/mavros/imu/data');
imu_data2 = readImu(bag2, '/mavros/imu/data');

attitude_cmd1 = readAttitudeTarget(bag1, '/mavros/setpoint_raw/target_attitude');
attitude_cmd2 = readAttitudeTarget(bag2, '/mavros/setpoint_raw/target_attitude');

attitude_cmd1.rpy = quat2rpy([attitude_cmd1.q(4,:)', attitude_cmd1.q(1:3,:)']');
attitude_cmd2.rpy = quat2rpy([attitude_cmd2.q(4,:)', attitude_cmd2.q(1:3,:)']');

imu_data1.rpy = quat2rpy([imu_data1.q(4,:)', imu_data1.q(1:3,:)']');
imu_data2.rpy = quat2rpy([imu_data2.q(4,:)', imu_data2.q(1:3,:)']');

attitude_cmd1.rpy = quat2rpy([attitude_cmd1.q(4,:)', attitude_cmd1.q(1:3,:)']');
attitude_cmd2.rpy = quat2rpy([attitude_cmd2.q(4,:)', attitude_cmd2.q(1:3,:)']');

t_start1 = imu_data1.t(1);
t_start2 = imu_data2.t(1);

imu_data1.t = imu_data1.t - t_start1;
imu_data2.t = imu_data2.t - t_start2;

attitude_cmd1.t = attitude_cmd1.t - attitude_cmd1.t(1);
attitude_cmd2.t = attitude_cmd2.t - attitude_cmd2.t(1);

%% sysid begins
attitude_cmd1.rpy_interp = zeros(size(imu_data1.rpy));
attitude_cmd2.rpy_interp = zeros(size(imu_data2.rpy));

%interp roll
attitude_cmd1.rpy_interp(1,:) = interp1(attitude_cmd1.t, attitude_cmd1.rpy(1,:), imu_data1.t, 'spline');
attitude_cmd2.rpy_interp(1,:) = interp1(attitude_cmd2.t, attitude_cmd2.rpy(1,:), imu_data2.t, 'spline');

%interp pitch
attitude_cmd1.rpy_interp(2,:) = interp1(attitude_cmd1.t, attitude_cmd1.rpy(2,:), imu_data1.t, 'spline');
attitude_cmd2.rpy_interp(2,:) = interp1(attitude_cmd2.t, attitude_cmd2.rpy(2,:), imu_data2.t, 'spline');

attitude_cmd1.t = imu_data1.t;
attitude_cmd2.t = imu_data2.t;

%get rid of first and last x seconds (to remove ground and transient effects)
t0 = 10;
t1 = 10;

imu_data1.t_clip = imu_data1.t(imu_data1.t > t0 & imu_data1.t < imu_data1.t(end) - t1);
imu_data2.t_clip = imu_data2.t(imu_data2.t > t0 & imu_data2.t < imu_data2.t(end) - t1);

imu_data1.rpy = imu_data1.rpy(:, imu_data1.t > t0 & imu_data1.t < imu_data1.t(end) - t1);
imu_data2.rpy = imu_data2.rpy(:, imu_data2.t > t0 & imu_data2.t < imu_data2.t(end) - t1);

attitude_cmd1.t_clip = attitude_cmd1.t(attitude_cmd1.t > t0 & attitude_cmd1.t < attitude_cmd1.t(end) - t1);
attitude_cmd2.t_clip = attitude_cmd2.t(attitude_cmd2.t > t0 & attitude_cmd2.t < attitude_cmd2.t(end) - t1);

attitude_cmd1.rpy_interp = attitude_cmd1.rpy_interp(:, attitude_cmd1.t > t0 & attitude_cmd1.t < attitude_cmd1.t(end) - t1);
attitude_cmd2.rpy_interp = attitude_cmd2.rpy_interp(:, attitude_cmd2.t > t0 & attitude_cmd2.t < attitude_cmd2.t(end) - t1);


%% ID of roll system
delay=[]; NaN;
np = 2; %1 for 1st order system, 2 for 2nd order system

%experiment 1
Experiment1.u1 = attitude_cmd1.rpy_interp(1,:);
Experiment1.y1 = imu_data1.rpy(1,:);

dt1 = mean(diff(imu_data1.t_clip));

roll_data1 = iddata(Experiment1.y1',Experiment1.u1',dt1, ...
    'ExperimentName', 'SysID_1', 'InputName','roll_{cmd}', ...
    'OutputName','roll', 'InputUnit','rad', 'OutputUnit','rad', ...
'TimeUnit','Second');
roll_data1 = detrend(roll_data1);

%experiment 2
Experiment2.u1 = attitude_cmd2.rpy_interp(1,:);
Experiment2.y1 = imu_data2.rpy(1,:);

dt2 = mean(diff(imu_data2.t_clip));

roll_data2 = iddata(Experiment2.y1',Experiment2.u1',dt2,...
    'ExperimentName', 'SysID_2', 'InputName','roll_{cmd}',...
    'OutputName','roll', 'InputUnit','rad', 'OutputUnit','rad',...
'TimeUnit','Second');
roll_data2 = detrend(roll_data2);

% *At this point we have 3 options!*
% 
% # Estimate a model from both experiments - but cannot validate it on independent dataset
% # Estimate a model from Exp1 and validate it on data from Exp2
% # Estimate a model from Exp2 and validate it on data from Exp1
%For now we choose the best model from options 2 and 3


%Assume 1st  order system  
%np = 2;
nz = 0;

%Generate model using Experiment1 and validate the model with Experiment2
roll_estimated_tf1 = tfest(roll_data1,np, nz,delay);

[~, fit1, ~] = compare(roll_data2, roll_estimated_tf1);

%Generate model using Experiment2 and validate the model with Experiment1
roll_estimated_tf2 = tfest(roll_data2,np, nz,delay);

[~, fit2, ~] = compare(roll_data1, roll_estimated_tf2);

if fit1>fit2
    %We pick the first Identification
    roll_estimated_tf = roll_estimated_tf1;
    disp('The roll model is estimated using experiment 1 and validated on data from experiment 2');
    figure;
    compare(roll_data2, roll_estimated_tf1);    
    disp(strcat('The roll model fits the validation data with **',...
        num2str(fit1), '** %'));
else
    %We pick the second Identification
    roll_estimated_tf = roll_estimated_tf2;
    disp('The roll model is estimated using experiment 2 and validated on data from experiment 1');
    figure;
    compare(roll_data1, roll_estimated_tf2);
    disp(strcat('The roll model fits the validation data with **',...
        num2str(fit2), '** %'));
end

%% ID pitch of system

%experiment 1
Experiment1.u2 = attitude_cmd1.rpy_interp(2,:);
Experiment1.y2 = imu_data1.rpy(2,:);

pitch_data1 = iddata(Experiment1.y2',Experiment1.u2',dt1,...
    'ExperimentName', 'SysID_1', 'InputName','pitch_{cmd}',...
    'OutputName','pitch', 'InputUnit','rad', 'OutputUnit','rad',...
'TimeUnit','Second');

pitch_data1 = detrend(pitch_data1);

%experiment 2
Experiment2.u2 = attitude_cmd2.rpy_interp(2,:);
Experiment2.y2 = imu_data2.rpy(2,:);

pitch_data2 = iddata(Experiment2.y2',Experiment2.u2',dt2, ...
    'ExperimentName', 'SysID_2', 'InputName','pitch_{cmd}',...
    'OutputName','pitch', 'InputUnit','rad', 'OutputUnit','rad', ...
    'TimeUnit','Second');

pitch_data2 = detrend(pitch_data2); 

% *At this point we have 3 options!*
% 
% # Estimate a model from both experiments - but cannot validate it on independent dataset
% # Estimate a model from Exp1 and validate it on data from Exp2
% # Estimate a model from Exp2 and validate it on data from Exp1
%For now we choose the best model from options 2 and 3
  
%Assume 1st order system
%np = 2;
nz = 0;

%Generate model using Experiment1 and validate the model with Experiment2
pitch_estimated_tf1 = tfest(pitch_data1,np, nz,delay);

[~, fit1, ~] = compare(pitch_data2, pitch_estimated_tf1);

%Generate model using Experiment2 and validate the model with Experiment1
pitch_estimated_tf2 = tfest(pitch_data2,np, nz,delay);

[~, fit2, ~] = compare(pitch_data1, pitch_estimated_tf2);

if fit1>fit2
    %We pick the first Identification
    pitch_estimated_tf = pitch_estimated_tf1;
    disp('The pitch model is estimated using experiment 1 and validated on data from experiment 2');
    figure;
    compare(pitch_data2, pitch_estimated_tf1);
    disp(strcat('The pitch model fits the validation data with **', ...
        num2str(fit1), '** %'));
else
    %We pick the second Identification
    pitch_estimated_tf = pitch_estimated_tf2;
    disp('The pitch model is estimated using experiment 2 and validated on data from experiment 1');
    figure;
    compare(pitch_data1, pitch_estimated_tf2);
    disp(strcat('The pitch model fits the validation data with **', ...
        num2str(fit2), '** %'));
end


%% plot

% *Plot attitude from experiment 1*
figure();
title('Experiment 1 Data');
subplot(2,1,1);
plot(imu_data1.t_clip, imu_data1.rpy(1,:)*180/pi, ...
    attitude_cmd1.t_clip, attitude_cmd1.rpy_interp(1,:)*180/pi, ...
    'g--', 'linewidth', 2);

xlabel('time');
legend('y','y_{ref}');
ylabel('roll [deg]');
title('roll from IMU, exp 1');

subplot(2,1,2);
plot(imu_data1.t_clip, imu_data1.rpy(2,:)*180/pi, ...
    attitude_cmd1.t_clip, attitude_cmd1.rpy_interp(2,:)*180/pi, ...
    'g--', 'linewidth', 2);

xlabel('time');
ylabel('pitch [deg]');
title('pitch from IMU, exp 1');

% *Plot attitude from experiment 2*
figure();
title('Experiment 2 Data');
subplot(2,1,1);
plot(imu_data2.t_clip, imu_data2.rpy(1,:)*180/pi, ...
    attitude_cmd2.t_clip, attitude_cmd2.rpy_interp(1,:)*180/pi, ...
    'g--', 'linewidth', 2);

xlabel('time');
legend('y','y_{ref}');
ylabel('roll [deg]');
title('roll from IMU, exp 2');

subplot(2,1,2);
plot(imu_data2.t_clip, imu_data2.rpy(2,:)*180/pi, ...
    attitude_cmd2.t_clip, attitude_cmd2.rpy_interp(2,:)*180/pi, ...
    'g--', 'linewidth', 2);

xlabel('time');
ylabel('pitch [deg]');
title('pitch from IMU, exp 2');

%% Estimate the Whole System as 2-input 2-output MIMO System
% *The purpose here is to see of there is coupling*

dt2 = dt1;    
Data1 = iddata([Experiment1.y1', Experiment1.y2'], ...
    [Experiment1.u1', Experiment1.u2'], dt1, ...
    'ExperimentName', 'FireFlySysID_1', ...
    'InputName',{'roll_{cmd}','pitch_{cmd}'},...
    'OutputName',{'roll','pitch'}', ...
    'InputUnit',{'rad', 'rad'},...
    'OutputUnit',{'rad', 'rad'},...
    'TimeUnit','Second');

Data2 = iddata([Experiment2.y1', Experiment2.y2'], ...
    [Experiment2.u1', Experiment2.u2'], dt2, ...
    'ExperimentName', 'FireFlySysID_2', ...
    'InputName',{'roll_{cmd}','pitch_{cmd}'},...
    'OutputName',{'roll','pitch'}', ...
    'InputUnit',{'rad', 'rad'},...
    'OutputUnit',{'rad', 'rad'}, ...
    'TimeUnit','Second');

MergedData = merge(Data1, Data2);

%np = 2;
nz = 0;
Full_estimated_tf = tfest(MergedData, np,nz);

figure;
bodemag(Full_estimated_tf);

%%% Estimated Transfer Functions

disp('Roll estimated transfer function is: ');
tf(roll_estimated_tf)

if(np==1)
    roll_params=getpvec(roll_estimated_tf);
    roll_gain=roll_params(1)/roll_params(2);
    roll_tau=1/roll_params(2);
    fprintf('roll tau=%.3f, gain=%.3f\n',roll_tau,roll_gain);
elseif(np==2)
    roll_params=getpvec(roll_estimated_tf);
    roll_omega=sqrt(roll_params(3));
    roll_gain=roll_params(1)/roll_params(3);
    roll_damping=roll_params(2)/(2*roll_omega);
    fprintf('roll omega=%.3f, gain=%.3f damping=%.3f\n',roll_omega,roll_gain,roll_damping);
end

figure('Name','System analysis (roll)');
subplot(311);
bode(roll_estimated_tf); grid;
title('Roll bode plot');

subplot(312);
rlocusplot(roll_estimated_tf); grid;
title('Roll RootLucas plot');

subplot(313);
step(roll_estimated_tf); grid;
title('Roll step response plot');

disp('Pitch estimated transfer function is: ');
tf(pitch_estimated_tf)

if(np==1)
    pitch_params=getpvec(pitch_estimated_tf);
    pitch_gain=pitch_params(1)/pitch_params(2);
    pitch_tau=1/pitch_params(2);
    fprintf('pitch tau=%.3f, gain=%.3f\n',pitch_tau, pitch_gain);
elseif(np==2)
    pitch_params=getpvec(pitch_estimated_tf);
    pitch_omega=sqrt(pitch_params(3));
    pitch_gain=pitch_params(1)/pitch_params(3);
    pitch_damping=pitch_params(2)/(2*pitch_omega);
    fprintf('pitch omega=%.3f, gain=%.3f damping=%.3f\n',pitch_omega,pitch_gain,pitch_damping);
end

figure('Name','System analysis (pitch)');
subplot(311);
bode(pitch_estimated_tf); grid;
title('Pitch bode plot');

subplot(312);
rlocusplot(pitch_estimated_tf); grid;
title('Pitch RootLucas plot');

subplot(313);
step(pitch_estimated_tf); grid;
title('Pitch step response plot');
