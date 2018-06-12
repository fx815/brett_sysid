%% load bag files: one bag for estimating the TF, one for validating that TF
clear;
close all
clc;
path(path,'../read_bags');
path(path,'../helper_functions');

bag_name1 = 'yaw1_2018-05-21-15-48-48.bag';
bag_name2 = 'yaw2_2018-05-21-15-49-58.bag';

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

t_start1 = imu_data1.t(1);
t_start2 = imu_data2.t(1);

imu_data1.t = imu_data1.t - t_start1;
imu_data2.t = imu_data2.t - t_start2;

attitude_cmd1.t = attitude_cmd1.t - attitude_cmd1.t(1);
attitude_cmd2.t = attitude_cmd2.t - attitude_cmd2.t(1);

%% sysid begins
attitude_cmd1.y_interp = zeros(length(imu_data1.rpy));
attitude_cmd2.y_interp = zeros(length(imu_data2.rpy));

%interp yaw
attitude_cmd1.y_interp = interp1(attitude_cmd1.t, attitude_cmd1.rpy(3,:), imu_data1.t, 'spline');
attitude_cmd2.y_interp = interp1(attitude_cmd2.t, attitude_cmd2.rpy(3,:), imu_data2.t, 'spline');

attitude_cmd1.t = imu_data1.t;
attitude_cmd2.t = imu_data2.t;

%get rid of first and last x seconds (to remove ground and transient effects)
t0 = 10;
t1 = 10;

imu_data1.t_clip = imu_data1.t(imu_data1.t > t0 & imu_data1.t < imu_data1.t(end)-t1);
imu_data2.t_clip = imu_data2.t(imu_data2.t > t0 & imu_data2.t < imu_data2.t(end)-t1);

imu_data1.rpy = imu_data1.rpy(:, imu_data1.t > t0 & imu_data1.t < imu_data1.t(end) - t1);
imu_data2.rpy = imu_data2.rpy(:, imu_data2.t > t0 & imu_data2.t < imu_data2.t(end) - t1);

attitude_cmd1.t_clip = attitude_cmd1.t(attitude_cmd1.t > t0 & attitude_cmd1.t < attitude_cmd1.t(end) - t1);
attitude_cmd2.t_clip = attitude_cmd2.t(attitude_cmd2.t > t0 & attitude_cmd2.t < attitude_cmd2.t(end) - t1);

attitude_cmd1.y_interp = attitude_cmd1.y_interp(:, attitude_cmd1.t > t0 & attitude_cmd1.t < attitude_cmd1.t(end) - t1);
attitude_cmd2.y_interp = attitude_cmd2.y_interp(:, attitude_cmd2.t > t0 & attitude_cmd2.t < attitude_cmd2.t(end) - t1);


%% ID of yaw system
np = 2; %1 for 1st order system, 2 for 2nd order system

%experiment 1

Experiment1.u1 = attitude_cmd1.y_interp;
Experiment1.y1 = imu_data1.rpy(3,:);

dt1 = mean(diff(imu_data1.t_clip));

yaw_data1 = iddata(Experiment1.y1',Experiment1.u1',dt1, ...
    'ExperimentName', 'SysID_1', 'InputName','yaw_{cmd}', ...
    'OutputName','yaw', 'InputUnit','rad', 'OutputUnit','rad', ...
'TimeUnit','Second');
yaw_data1 = detrend(yaw_data1);

%experiment 2
Experiment2.u1 = attitude_cmd2.y_interp;
Experiment2.y1 = imu_data2.rpy(3,:);

dt2 = mean(diff(imu_data2.t_clip));

yaw_data2 = iddata(Experiment2.y1',Experiment2.u1',dt2,...
    'ExperimentName', 'SysID_2', 'InputName','yaw_{cmd}',...
    'OutputName','yaw', 'InputUnit','rad', 'OutputUnit','rad',...
'TimeUnit','Second');
yaw_data2 = detrend(yaw_data2);

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
yaw_estimated_tf1 = tfest(yaw_data1,np, nz);

[~, fit1, ~] = compare(yaw_data2, yaw_estimated_tf1);

%Generate model using Experiment2 and validate the model with Experiment1
yaw_estimated_tf2 = tfest(yaw_data2,np, nz);

[~, fit2, ~] = compare(yaw_data1, yaw_estimated_tf2);

if fit1>fit2
    %We pick the first Identification
    yaw_estimated_tf = yaw_estimated_tf1;
    disp('The yaw is estimated using experiment 1 and validated on data from experiment 2');
    figure;
    compare(yaw_data2, yaw_estimated_tf1);    
    disp(strcat('The yaw model fits the validation data with **',...
        num2str(fit1), '** %'));
else
    %We pick the second Identification
    yaw_estimated_tf = yaw_estimated_tf2;
    disp('The yaw model is estimated using experiment 2 and validated on data from experiment 1');
    figure;
    compare(yaw_data1, yaw_estimated_tf2);
    disp(strcat('The yaw model fits the validation data with **',...
        num2str(fit2), '** %'));
end

%%% Estimated Transfer Functions

disp('yawrate estimated transfer function is: ');
tf(yaw_estimated_tf)
yawrate_params=getpvec(yaw_estimated_tf);
if(np==1)
    yawrate_gain=yawrate_params(1)/yawrate_params(2);
    yawrate_tau=1/yawrate_params(2);
    fprintf('yawrate gain=%.3f, tau=%.3f\n',yawrate_gain,yawrate_tau);
elseif(np==2)
    yawrate_omega=sqrt(yawrate_params(3));
    yawrate_gain=yawrate_params(1)/yawrate_params(3);
    yawrate_damping=yawrate_params(2)/(2*yawrate_omega);
    fprintf('yawrate omega=%.3f, gain=%.3f damping=%.3f\n',yawrate_omega,yawrate_gain,yawrate_damping);
end


figure('Name','System analysis (yawrate)');
subplot(311);
bode(yaw_estimated_tf); grid;
title('yawrate bode plot');

subplot(312);
rlocusplot(yaw_estimated_tf); grid;
title('yawrate RootLucas plot');

subplot(313);
step(yaw_estimated_tf); grid;
title('yawrate step response plot');

%% plot

% *Plot attitude*
figure();
title('Experiment 1 and 2 Data');
subplot(2,1,1);
plot(imu_data1.t_clip, imu_data1.rpy(3,:)*180/pi, ...
    attitude_cmd1.t_clip, attitude_cmd1.y_interp*180/pi, ...
    'g--', 'linewidth', 2);

xlabel('time');
legend('y','y_{ref}');
ylabel('yaw [deg]');
title('yaw from IMU, exp 1');

subplot(2,1,2);
plot(imu_data2.t_clip, imu_data2.rpy(3,:)*180/pi, ...
    attitude_cmd2.t_clip, attitude_cmd2.y_interp*180/pi, ...
    'g--', 'linewidth', 2);

xlabel('time');
ylabel('yaw [deg]');
title('yaw from IMU, exp 2');

