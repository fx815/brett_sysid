%% load bag files: one bag for estimating the TF, one for validating that TF
clear;
close all
clc;
path(path,'../read_bags');
path(path,'../helper_functions');

bag_name1 = '2018-07-06-13-30-22.bag';
bag_name2 = '2018-07-06-13-31-10.bag';

bag1 = ros.Bag(bag_name1);
bag2 = ros.Bag(bag_name2);

%% read topics
imu_data1 = readImu(bag1, '/mavros/imu/data');
imu_data2 = readImu(bag2, '/mavros/imu/data');

attitude_cmd1 = readAttitudeTarget(bag1, '/mavros/setpoint_raw/target_attitude');
attitude_cmd2 = readAttitudeTarget(bag2, '/mavros/setpoint_raw/target_attitude');

thrust_cmd1 = attitude_cmd1.thrust; %thrust in [N]
thrust_cmd2 = attitude_cmd2.thrust; %thrust in [N]

imu_data1_z_acc = imu_data1.a(3,:);
imu_data2_z_acc = imu_data2.a(3,:);

t_start1 = imu_data1.t(1);
t_start2 = imu_data2.t(1);

imu_data1.t = imu_data1.t - t_start1;
imu_data2.t = imu_data2.t - t_start2;

attitude_cmd1.t = attitude_cmd1.t - attitude_cmd1.t(1);
attitude_cmd2.t = attitude_cmd2.t - attitude_cmd2.t(1);

%% sysid begins

%interp thrust
thrust_cmd1_interp = interp1(attitude_cmd1.t, thrust_cmd1, imu_data1.t, 'spline');
thrust_cmd2_interp = interp1(attitude_cmd2.t, thrust_cmd2, imu_data2.t, 'spline');

attitude_cmd1.t = imu_data1.t;
attitude_cmd2.t = imu_data2.t;

%get rid of first and last x seconds (to remove ground and transient effects)
st = 0;
clip = 0;

imu_data1.t = imu_data1.t(imu_data1.t > st & imu_data1.t < imu_data1.t(end)-clip);
imu_data2.t = imu_data2.t(imu_data2.t > st & imu_data2.t < imu_data2.t(end)-clip);

imu_data1_z_acc = imu_data1_z_acc(imu_data1.t > st & imu_data1.t < imu_data1.t(end)-clip);
imu_data2_z_acc = imu_data2_z_acc(imu_data2.t > st & imu_data2.t < imu_data2.t(end)-clip);

attitude_cmd1.t = attitude_cmd1.t(attitude_cmd1.t > st & attitude_cmd1.t < attitude_cmd1.t(end)-clip);
attitude_cmd2.t = attitude_cmd2.t(attitude_cmd2.t > st & attitude_cmd2.t < attitude_cmd2.t(end)-clip);

thrust_cmd1_interp = thrust_cmd1_interp(attitude_cmd1.t > st & attitude_cmd1.t < attitude_cmd1.t(end)-clip);
thrust_cmd2_interp = thrust_cmd2_interp(attitude_cmd2.t > st & attitude_cmd2.t < attitude_cmd2.t(end)-clip);


%% ID of thrust system
m = 1.760; %mass [kg]
g = 9.81; %[m/s^2]

%thrust from linear acceleration 
thrust_out1 = m*imu_data1_z_acc;
thrust_out2 = m*imu_data2_z_acc;

% figure()
% scatter(thrust_cmd1_interp,thrust_out1);hold on;
% Fit1 = polyfit(thrust_cmd1_interp,thrust_out1,1);hold on;
% plot(thrust_cmd1_interp,polyval(Fit1,thrust_cmd1_interp),'LineWidth',2);
% title('data1');
% 
% figure()
% scatter(thrust_cmd2_interp,thrust_out2);
% Fit2 = polyfit(thrust_cmd2_interp,thrust_out2,1);hold on;
% plot(thrust_cmd2_interp,polyval(Fit2,thrust_cmd2_interp),'LineWidth',2);
% title('data2');

thrust_total = [thrust_cmd1_interp,thrust_cmd2_interp];
thrust_out_tot = [thrust_out1,thrust_out2];

figure()
scatter(thrust_total,thrust_out_tot);
Fit3 = polyfit(thrust_total,thrust_out_tot,1); hold on;
plot(thrust_total,polyval(Fit3,thrust_total),'LineWidth',2);
xlabel('thrust [%]');ylabel('thrust [N]');
title('combined data');

fprintf('The line of best fit is thrust[N] = %.3f * thrust[percent] + %.3f\n',Fit3(1),Fit3(2)); 
