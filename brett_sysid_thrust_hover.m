clear;
close all
clc;
path(path,'../read_bags');
path(path,'../helper_functions');

bag_name = 'thrust_hover_2018-06-07-12-49-41.bag';

bag = ros.Bag(bag_name);


%% read topics
attitude = readAttitudeTarget(bag, '/mavros/setpoint_raw/target_attitude');
thrust_cmd = attitude.thrust; %thrust in [N]
thrust_avg = mean(thrust_cmd);

fprintf('The average thrust percent = %.3f \n',thrust_avg); 
