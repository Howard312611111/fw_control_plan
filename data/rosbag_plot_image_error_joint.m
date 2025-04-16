clear 
clc

% read rosbag
bag = rosbag('track_error_2023-08-11-15-45-55.bag');
bag.AvailableTopics

bag_pos = select(bag, "Topic", "/uav0/tracking/image_error/position");
pos = readMessages(bag_pos, "DataFormat", "struct");
bag_yolo = select(bag, "Topic", "/uav0/tracking/image_error/yolo");
yolo = readMessages(bag_yolo, "DataFormat", "struct");
bag_joint = select(bag, "Topic", "/uav0/gimbal/joint_states");
joint = readMessages(bag_joint, "DataFormat", "struct");
bag_uav = select(bag, "Topic", "/uav0/base_pose_ground_truth");
uav = readMessages(bag_uav, "DataFormat", "struct");

dataNum1 = size(pos);
dataNum2 = size(yolo);
dataNum3 = size(joint);
dataNum4 = size(uav);

dataNum = dataNum1(1);
if dataNum2(1) < dataNum1(1)
    dataNum = dataNum2(1);
end

% time
Duration = bag.EndTime - bag.StartTime;
dt = Duration / dataNum;
t = 0:dt:Duration-dt;

dt_joint = Duration / dataNum3(1);
t_joint = 0:dt_joint:Duration-dt_joint;

dt_uav = Duration / dataNum4(1);
t_uav = 0:dt_uav:Duration-dt_uav;



pos_data = zeros(dataNum, 3);
yolo_data = zeros(dataNum, 3);
joint_data = zeros(dataNum3(1), 4);
uav_data = zeros(dataNum4(1), 5);

for i = 1:dataNum
    pos_data(i, 1) = pos{i}.X;
    pos_data(i, 2) = pos{i}.Y;
    pos_data(i, 3) = pos{i}.Z;

    yolo_data(i, 1) = yolo{i}.X;
    yolo_data(i, 2) = yolo{i}.Y;
    yolo_data(i, 3) = yolo{i}.Z;
end

rad2deg = 180/3.141596;

for i = 1:dataNum3
    joint_data(i, 1) = joint{i}.Position(2) * rad2deg;
    joint_data(i, 2) = joint{i}.Position(3) * rad2deg;
    joint_data(i, 3) = joint{i}.Velocity(2) * rad2deg;
    joint_data(i, 4) = joint{i}.Velocity(3) * rad2deg;
end

for i = 1:dataNum4
    uav_data(i, 1) = uav{i}.Pose.Pose.Position.X;
    uav_data(i, 2) = uav{i}.Pose.Pose.Position.Y;
    uav_data(i, 3) = uav{i}.Pose.Pose.Position.Z;
    uav_data(i, 4) = ( uav_data(i, 1)^2 + uav_data(i, 2)^2 )^0.5;
    uav_data(i, 5) = uav{i}.Twist.Twist.Angular.Z * rad2deg;
end

%% plot
% image error
figure(1)
subplot(4,1,1);
plot(t, pos_data(:, 1), '-', t, yolo_data(:, 1), '--', MarkerSize=3);
%axis([0, t(end), -10, 50]);
title('Error of (u-cu)');
xlabel('time (s)');
ylabel('u direction (pixel)');
lgd = legend('3D position', 'yolo');

subplot(4, 1, 2)
plot(t_joint, joint_data(:, 2), '-', MarkerSize=3);
%axis([0, t(end), -10, 50]);
title('Gimbal Joint Angle - Pan');
xlabel('time (s)');
ylabel('angle (degree)');

subplot(4, 1, 3)
plot(t_uav, uav_data(:, 4), '-', MarkerSize=3);
%axis([0, t(end), -10, 50]);
title('UAV X-Y Distance');
xlabel('time (s)');
ylabel('distance (m)');

subplot(4, 1, 4)
plot(t_joint, joint_data(:, 3), '-', MarkerSize=3);
%axis([0, t(end), -10, 50]);
title('Gimbal Angular Velocity');
xlabel('time (s)');
ylabel('tilt (deg/s)');

%plot(t_uav, uav_data(:, 5), '-', MarkerSize=3);
%axis([0, t(end), -10, 50]);
%title('UAV Yaw Angular Velocity');
%xlabel('time (s)');
%ylabel('rate (deg/s)');


% joint_errror
figure(2)
subplot(3,1,1);
plot(t, pos_data(:, 2), '-', t, yolo_data(:, 2), '--', MarkerSize=3);
%axis([0, t(end), -6, 14]);
title('Error of (v-cv)');
xlabel('time (s)');
ylabel('v direction (pixel)');
lgd = legend('3D position', 'yolo');

subplot(3, 1, 2)
plot(t_joint, joint_data(:, 1), '-', MarkerSize=3);
%axis([0, t(end), -10, 50]);
title('Gimbal Joint Angle - Tilt');
xlabel('time (s)');
ylabel('angle (degree)');

subplot(3, 1, 3)
%plot(t_joint, joint_data(:, 4), '-*', MarkerSize=3);
%axis([0, t(end), -10, 50]);
%title('Gimbal Angular Velocity');
%xlabel('time (s)');
%ylabel('pan (deg/s)');

plot(t_uav, uav_data(:, 3), '-', MarkerSize=3);
%axis([0, t(end), -10, 50]);
title('UAV Z Distance');
xlabel('time (s)');
ylabel('height (m)');