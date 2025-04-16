clear 
clc

% read rosbag
bag = rosbag('track_error_2023-08-17-11-38-09.bag');
bag.AvailableTopics

bag_pos = select(bag, "Topic", "/uav0/tracking/image_error");
pos = readMessages(bag_pos, "DataFormat", "struct");
bag_yolo = select(bag, "Topic", "/uav0/tracking/image_error/yolo");
yolo = readMessages(bag_yolo, "DataFormat", "struct");

dataNum1 = size(pos);
dataNum2 = size(yolo);

dataNum = dataNum1(1);
if dataNum2(1) < dataNum1(1)
    dataNum = dataNum2(1);
end

% time
Duration = bag.EndTime - bag.StartTime;
dt = Duration / dataNum;
t = 0:dt:Duration-dt;




pos_data = zeros(dataNum, 3);
yolo_data = zeros(dataNum, 3);


for i = 1:dataNum
    pos_data(i, 1) = pos{i}.X;
    pos_data(i, 2) = pos{i}.Y;
    pos_data(i, 3) = pos{i}.Z;

    yolo_data(i, 1) = yolo{i}.X;
    yolo_data(i, 2) = yolo{i}.Y;
    yolo_data(i, 3) = yolo{i}.Z;
end



%% plot
% image error
figure(1)
subplot(2,1,1);
plot(t, pos_data(:, 1), '-', t, yolo_data(:, 1), '--', MarkerSize=3);
%axis([0, t(end), -10, 50]);
title('Error of (u-cu)');
xlabel('time (s)');
ylabel('u direction (pixel)');
lgd = legend('3D position', 'yolo');

subplot(2,1,2);
plot(t, pos_data(:, 2), '-', t, yolo_data(:, 2), '--', MarkerSize=3);
%axis([0, t(end), -6, 14]);
title('Error of (v-cv)');
xlabel('time (s)');
ylabel('v direction (pixel)');
lgd = legend('3D position', 'yolo');
