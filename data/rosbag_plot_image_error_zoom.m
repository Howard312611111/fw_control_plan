clear 
clc

% read rosbag
bag = rosbag('track_error_2023-08-17-11-52-05.bag');
bag.AvailableTopics

bag_pos = select(bag, "Topic", "/uav0/tracking/image_error");
pos = readMessages(bag_pos, "DataFormat", "struct");


dataNum1 = size(pos);


dataNum = dataNum1(1);

% time
Duration = bag.EndTime - bag.StartTime;
dt = Duration / dataNum;
t = 0:dt:Duration-dt;




pos_data = zeros(dataNum, 3);



for i = 1:dataNum
    pos_data(i, 1) = pos{i}.X;
    pos_data(i, 2) = pos{i}.Y;
    pos_data(i, 3) = pos{i}.Z;

end



%% plot
% image error
figure(1)
subplot(2,1,1);
plot(t, pos_data(:, 1), '-', MarkerSize=3);
%axis([0, t(end), -10, 50]);
title('Error of (u-cu)');
xlabel('time (s)');
ylabel('u direction (pixel)');


subplot(2,1,2);
plot(t, pos_data(:, 2), '-', MarkerSize=3);
%axis([0, t(end), -6, 14]);
title('Error of (v-cv)');
xlabel('time (s)');

