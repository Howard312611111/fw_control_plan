clear 
clc

% read rosbag
bag = rosbag('ukf_fw_2023-08-27-13-20-01.bag');
bag.AvailableTopics;

%% ukf data
bag_gt = select(bag, "Topic", "/uav0/estimation/ukf/groundtruth");
gt = readMessages(bag_gt, "DataFormat", "struct");
bag_out = select(bag, "Topic", "/uav0/estimation/ukf/output_data");
output = readMessages(bag_out, "DataFormat", "struct");

dataNum_gt = size(gt);
dataNum_output = size(output);

dataNum_ukf = dataNum_gt(1);
if dataNum_output(1) < dataNum_gt(1)
    dataNum_ukf = dataNum_output(1);
end

% time
Duration = bag.EndTime - bag.StartTime;
dt_ukf = Duration / dataNum_ukf;
t_ukf = 0:dt_ukf:Duration-dt_ukf;

%% tracking error
bag_error = select(bag, "Topic", "/uav0/tracking/image_error/yolo");
error = readMessages(bag_error, "DataFormat", "struct");

dataNum_error = size(error);
dataNum_error = dataNum_error(1);

% time
dt_error = Duration / dataNum_error;
t_error = 0:dt_error:Duration-dt_error;

%% insert data
gt_data = zeros(dataNum_ukf, 8);
output_data = zeros(dataNum_ukf, 8);
error_data = zeros(dataNum_error, 2);

for i = 1:dataNum_ukf
    gt_data(i, 1) = gt{i}.Feature1.Data;
    gt_data(i, 2) = gt{i}.Feature2.Data;
    gt_data(i, 3) = gt{i}.TargetPose.X;
    gt_data(i, 4) = gt{i}.TargetPose.Y;
    gt_data(i, 5) = gt{i}.TargetPose.Z;
    gt_data(i, 6) = gt{i}.TargetVel.X;
    gt_data(i, 7) = gt{i}.TargetVel.Y;
    gt_data(i, 8) = gt{i}.TargetVel.Z;

    output_data(i, 1) = output{i}.Feature1.Data;
    output_data(i, 2) = output{i}.Feature2.Data;
    output_data(i, 3) = output{i}.TargetPose.X;
    output_data(i, 4) = output{i}.TargetPose.Y;
    output_data(i, 5) = output{i}.TargetPose.Z;
    output_data(i, 6) = output{i}.TargetVel.X;
    output_data(i, 7) = output{i}.TargetVel.Y;
    output_data(i, 8) = output{i}.TargetVel.Z;
end

for i = 1:dataNum_error
    error_data(i, 1) = error{i}.X;
    error_data(i, 2) = error{i}.Y;
end

%% plot ukf
% position
figure(1)
subplot(3,1,1);
plot(t_ukf, gt_data(:, 3), '--', t_ukf, output_data(:, 3));
%axis([0, t_ukf(end), 10, 22]);
title('Position Estimation');
xlabel('time (s)');
ylabel('X (m)');
lgd = legend('Ground Truth', 'Estimate');

subplot(3,1,2);
plot(t_ukf, gt_data(:, 4), '--', t_ukf, output_data(:, 4));
%axis([0, t_ukf(end), 400, 600]);
title('Position Estimation');
xlabel('time (s)');
ylabel('Y (m)');
lgd = legend('Ground Truth', 'Estimate');

subplot(3,1,3);
plot(t_ukf, gt_data(:, 5), '--', t_ukf, output_data(:, 5));
%axis([0, t_ukf(end), -3, 3]);
title('Position Estimation');
xlabel('time (s)');
ylabel('Z (m)');
lgd = legend('Ground Truth', 'Estimate');

% velocity
figure(2)
subplot(3,1,1);
plot(t_ukf, gt_data(:, 6), '--', t_ukf, output_data(:, 6));
%axis([0, t_ukf(end), 12, 18]);
title('Velocity Estimation');
xlabel('time (s)');
ylabel('X (m/s)');
lgd = legend('Ground Truth', 'Estimate');

subplot(3,1,2);
plot(t_ukf, gt_data(:, 7), '--', t_ukf, output_data(:, 7));
%axis([0, t_ukf(end), -1, 2]);
title('Velocity Estimation');
xlabel('time (s)');
ylabel('Y (m/s)');
lgd = legend('Ground Truth', 'Estimate');

subplot(3,1,3);
plot(t_ukf, gt_data(:, 8), '--', t_ukf, output_data(:, 8));
%axis([0, t_ukf(end), -0.5, 0.3]);
title('Velocity Estimation');
xlabel('time (s)');
ylabel('Z (m/s)');
lgd = legend('Ground Truth', 'Estimate');

% state
figure(3)
subplot(2,1,1);
plot(t_ukf, gt_data(:, 1), '--', t_ukf, output_data(:, 1));
%axis([0, t_ukf(end), 0.1, 0.7]);
title('State Estimation');
xlabel('time (s)');
ylabel('x1');
lgd = legend('Ground Truth', 'Estimate');

subplot(2,1,2);
plot(t_ukf, gt_data(:, 2), '--', t_ukf, output_data(:, 2));
%axis([0, t_ukf(end), -0.05, 0.1]);
title('State Estimation');
xlabel('time (s)');
ylabel('x2');
lgd = legend('Ground Truth', 'Estimate');

%% plot tracking error
figure(4)
% 1280 x 720 -> 640 x 360
subplot(2, 1, 1);
plot(t_error, error_data(:, 1), '-');
%axis([0, t_error(end), -500,500]);
title('Tracking Error (u Direction)');
xlabel('time (s)');
ylabel('Error (pixel)');
subplot(2, 1, 2);
plot(t_error, error_data(:, 2), '-');
%axis([0, t_error(end), -300, 300]);
title('Tracking Error (v Direction)');
xlabel('time (s)');
ylabel('Error (pixel)');