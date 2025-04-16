clear 
clc

% read rosbag
bag = rosbag('ukf_fw_0827_cir-oppsite/ukf_fw_2023-08-27-12-14-43.bag');
bag.AvailableTopics;

bag_gt = select(bag, "Topic", "/uav0/estimation/ukf/groundtruth");
gt = readMessages(bag_gt, "DataFormat", "struct");
bag_out = select(bag, "Topic", "/uav0/estimation/ukf/output_data");
output = readMessages(bag_out, "DataFormat", "struct");

dataNum1 = size(gt);
dataNum2 = size(output);

dataNum = dataNum1(1);
if dataNum2(1) < dataNum1(1)
    dataNum = dataNum2(1);
end

% time
Duration = bag.EndTime - bag.StartTime;
dt = Duration / dataNum;
t = 0:dt:Duration-dt;

gt_data = zeros(dataNum, 8);
output_data = zeros(dataNum, 8);
error = zeros(dataNum, 6);
nonError = zeros(dataNum, 1);

for j = 1:(dataNum)
    i = j;
    gt_data(j, 1) = gt{i}.Feature1.Data;
    gt_data(j, 2) = gt{i}.Feature2.Data;
    gt_data(j, 3) = gt{i}.TargetPose.X;
    gt_data(j, 4) = gt{i}.TargetPose.Y;
    gt_data(j, 5) = gt{i}.TargetPose.Z;
    gt_data(j, 6) = gt{i}.TargetVel.X;
    gt_data(j, 7) = gt{i}.TargetVel.Y;
    gt_data(j, 8) = gt{i}.TargetVel.Z;

    output_data(j, 1) = output{i}.Feature1.Data;
    output_data(j, 2) = output{i}.Feature2.Data;
    output_data(j, 3) = output{i}.TargetPose.X;
    output_data(j, 4) = output{i}.TargetPose.Y;
    output_data(j, 5) = output{i}.TargetPose.Z;
    output_data(j, 6) = output{i}.TargetVel.X;
    output_data(j, 7) = output{i}.TargetVel.Y;
    output_data(j, 8) = output{i}.TargetVel.Z;

    error(j, 1) =  ( output_data(j, 3) - gt_data(j, 3) );
    error(j, 2) =  ( output_data(j, 4) - gt_data(j, 4) );
    error(j, 3) =  ( output_data(j, 5) - gt_data(j, 5) );
    error(j, 4) =  ( output_data(j, 6) - gt_data(j, 6) );
    error(j, 5) =  ( output_data(j, 7) - gt_data(j, 7) );
    error(j, 6) =  ( output_data(j, 8) - gt_data(j, 8) );
end

%acc
t_sec = 0.2;
sec = t(end) / t_sec;
num = t_sec / dt;
acc = zeros(round(sec), 3);

i = 1;
for j = 1:round(sec)
    if (i+round(num)-1) < dataNum
        acc(j, 1) = ( gt{i+round(num)-1}.TargetVel.X - gt{i}.TargetVel.X ) / t_sec;
        acc(j, 2) = ( gt{i+round(num)-1}.TargetVel.Y - gt{i}.TargetVel.Y ) / t_sec;
        acc(j, 3) = ( gt{i+round(num)-1}.TargetVel.Z - gt{i}.TargetVel.Z ) / t_sec;
    end
    i = i + round(num)
end

%% plot
% position
figure(1)
subplot(3,1,1);
plot(t, gt_data(:, 3), '--', t, output_data(:, 3));
axis([0, t(end), -20, 30]);
title('Estimated Position of Target in The Earth Frame');
subtitle('Position on X Direction');
xlabel('time (s)');
ylabel('X (m)');
legend('Ground Truth', 'Estimate');
legend('Orientation','horizontal')

subplot(3,1,2);
plot(t, gt_data(:, 4), '--', t, output_data(:, 4));
axis([0, t(end), -20, 30]);
subtitle('Position on Y Direction');
xlabel('time (s)');
ylabel('Y (m)');
legend('Ground Truth', 'Estimate');
legend('Orientation','horizontal')

subplot(3,1,3);
plot(t, gt_data(:, 5), '--', t, output_data(:, 5));
axis([0, t(end), -10, 20]);
subtitle('Position on Z Direction');
xlabel('time (s)');
ylabel('Z (m)');
legend('Ground Truth', 'Estimate');
legend('Orientation','horizontal')



% velocity
figure(2)
subplot(3,1,1);
plot(t, gt_data(:, 6), '--', t, output_data(:, 6));
axis([0, t(end), -8, 8]);
yticks(-8:4:8);
title('Estimated Velocity of Target in The Earth Frame');
subtitle('Velocity on X Direction');
xlabel('time (s)');
ylabel('X (m/s)');
legend('Ground Truth', 'Estimate');
legend('Orientation','horizontal');

subplot(3,1,2);
plot(t, gt_data(:, 7), '--', t, output_data(:, 7));
axis([0, t(end), -8, 8]);
yticks(-8:4:8);
subtitle('Velocity on Y Direction');
xlabel('time (s)');
ylabel('Y (m/s)');
legend('Ground Truth', 'Estimate');
legend('Orientation','horizontal')

subplot(3,1,3);
plot(t, gt_data(:, 8), '--', t, output_data(:, 8));
axis([0, t(end), -8, 8]);
yticks(-8:4:8);
subtitle('Velocity on Z Direction');
xlabel('time (s)');
ylabel('Z (m/s)');
legend('Ground Truth', 'Estimate');
legend('Orientation','horizontal')

%%
% state
figure(3)
subplot(2,1,1);
plot(t, gt_data(:, 1), '--', t, output_data(:, 1));
axis([0, t(end), -0.06, 0.06]);
title('State x1 Estimation');
xlabel('time (s)');
ylabel('x1');
lgd = legend('Ground Truth', 'Estimate');

subplot(2,1,2);
plot(t, gt_data(:, 2), '--', t, output_data(:, 2));
axis([0, t(end), -0.06, 0.06]);
title('State x2 Estimation');
xlabel('time (s)');
ylabel('x2');
lgd = legend('Ground Truth', 'Estimate');


%%
% error of position
figure(4)
subplot(3,1,1);
plot(t, nonError(:, 1), 'k-', t, error(:, 1));
axis([0, t(end), -4, 4]);
yticks(-4:2:4);
title('Absolute Position Error');
subtitle('Error on X Direction');
xlabel('time (s)');
ylabel('error (m)');

subplot(3,1,2);
plot(t, nonError(:, 1), 'k-', t, error(:, 2));
axis([0, t(end), -4, 4]);
yticks(-4:2:4);
subtitle('Error on Y Direction');
xlabel('time (s)');
ylabel('error (m)');

subplot(3,1,3);
plot(t, nonError(:, 1), 'k-', t, error(:, 3));
axis([0, t(end), -4, 4]);
yticks(-4:2:4);
subtitle('Error on Z Direction');
xlabel('time (s)');
ylabel('error (m)');

% error of velocity
figure(5)
subplot(3,1,1);
plot(t, nonError(:, 1), 'k-', t, error(:, 4));
axis([0, t(end), -2, 2]);
yticks(-2:1:2);
title('Absolute Velocity Error');
subtitle('Error on X Direction');
xlabel('time (s)');
ylabel('error (m/s)');

subplot(3,1,2);
plot(t, nonError(:, 1), 'k-', t, error(:, 5));
axis([0, t(end), -2, 2]);
yticks(-2:1:2);
subtitle('Error on Y Direction');
xlabel('time (s)');
ylabel('error (m/s)');

subplot(3,1,3);
plot(t, nonError(:, 1), 'k-', t, error(:, 6));
axis([0, t(end), -2, 2]);
yticks(-2:1:2);
subtitle('Error on Z Direction');
xlabel('time (s)');
ylabel('error (m/s)');