% Brendan Martin January 2018
% Plot Attitude_Log.txt 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear all; clc;

%% Open the File for read and parse into usable matrix

File = fopen('Attitude_Log.txt', 'r');
formatSpec = '%f';
Raw_Data = fscanf(File, formatSpec);
Total_Rows = length(Raw_Data / 7);
row = 1;
col = 1;
for i = 1:Total_Rows
    Data(row, col) = Raw_Data(i);
    col = col + 1;
    if col == 8
        col = 1;
        row = row + 1;
    end
end

Time(1,1) = 0;
for i = 2:length(Data)
    Time(i,1) = Data(i, 1) + Time(i-1);
end

%% Plot Data vs. Time

plot(Time, Data(:,2), '--', 'LineWidth', 1)
hold on
plot(Time, Data(:,3), '--', 'LineWidth', 1)
plot(Time, Data(:,4), '--', 'LineWidth', 1)
plot(Time, Data(:,5), '--', 'LineWidth', 1)
plot(Time, Data(:,6),'LineWidth', 2)
plot(Time, Data(:,7),'LineWidth', 2)
xlabel('Time (seconds)')
ylabel('IMU Reference Frame Attitude Angle (degrees)')
title('Attitude Log Data Export')
legend('Accelerometer Pitch', 'Accelerometer Roll', 'Gyro Pitch', ...
       'Gyro Roll', 'Fused Pitch', 'Fused Roll', 'Location','NorthWest')




