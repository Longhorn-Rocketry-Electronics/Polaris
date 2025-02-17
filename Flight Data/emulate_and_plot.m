% This code takes a raw polaris data file, creates a MET (mission time)
% launch clock, and then plots the altitude and polaris state data. 0 =
% prelaunch, 1 = launch, 2 = post apogee, 3 = landing.
clear; clc; close all

% VARIABLES TO CHANGE ===================================================
last_second_of_graph = 175;

% Specify the file name (modify this according to your file location)
filename = 'Polaris AVTR FTE2 Raw.csv'; %Transonic L2 Rocket
%filename = 'Polaris AVTR FTE1 Raw.csv'; %L2 Rocket
%filename = 'TRA_Failed_Polaris.csv'; % TRA's failed flight
%filename = 'Drone Polaris Test.csv'; % Polaris drone test
% Read the CSV file into a table
flightData = readtable(filename);


% DATA CODE ============================================================
% Align launch clock
flightData.time_s = flightData.time_us / 1E6;
index_ascend = find(flightData.altitude > 2, 1, 'first');
time_ascend = flightData.time_s(index_ascend);
flightData.met_s = flightData.time_s - time_ascend; %2616.9

% Display the first few rows to verify the data
disp('First few rows of the dataset:');
disp(flightData(1:5, :));

% Extract relevant columns (assuming standard rocket flight data format)
% Plot Altitude vs. Time
time = flightData.met_s;          % Time (s)
altitude = flightData.altitude;  % Altitude (m)
state = flightData.state;

% Plot
figure;
xlim([-5, min(max(flightData.met_s),last_second_of_graph)])
yyaxis left
plot(time, altitude, 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Rocket Altitude vs. Time');
yyaxis right
plot(time, state)
grid on;
legend;

