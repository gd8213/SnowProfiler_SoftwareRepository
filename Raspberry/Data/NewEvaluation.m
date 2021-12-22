clear; clc; close all;
filename = "Me.csv"

%% Import data from text file

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 4);

% Specify range and delimiter
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["Times", "Accelerationmg", "Times_1", "ForceN"];
opts.VariableTypes = ["double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
Copyof20211221140353 = readtable(filename, opts);

%% Convert to output type
rawData = table2array(Copyof20211221140353);
tAccel = rawData(:,1);
accel = rawData(:,2);
tForce = rawData(:,3);
force = rawData(:,4);
%% Clear temporary variables
clear opts

%% Evaluation
figure
acc2 = interp(accel,2);
velocity = cumtrapz(tAccel,acc2(1:4096)/(9.81*1000));
position = cumtrapz(tAccel, velocity);
plot(position,force);



figure
subplot(2,1,1)
plot(tAccel, accel/1000)
grid on
legend("Acceleration")
ylabel("Acceleration in g")
xlabel("Time in sec")

subplot(2,1,2)
plot(tForce, force)
grid on
legend("Force")
ylabel("Force in N")
xlabel("Time in sec")
ylim([0 inf])

figure
plot(tAccel, accel/1000, tForce, force)
legend("Acceleration in g", "Force in N")
grid on
ylim([0 inf])


figure
xlabel("Time in sec")
yyaxis left
plot(tAccel, accel/1000)
ylabel("Acceleration in g")
grid on
yyaxis right
plot(tForce, force)
ylim([0 inf])
ylabel("Force in N")
grid on

