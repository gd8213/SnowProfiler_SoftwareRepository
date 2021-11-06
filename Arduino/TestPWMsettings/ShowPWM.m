clc; clear; close all;


%% Import data from text file
% Script for importing data from the following text file:
%
%    filename: C:\Users\kaebi\OneDrive - mci4me.at\03_MCI Master\3.Semester_WS2021\Project 2\SoftwareRepository\Arduino\TestPWMsettings\Data.txt
%
% Auto-generated by MATLAB on 06-Nov-2021 14:36:52

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 20);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["VarName1", "VarName2", "VarName3", "VarName4", "VarName5", "VarName6", "VarName7", "VarName8", "VarName9", "VarName10", "VarName11", "VarName12", "VarName13", "VarName14", "VarName15", "VarName16", "VarName17", "VarName18", "VarName19", "VarName20"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Specify variable properties
opts = setvaropts(opts, ["VarName1", "VarName2", "VarName3", "VarName4", "VarName5", "VarName6", "VarName7", "VarName8", "VarName9", "VarName10", "VarName11", "VarName12", "VarName13", "VarName14", "VarName15", "VarName16", "VarName17", "VarName18", "VarName19", "VarName20"], "DecimalSeparator", ",");
opts = setvaropts(opts, ["VarName1", "VarName2", "VarName3", "VarName4", "VarName5", "VarName6", "VarName7", "VarName8", "VarName9", "VarName10", "VarName11", "VarName12", "VarName13", "VarName14", "VarName15", "VarName16", "VarName17", "VarName18", "VarName19", "VarName20"], "ThousandsSeparator", ".");

% Import the data
Data = readtable("Data.txt", opts);

%% Convert to output type
Data = table2array(Data)';

%% Clear temporary variables
clear opts



%% Plot
figure
plot(Data(:,1)/1000,Data(:,2), 'x-')
grid on

figure
stairs(Data(:,1)/1000,Data(:,2))
grid on

%% 
freq = 1000000 / (Data(3,1)-Data(1,1))
duty = (Data(2,1)/Data(3,1))

