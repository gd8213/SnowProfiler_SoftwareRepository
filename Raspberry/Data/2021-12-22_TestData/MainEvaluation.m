%% Clean up
clc; clear; close all;
numberOfTests = 9;

%% Import Data
for i=1:numberOfTests   % Get all csv files
    folderName = "Test_" + int2str(i);
    folderContent = dir(folderName);
    for j = 3 : length(folderContent)
        if (folderContent(j).isdir == false)
            [~,~,ext] = fileparts(folderContent(j).name);
            if (ext == ".csv")
                csvFile = folderContent(j).name;
                break;
            end
        end
    end
    
    
    [tAccel(:,i), accel(:,i), tForce(:,i), force(:,i)] = ImportCSV(folderName + "/" + csvFile);
    
    % Calculate Position
%     acc2 = interp(accel(:,i),2);
%     velocity = cumtrapz(tAccel(:,i),acc2(1:4096)/(9.81*1000));
%     position(:,i) = cumtrapz(tAccel(:,i), velocity);
    
    velocity = cumtrapz(tAccel(:,i),accel(:,i)/1000*9.81);  % accel to m/s^2, velocity in m/s
    positionCoarse(:,i) = cumtrapz(tAccel(:,i),velocity);     % position in m
    position(:,i) = interp1(tAccel(:,i), positionCoarse(:,i), tForce(:,i));     % interpolation
end

accel = accel/1000;             % to g
position = position*1000;       % to mm
clear folderName folderContent i j ext csvFile velocity acc2


%% Plot data

% Overview Data
PlotOverview(numberOfTests, tAccel,accel, tForce, force, position);

% Detailed Test
 PlotTest(1, tAccel, accel, tForce, force, position)
% PlotTest(2, tAccel, accel, tForce, force, position)
% PlotTest(3, tAccel, accel, tForce, force, position)
% PlotTest(4, tAccel, accel, tForce, force, position)
% PlotTest(5, tAccel, accel, tForce, force, position)
% PlotTest(6, tAccel, accel, tForce, force, position)
% PlotTest(7, tAccel, accel, tForce, force, position)
% PlotTest(8, tAccel, accel, tForce, force, position)
% PlotTest(9, tAccel, accel, tForce, force, position)

    