function [] = PlotOverview(numberOfTests, tAccel,accel, tForce, force, position)

%% Force and Acceleration depending on time
    figure
    for i=1:numberOfTests   % Get all csv files
        subplot(3,3,i)
        yyaxis left
        plot(tAccel(:,i), accel(:,i));
        ylabel("Accel in g")
        yyaxis right
        plot(tForce(:,i), force(:,i));
        ylim([-1 inf])
        ylabel("Force in N")
        
        xlim([0 2])
        grid on
        title("Test " + int2str(i))
        xlabel("Time in sec")
        
    end
    
    %% Acceleration depending on time
    figure
    for i=1:numberOfTests   % Get all csv files
        subplot(3,3,i)
        plot(tAccel(:,i), accel(:,i));
        ylabel("Accel in g")
        
        xlim([0 1])
        grid on
        title("Test " + int2str(i))
        xlabel("Time in sec")
        
    end
    
    %% Force depending on time
    figure
    for i=1:numberOfTests   % Get all csv files
        subplot(3,3,i)
        plot(tForce(:,i), force(:,i));
        ylabel("Force in N")
        
        xlim([0 1])
        ylim([-1 inf])
        grid on
        title("Test " + int2str(i))
        xlabel("Time in sec")
        
    end
    
    %% Force depending on position
    figure
    for i=1:numberOfTests   % Get all csv files
        subplot(3,3,i)
        
        plot(position(:,i), force(:,i));
        ylim([-1 inf])
        ylabel("Force in N")

        grid on
        xlim([0 inf])
        title("Test " + int2str(i))
        xlabel("Position in mm")
    end
end

