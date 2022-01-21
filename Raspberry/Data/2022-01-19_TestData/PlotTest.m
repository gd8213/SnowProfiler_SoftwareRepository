function [] = PlotTest(testNumber, tAccel,accel, tForce, force, position)

    figure
    subplot(1,2,1)
        yyaxis left
            plot(tAccel(:,testNumber), accel(:,testNumber));
            ylabel("Accel in g")
        yyaxis right
            plot(tForce(:,testNumber), force(:,testNumber));
            ylim([-1 inf])
            ylabel("Force in N")
        xlabel("Time in sec")
        grid on
        title("Test " + int2str(testNumber)+ " (Time)")
        xlim([0 2])
    
    subplot(1,2,2)
        plot(position(:,testNumber), force(:,testNumber));
        xlabel("Position in mm")
        ylim([-1 inf])
        grid on
        title("Test " + int2str(testNumber) + " (Position)")
end

