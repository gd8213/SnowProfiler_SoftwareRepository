clear all
close all
clc


filename='2021-12-20_15-39-06.csv';
[t, acc, f] = importfile(filename, [2, Inf]);


% for i=1:length(acc)
%     if acc(i)<100
%         acc(i)=acc(i-1);
%     end
% end

figure
plot(t,acc,t,f)
grid on
legend('Accelerometer','Force Sensor')
