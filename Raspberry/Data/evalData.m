clear all
close all
clc


filename='2021-12-21_12-38-33.csv';
[t, acc, f] = importfile(filename, [2, Inf]);


% for i=1:length(acc)
%     if acc(i)<100
%         acc(i)=acc(i-1);
%     end
% end

figure
subplot(2,1,1)
plot(t,acc)
grid on
legend('Accelerometer')

subplot(2,1,2)
plot(t,f)
grid on
legend('Force Sensor')
