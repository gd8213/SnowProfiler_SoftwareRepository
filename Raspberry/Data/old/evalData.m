clear all
close all
clc


filename='ShowThis.csv';
% filename='2021-12-21_11-43-53.csv';
[t, acc, f] = importfile(filename, [2, Inf]);


% for i=1:length(acc)
%     if acc(i)<100
%         acc(i)=acc(i-1);
%     end
% end

figure
subplot(2,1,1)
plot(t,acc/1000)
grid on
legend('Accelerometer in g')

subplot(2,1,2)
plot(t,f)
grid on
legend('Force in N')


