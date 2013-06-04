% clc

% ch0 is the timing data for QNX loop; 
% ch1 is the timing data for control loop; 
% ch2 is the velocity data sent from camare;
% ch3 is the velocity data got from the encoder;

%%
framerate = 250;
core_rate = 10000;

num = length(ch3);

for i = 1:num
    if isinf(ch3(i))
        n = i-1;
        break;
    end
end

core_time = ch0(1:n);
cl_time = ch1(1:n);
cvel = ch2(1:n);
evel = ch3(1:n);
% rnt = ch3(1:n);
rnt = 1:n;
%%
h = gcf;
if (h == 1) 
    h = 0;
end

% These three plots are for QNX core loop timing
figure(h+1)
normplot(core_time(1:n));
title('Intigrated Probability plot of QNX loop period time');

figure(h+2)
int = min(core_time(1:n)):0.0000002:max(core_time(1:n));
hist(core_time(1:n),int);
title(['Histogram of the QNX loop period time(s). Total samples:' num2str(n)]);
xlabel('Period time'); ylabel('Number of samples in each time period');
axis tight

figure(h+3)
plot(core_time(1:n), 'linewidth', 1);
line([rnt(1) rnt(n)], [1/core_rate 1/core_rate], 'linewidth', 2, 'color', 'r');
ave_core_time = mean(core_time(1:n));
line([rnt(1) rnt(n)], [ave_core_time ave_core_time], 'linewidth', 2, 'color', 'g');
title('Plot of QNX loop period time vs run count #');
legend('QNX loop time', ['Theoretical value(' num2str(core_rate) 'Hz)'], ['Ave value = ' num2str(ave_core_time) '; Std = ' num2str(std(core_time(1:n)))]);
xlabel('Run count'); ylabel('Period time(sec)');
axis tight

%%
% The following three plots are for control loop timing
figure(h+4)
normplot(cl_time(1:n));
title('Intigrated Probability plot of Control loop period time');

figure(h+5)
int = min(cl_time(1:n)):0.0001:max(cl_time(1:n));
hist(cl_time(1:n),int);
title(['Histogram of the contorl loop period time(s). Total samples:' num2str(n)]);
xlabel('Period time'); ylabel('Number of samples in each time period');
axis tight

figure(h+6)
plot(cl_time(1:n), 'linewidth', 1);
line([rnt(1) rnt(n)], [1/framerate 1/framerate], 'linewidth', 2, 'color', 'r');
ave_cl_time = mean(cl_time(1:n));
line([rnt(1) rnt(n)], [ave_cl_time ave_cl_time], 'linewidth', 2, 'color', 'g');
title('Plot of control loop period time vs run count #');
legend('Control loop time', ['Theoretical value(' num2str(framerate) 'Hz)'], ['Ave value = ' num2str(ave_cl_time) '; Std = ' num2str(std(cl_time(1:n)))]);
xlabel('Run count'); ylabel('Period time(sec)');
axis tight

%%
% Here calculating the velocity data sent from the QNX
cvel_mean = mean(cvel);
cvel_std = std(cvel);
cvel_max = max(cvel);
cvel_min = min(cvel);

evel_mean = mean(evel);
evel_std = std(evel);
evel_max = max(evel);
evel_min = min(evel);

figure(h+7)

subplot(2,1,1);
plot(rnt, cvel);
line([rnt(1) rnt(n)], [cvel_mean cvel_mean], 'linewidth', 2, 'color', 'r');
line([rnt(1) rnt(n)], [cvel_max cvel_max], 'linewidth', 2, 'color', 'g');
line([rnt(1) rnt(n)], [cvel_min cvel_min], 'linewidth', 2, 'color', 'g');
title('Camera velocity data (rad/sec)');
legend('Angular Vel by camera',['Mean = ' num2str(cvel_mean) '; Std = ' num2str(cvel_std)]);
axis tight

subplot(2,1,2);
plot(rnt, evel);
line([rnt(1) rnt(n)], [evel_mean evel_mean], 'linewidth', 2, 'color', 'r');
line([rnt(1) rnt(n)], [evel_max evel_max], 'linewidth', 2, 'color', 'g');
line([rnt(1) rnt(n)], [evel_min evel_min], 'linewidth', 2, 'color', 'g');
legend('Angular Vel by encoder',['Mean = ' num2str(evel_mean) '; Std = ' num2str(evel_std)]);
title('Encoder velocity data');
axis tight

%%
% figure(10)
% a = rand(10000,1);
% normplot(a);
% hist(a);