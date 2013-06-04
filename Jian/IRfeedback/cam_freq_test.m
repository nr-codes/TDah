clc

data = load('Nov13CameraTestTiming.mat');
f = gcf;
nplots = 5;
%%
buff = data.data;
n = length(buff);
figure(f+1)
subplot(nplots,1,1);
plot(buff(2:n),':.b','marker','o');
title('Period of camera LED flashing(normal case)');
axis tight;
% f = f+1;
% figure(f)
% hist(buff(2:n),min(buff(2:n)):1:max(buff));
%%
buff = data.data_2min_no_glitch;
n = length(buff);
% f = f+1;
% figure(f)
subplot(nplots,1,2);
plot(buff(2:n),':.b','marker','o');
title('Period of camera LED flashing(2 mins run without glitch)');
axis tight;
% f = f+1;
% figure(f)
% hist(buff(2:n),min(buff(2:n)):1:max(buff));
%%
buff = data.data_box;
n = length(buff);
% f = f+1;
% figure(f)
subplot(nplots,1,3);
plot(buff(2:n),':.b','marker','o');
title('Period of camera LED flashing(with box covered)');
axis tight;
% f = f+1;
% figure(f)
% hist(buff(50:n),min(buff(50:n)):1:max(buff));
%%
buff = data.data_box2;
n = length(buff);
% f = f+1;
% figure(f)
subplot(nplots,1,4);
plot(buff(2:n),':.b','marker','o');
title('Period of camera LED flashing(with box covered 2)');
axis tight;
% f = f+1;
% figure(f)
% hist(buff(50:n),min(buff(50:n)):1:max(buff));
%%
buff = data.data_partial_box;
n = length(buff);
% f = f+1;
% figure(f)
subplot(nplots,1,5);
plot(buff(2:n),':.b','marker','o');
title('Period of camera LED flashing(partial box covered)');
axis tight;
% f = f+1;
% figure(f)
% hist(buff(50:n),min(buff(100:n)):50:max(buff));
% axis tight;
