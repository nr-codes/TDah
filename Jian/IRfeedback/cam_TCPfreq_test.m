load 'cp.mat';
% The data is obtained by "WireShark"

%%
tt1 = cp.data_recv_only;
n = length(tt1);

a1 = zeros(n-1,1);
for i = 1:n -1
    a1(i) = tt1(i+1)-tt1(i);
end

f = gcf;
figure(f+1)
subplot(4,1,1);
plot(a1)
title('Period of camera sending data(250Hz PC only recieve, no calculation)');
ylabel('Period/sec'); xlabel('Samples');

%%
tt2 = cp.data_CalCoordi;
n = length(tt2);

a2 = zeros(n-1,1);
for i = 1:n -1
    a2(i) = tt2(i+1)-tt2(i);
end
% f = f+1;
% figure(f)
subplot(4,1,2);
plot(a2)
title('Period of camera sending data(250Hz PC calculate the coordinates, no sending out)');
ylabel('Period/sec'); xlabel('Samples');

%%
tt3 = cp.data_withControlOutput_cam2pc;
n = length(tt3);

a3 = zeros(n-1,1);
for i = 1:n -1
    a3(i) = tt3(i+1)-tt3(i);
end
% f = f+1;
% figure(f)
subplot(4,1,3);
plot(a3)
title('Period of camera sending data to PC(250Hz With both PC and QNX communications, & outputing control)');
ylabel('Period/sec'); xlabel('Samples');

%%
tt4 = cp.data_withControlOutput_pc2qnx;
n = length(tt4);

a4 = zeros(n-1,1);
for i = 1:n -1
    a4(i) = tt4(i+1)-tt4(i);
end
% f = f+1;
% figure(f)
subplot(4,1,4);
plot(a4)
title('Period of PC sending data to QNX(250Hz With both PC and QNX communications, & outputing control)');
ylabel('Period/sec'); xlabel('Samples');

%%
figure(f+2)

subplot(2,1,1);
plot(diff(cp.timestamp_onlyCalCoordi));
title('Period data from "time stamp" function(250Hz PC calculating the coordinates, but no sending out to QNX)');
ylabel('Period/sec'); xlabel('Samples');
axis tight

%%

subplot(2,1,2);
plot(diff(cp.timestamp_withControlOutput));
title('Period data from "time stamp" function(250Hz With both PC and QNX communications, & outputing control)');
ylabel('Period/sec'); xlabel('Samples');
axis tight