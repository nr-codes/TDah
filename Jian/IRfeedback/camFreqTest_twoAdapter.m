load 'TwoAdapter.mat';
% The data is obtained by "WireShark"

%%
tt1 = TwoAdapter.camera2pc;
% n = length(tt1);

% a1 = zeros(n-1,1);
% for i = 1:n -1
%     a1(i) = tt1(i+1)-tt1(i);
% end

f = gcf;
figure(f+1)
subplot(3,1,1);
plot(diff(tt1))
title('Two Adapter, by Wireshark; Period of camera sending data to PC(250Hz PC calculates velocity and send to QNX; QNX outputs control)');
ylabel('Period/sec'); xlabel('Samples');

%%
tt2 = TwoAdapter.QnxUDP_WiresharkTiming;

subplot(3,1,2);
plot(diff(tt2))
title('Only QNX sends UDP data, by Wireshark; Period captured by WireShark (250Hz)');
ylabel('Period/sec'); xlabel('Samples');

%%
tt3 = TwoAdapter.QnxUDP_QnxEndPeriod;

subplot(3,1,3);
plot(tt3)
title('Only QNX sends UDP data, by Wireshark; Period recorded by QNX (250Hz)');
ylabel('Period/sec'); xlabel('Samples');

