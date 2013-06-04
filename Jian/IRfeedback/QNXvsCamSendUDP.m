% Ploting period data of receiving UDP packets from Cam and QNX sending the same amount of data per packet 

load 'Jan9periodDATA.mat';
% The data is obtained by "WireShark"

%%
tt1 = periodDATA.RecvCamPCend;
% n = length(tt1);

% a1 = zeros(n-1,1);
% for i = 1:n -1
%     a1(i) = tt1(i+1)-tt1(i);
% end

f = gcf;
figure(f+1)
subplot(4,1,1);
plot(tt1)
title('Time stamp data from PC; Period of camera sending data to PC(250Hz PC calculates velocity only)');
ylabel('Period/sec'); xlabel('Samples'); 
legend(['Ave:' num2str(mean(tt1)) '; Std:' num2str(std(tt1))]);
axis tight
%%
tt2 = periodDATA.RecvCamWireSharkEnd;

subplot(4,1,2);
plot(tt2)
title('Period from WireShark data; Period of camera sending data to PC(250Hz PC calculates velocity only)');
ylabel('Period/sec'); xlabel('Samples');
legend(['Ave:' num2str(mean(tt2)) '; Std:' num2str(std(tt2))]);
axis tight
%%
tt3 = periodDATA.RecvQNXqnxEnd;

subplot(4,1,3);
plot(tt3)
title('Period recorded by QNX; Period of QNX sending same size of UDP packets to PC (250Hz)');
ylabel('Period/sec'); xlabel('Samples');
legend(['Ave:' num2str(mean(tt3)) '; Std:' num2str(std(tt3))]);
axis tight
%%
tt4 = periodDATA.RecvQNXWireSharkEnd;

subplot(4,1,4);
plot(tt4)
title('Period from WireShark data; Period of QNX sending same size of UDP packets to PC (250Hz)');
ylabel('Period/sec'); xlabel('Samples');
legend(['Ave:' num2str(mean(tt4)) '; Std:' num2str(std(tt4))]);
axis tight
