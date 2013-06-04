% Real Time Plot via TCP/IP example
% Alex Makhlin, Kinea Design
% August 4, 2010

clear t;

% create a tcp/ip socket
t=tcpip('192.168.1.65',3100);
% x86 is littleEndian, matlab defaults to bigEndian for some reason
set(t,'ByteOrder','littleEndian');
% make the input buffer larger, default is 512 bytes
set(t,'InputBufferSize',4095*4); % 4095
% fail sooner due to lack of connection
set(t,'Timeout',2); %2
% open the file descriptor associated with the socket
fopen(t);

freq = 250; % Hz. To sync with the qnx program which the data is sent from.
time = 20; % sec

% plot this many datapoints on the screen.  np: number of data points?
np= freq*time; %10000; %10000; 
% get/plot this many datapoints for each signal with every read on the
% socket. This is why it looks like a discontinous jumping when being plot.
plot_batch = 25; %freq/10; % read every 0.1 sec; freq/10

% setup plot limits
axes('xlim',[1,np],'ylim',[-0.1, 6.5]); % -20, 20
%axes('xlim',[1,np],'ylim',[-20, 20]); % -20, 20
x=1:np;

%plot(x, 0*pi()/180*ones(size(x')));hold on;
% plot(x, 30*pi()/180*ones(size(x)));hold on;
% plot(x, 60*pi()/180*ones(size(x)));hold on;
%plot(x, 90*pi()/180*ones(size(x)));hold on;
% plot(x, 120*pi()/180*ones(size(x)));hold on;
% plot(x, 150*pi()/180*ones(size(x)));hold on;
%plot(x, 180*pi()/180*ones(size(x)));hold on;
% plot(x, 210*pi()/180*ones(size(x)));hold on;
% plot(x, 240*pi()/180*ones(size(x)));hold on;
%plot(x, 270*pi()/180*ones(size(x)));hold on;
% plot(x, 300*pi()/180*ones(size(x)));hold on;
% plot(x, 330*pi()/180*ones(size(x)));hold on;

% clear ch data
ch0=-inf*ones(size(x));
ch1=-inf*ones(size(x));
ch2=-inf*ones(size(x));
ch3=-inf*ones(size(x));

% and plot data
lhch0=line(x,ch0, 'Color','b');
lhch1=line(x,ch1, 'Color','g');
lhch2=line(x,ch2, 'Color','r');
lhch3=line(x,ch3, 'Color','c');

% setup a vertical plotter line
lb=line([inf,inf],[-0.1,6.5]); %-400,400
%lb=line([inf,inf],[-400,400]); %-400,400

legend('ch0', 'ch1', 'ch2', 'ch3');
%legend('correctedAcc(ch0)', 'current(ch1)', 'orignialAcc(ch2)', 'theta(ch3)');
%legend('\theta_{enc}[rad](ch0)','velocity_{enc}[rad/s](ch1)','Amp Input[V](ch2)','ch3');
%legend('\theta_{enc}(ch0)','\theta_{cam}(ch1)','error*10(ch2)','ch3');
shg;

% data gathering/plotting loop

stopRtPlot = 0;
while stopRtPlot~=1
    
     % clear data on wrap-around
     ch0=-inf*ones(size(x));
     ch1=-inf*ones(size(x));
     ch2=-inf*ones(size(x));
     ch3=-inf*ones(size(x));
     
     % screenfull of data loop
     display_data = 0; % for displaying data less frequently.
     for ix=1:plot_batch:np,

         % read data from the socket
         % here we're getting a group of 4 channels
         % plot_batch times
         try
             [data, cnt] = fread(t,4*plot_batch,'double');
         catch
             fprintf(1,'Error reading from socket\n');
             rtplot_stop;
             break;
         end
         
         if cnt ~= 4*plot_batch,
             fprintf(1,'Incomplete reading from socket\n');
             rtplot_stop;
             break;
         end
         
         % slice the data by channel number
         data = reshape(data,4,plot_batch);
         
         % and record it
         if( (ix+plot_batch-1) <= np)
             ch0(ix:ix+plot_batch-1)=data(1,:);
             ch1(ix:ix+plot_batch-1)=data(2,:);
             ch2(ix:ix+plot_batch-1)=data(3,:);
             ch3(ix:ix+plot_batch-1)=data(4,:);
             
             if ( mod(display_data, 4) == 0 ) % only the other time
                 % now set the plot data
                 set(lhch0,...
                     'ydata',ch0);
                 set(lhch1,...
                     'ydata',ch1);
                 set(lhch2,...
                     'ydata',ch2);
                 set(lhch3,...
                     'ydata',ch3);
                 
                 set(lb,...
                     'xdata',[ix+plot_batch-1,ix+plot_batch-1]);

                % refresh the plot
                drawnow;
             end
             display_data = display_data + 1;
         end
         
         % exit the loop if stopping
         if(stopRtPlot==1),
             break;
         end
     end
     
     if(oneshot==1)
         rtplot_stop;
         break;
     end
end