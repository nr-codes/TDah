% clc

framerate = 250;
data = load('data4fixI.txt');
num_markers = (length(data(1,:))-1)/2;
n = length(data(:,1));

% 'a' is for memory the X and Y data for each marker;
a = zeros(n,2,num_markers);
for i = 1:num_markers
    j = i*2;
    a(:,:,i) = data(1:n,j-1:j);
end
% Right now t is acturally equal to frame number
t = data(1:n,num_markers*2+1);

d = zeros(n,num_markers);
for k = 1:num_markers
    for i = 1:n
        for j = 1:n
            temp = sqrt((a(i,1,k)-a(j,1,k))^2 + (a(i,2,k)-a(j,2,k))^2);
            if temp > d(i,k)
                d(i,k) = temp;
            end
        end
    end
end

cX = zeros(num_markers);cY = zeros(num_markers);r = zeros(num_markers);
for i = 1:num_markers
    cX(i) = (min(a(:,1,i)) + max(a(:,1,i)))/2;
    cY(i) = (min(a(:,2,i)) + max(a(:,2,i)))/2;
    r(i) = max(d(:,i))/2;
    
    str = ['#define r', num2str(i), ' ',num2str(max(d(:,i))/2)];
disp(str);
end


figure(1)
hold on
angle = 0:0.01:2*pi+0.01;
int = 1/num_markers;
for i = 1:num_markers
    plot(a(:,1,i),a(:,2,i),'linewidth',2,'color',[mod(i*0.3,1) mod((i-1)*0.6,1) mod((1-i*0.3),1)]);
    line(r(i)*cos(angle)+cX(i), r(i)*sin(angle)+cY(i), 'color','r','linewidth',1);
    line([cX(i) cX(i)],[cY(i) cY(i)],'Marker','o','linewidth',2, 'color',[mod(i*0.3,1) mod((i-1)*0.6,1) mod((1-i*0.3),1)]);
end
title('Position data (in mm)');
axis equal
grid on
legend('camera data','Min circle which can cover all the samples');
hold off

figure(2)
for i = 1:num_markers
    subplot(num_markers,1,i);
    plot(d(:,i), 'linewidth',2,'color',[mod(i*0.3,1) mod((i-1)*0.6,1) mod((1-i*0.3),1)]);
    title('Max distance between one point to all the other points (mm)');
    line([0 n], [max(d(:,i)) max(d(:,i))],'linewidth',2,'color','g');
    line([0 n], [min(d(:,i)) min(d(:,i))],'linewidth',2,'color','g');
    line([0 n], [mean(d(:,i)) mean(d(:,i))],'linewidth',2,'color','r');
    xlabel('sample #');
    tilting_angle = acos(min(d(:,i))/max(d(:,i)))/pi*180
end
% tilting_angle = acos(min(d)/max(d))/pi*180;

v = inf*ones(n-1, num_markers);
w = inf*ones(n-1, num_markers);
for k = 1:num_markers
    for i = 1:n-1
        if ((t(i+1) - t(i)) > 0.00)
%             v(i,k) = sqrt((a(i+1,1,k)-a(i,1,k))^2 + (a(i+1,2,k)-a(i,2,k))^2)/(t(i+1) - t(i));
            v(i,k) = sqrt((a(i+1,1,k)-a(i,1,k))^2 + (a(i+1,2,k)-a(i,2,k))^2)/(t(i+1) - t(i))*framerate;
        else
            v(i,k) = v(i-1,k);
        end
        
%         v(i,k) = sqrt((a(i+1,1,k)-a(i,1,k))^2 + (a(i+1,2,k)-a(i,2,k))^2)/(1/framerate);
        w(i,k) = v(i,k)/d(i,k)*2;
    end
end
w_ave = zeros(n-1,1);
for i = 1:n-1
    w_ave(i,1) = mean(w(i,:));
end


% figure(3)
% plot(v);
figure(4)
w_mean = zeros(num_markers,1);w_std = zeros(num_markers,1);
w_max = zeros(num_markers,1); w_min = zeros(num_markers,1);
title('anguler velocity by camera');
for i = 1:num_markers
    subplot(num_markers+1,1,i);
    plot(t(1:n-1), w(:,i));
    w_mean(i) = mean(w(:,i));
    w_std(i) = std(w(:,i));
    line([t(1) t(n-1)], [w_mean(i) w_mean(i)],'linewidth',2,'color','r');
    w_max(i) = max(w(:,i));
    line([t(1) t(n-1)], [w_max(i) w_max(i)],'linewidth',2,'color','g');
    w_min(i) = min(w(:,i));
    line([t(1) t(n-1)], [w_min(i) w_min(i)],'linewidth',2,'color','g');
    xlabel('frame # (sample rate = 250Hz)'); ylabel('in rad/sec');
end
w_mean
w_std

subplot(num_markers+1,1,num_markers+1);
plot(t(1:n-1), w_ave);
title('The average anguler velocity by camera');
w_ave_mean = mean(w_ave)
w_ave_std = std(w_ave)
line([t(1) t(n-1)], [w_ave_mean w_ave_mean],'linewidth',2,'color','r');
w_ave_max = max(w_ave);
line([t(1) t(n-1)], [w_ave_max w_ave_max],'linewidth',2,'color','g');
w_ave_min = min(w_ave);
line([t(1) t(n-1)], [w_ave_min w_ave_min],'linewidth',2,'color','g');
xlabel('frame # (sample rate = 250Hz)'); ylabel('in rad/sec');

axis tight


% figure(5)
% % subplot(2,1,2);
% plot(ch1);
% ch1_mean = mean(ch1)
% ch1_std = std(ch1)
% line([0 length(ch1)], [ch1_mean ch1_mean],'linewidth',2,'color','r');
% ch1_max = max(ch1);
% line([0 length(ch1)], [ch1_max ch1_max],'linewidth',2,'color','g');
% ch1_min = min(ch1);
% line([0 length(ch1)], [ch1_min ch1_min],'linewidth',2,'color','g');
% title('anguler velocity by encoder');
% xlabel('sample (sample rate = 1000Hz)'); ylabel('in rad/sec');

display('--------------------');