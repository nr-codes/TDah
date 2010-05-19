% NEW_REC = 102;
NEW_REC = 1002;
REC_OFFSET = 2;
NUM_REC = 1000;
NOT_SW_TRIG = -1;

% d = importdata('fixed-chauncy-esque_051410.txt');
d = importdata('chauncy-esque051910.txt');

n = length(d);
x = zeros(n / NEW_REC, 1);
y = zeros(n / NEW_REC, 1);
w = zeros(n / NEW_REC, 1);
h = zeros(n / NEW_REC, 1);

y_pc = zeros(n / NEW_REC, 1);

j = 1;
for i = 1:NEW_REC:n
    sw_trig = d(i+3, 4);
    exp = d(i, 3);
    w(j) = d(i, 1);
    h(j) = d(i, 2);
    cps = d(i+1, 1);
    
    if sw_trig == NOT_SW_TRIG
        x(j) = d(i,1)*d(i,2);
        y(j) = mean(diff(d(i+REC_OFFSET:i+NUM_REC+1, 3)));
        y_pc(j) = mean(diff(d(i+REC_OFFSET:i+NUM_REC+1, 2))) / cps * 1e6;
    else
        x(j) = d(i,1)*d(i,2);
        y(j) = mean(diff(d(i+REC_OFFSET:i+NUM_REC+1, 4)));
        y_pc(j) = mean(d(i+REC_OFFSET:i+NUM_REC+1, 3) - ...
            d(i+REC_OFFSET:i+NUM_REC+1, 2)) / cps * 1e6;
    end
    j = j + 1;
end

% plot(x(1:2:end), y(1:2:end), '*',x(1:2:end), y_pc(1:2:end), '*');
% plot(x(2:2:end), y(2:2:end), '*',x(2:2:end), y_pc(2:2:end), '*');
plot(x(2:2:end), y(2:2:end), '*');
xlabel('image size (width x height)')
ylabel('acquisition time (us)')
% 
% plot(x(1:2:end), -y(1:2:end) + y_pc(1:2:end), '*');
% xlabel('image size (width x height)')
% ylabel('acquisition time difference pc - fg (us)')