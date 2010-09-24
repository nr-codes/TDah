REC_OFFSET = 2;
NUM_REC = 1000;
NEW_REC = REC_OFFSET + NUM_REC;
NOT_SW_TRIG = -1;

d = importdata('chauncy-esque051910.txt');
n = length(d);

%%
x = zeros(n / NEW_REC, 1);
y_fg = zeros(n / NEW_REC, 1);
y_pc = zeros(n / NEW_REC, 1);
w = zeros(n / NEW_REC, 1);
h = zeros(n / NEW_REC, 1);

dropped = zeros(n / NEW_REC, 1);

y_mo = zeros(n / NEW_REC, 1);
A = zeros(n / NEW_REC, 5);

j = 1;
for i = 1:NEW_REC:n
    sw_trig = d(i+3, 4);
    exp = d(i, 3);
    w(j) = d(i, 1);
    h(j) = d(i, 2);
    cps = d(i+1, 1);
    
    if sw_trig == NOT_SW_TRIG
        sprintf('images in record %d was not non-software triggered', i);
    end

    x(j) = d(i,1)*d(i,2);
    y_fg(j) = mean(diff(d(i+REC_OFFSET:i+NUM_REC+1, 4)));
    y_pc(j) = mean(d(i+REC_OFFSET:i+NUM_REC+1, 3) - ...
        d(i+REC_OFFSET:i+NUM_REC+1, 2)) / cps * 1e6;
    
    dropped(j) = d(i+NUM_REC+1, 1) - NUM_REC;

    A(j,:) = [exp, w(j), h(j), x(j), 1];
    
    if w(j) < 528
        y_mo(j) = exp + (h(j) + 1)*(.2 + w(j)/40/2) + .2;
    else
        y_mo(j) = exp + h(j)*(.2 + w(j)/40/2) + .2;
    end
    
    j = j + 1;
end

% plot(x, y_fg, '*', x, y_pc, '*');
% xlabel('image size (width x height)')
% ylabel('acquisition time (us)')
% 

%%
j = 1;
for i = 1:NEW_REC:n
    sw_trig = d(i+3, 4);
    exp = d(i, 3);
    w(j) = d(i, 1);
    h(j) = d(i, 2);
    cps = d(i+1, 1);
    
    if sw_trig == NOT_SW_TRIG
        sprintf('images in record %d was not non-software triggered', i);
    end
   
    img_nr = d(i+REC_OFFSET:i+NUM_REC+1, 1);
    for k = 1:NUM_REC
        if img_nr(k) > k
           fprintf('%d image lost in run %d %dx%d @ %f, expected %d got %d\n',...
               img_nr(k) - k, i, w(j), h(j), exp, k, img_nr(k));
        elseif img_nr(k) < k
            fprintf('same image in run %d %dx%d @ %f, expected %d got %d\n', ...
               i, w(j), h(j), exp, k, img_nr(k));
        end
    end
    j = j + 1;
end