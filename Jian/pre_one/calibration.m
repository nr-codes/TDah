% clc
% 

f = load('camera_frame.txt');
w = load('world_frame.txt');

f = f';
w = w';

a = f*w'/(w*w');
b = f/w;
% a_inv = inv(a'*a)*a';
a_inv = inv(a);
% aa*f
str = ['#define a11 ', num2str(aa(1,1))];
disp(str);
str = ['#define a12 ', num2str(aa(1,2))];
disp(str);
str = ['#define a13 ', num2str(aa(1,3))];
disp(str);
str = ['#define a21 ', num2str(aa(2,1))];
disp(str);
str = ['#define a22 ', num2str(aa(2,2))];
disp(str);
str = ['#define a23 ', num2str(aa(2,3))];
disp(str);disp(' ');

% str = ['#define a11 ', num2str(aa(1,1))];
% disp(str);
% str = ['#define a12 ', num2str(aa(1,2))];
% disp(str);
% str = ['#define a13 ', num2str(aa(1,3))];
% disp(str);
% str = ['#define a14 ', num2str(aa(1,4))];
% disp(str);
% str = ['#define a15 ', num2str(aa(1,5))];
% disp(str);
% str = ['#define a16 ', num2str(aa(1,6))];
% disp(str);
% str = ['#define a21 ', num2str(aa(2,1))];
% disp(str);
% str = ['#define a22 ', num2str(aa(2,2))];
% disp(str);
% str = ['#define a23 ', num2str(aa(2,3))];
% disp(str);
% str = ['#define a24 ', num2str(aa(2,4))];
% disp(str);
% str = ['#define a25 ', num2str(aa(2,5))];
% disp(str);
% str = ['#define a26 ', num2str(aa(2,6))];
% disp(str);disp(' ');