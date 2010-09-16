clf;

markersize = 10;
x = 0:1:3;
y = 0:1:2;
[X,Y] = meshgrid(x, y);

% bounding box
eps = .3;
xb = x(1) - eps;
yb = y(1) - eps;
w = x(end) - x(1) + 2*eps;
h = y(end) - y(1) + 2*eps;
box = [xb yb w h];

hold on
plot(X, Y, 'ok', 'MarkerFaceColor', 'k', 'MarkerSize', markersize);
plot(0, 0, 'ow', 'MarkerFaceColor', 'w', 'MarkerSize', markersize/6);
plot(xb, yb, xb + w, yb + h);
rectangle('Position', box, 'EdgeColor', 'k', 'LineWidth', 15);
hold off
axis image
axis off