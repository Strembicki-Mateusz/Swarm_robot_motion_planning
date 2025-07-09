function dis = distance_a(node, xy)
% distance_a - function which calculate Manchattan value of the next step in
% grid for robot to reach target.

x_node = node(1);
y_node = node(2);
x = xy(1);
y = xy(2);

dis = sqrt((x_node - x)^2 + abs(y_node-y)^2);
%dis = abs(x_node - x) + abs(y_node-y);
end