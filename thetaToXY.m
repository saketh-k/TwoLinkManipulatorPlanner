function [x, y] = thetaToXY(thetas,len1, len2)
theta1 = thetas(1);
theta2 = thetas(2);
x = len1*cos(theta1) + len2*cos(theta1+theta2);
y = len1*sin(theta1) + len2*sin(theta1+theta2);
end

