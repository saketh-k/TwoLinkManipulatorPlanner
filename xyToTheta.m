function [theta1, theta2] = xyToTheta(point, len1, len2)
    x = point(1)
    y = point(2)
    % elbow down
     theta2 = pi - acos( (len1^2 + len2^2 - x^2 - y^2) / (2*len1*len2))
    if imag(theta2) > 0
    % elbow up 
        theta2 = -pi + acos( (len1^2 + len2^2 - x^2 - y^2) / (2*len1*len2))
    end
    
    t1num = len2*sin(theta2)
    t1denom = len1+(len2*cos(theta2))
    theta1 = atan2(y,x)-atan2(t1num,t1denom);
end