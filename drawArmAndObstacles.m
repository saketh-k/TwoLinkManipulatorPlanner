function drawArmAndObstacles(len1, len2, theta1, theta2, obs, labelloc)

    % Calculate position of joint and end effector
    joint = [len1*cos(theta1) len1*sin(theta1)];
    end_effector = [joint(1) + len2*cos(theta1+theta2), joint(2) + len2*sin(theta1+theta2)];

    % Draw links
    plot([0, joint(1)], [0, joint(2)], 'k', 'LineWidth', 2);
    hold on;
    plot([joint(1), end_effector(1)], [joint(2), end_effector(2)], 'k', 'LineWidth', 2);
    
    % Draw joint and end effector
    plot(0, 0, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');  % Base
    plot(joint(1), joint(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');  % Joint
    plot(end_effector(1), end_effector(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');  % End effector

    %draw labels
    scatter(labelloc(:,1),labelloc(:,2),'x');

    % Draw obstacles
    for i = 1:size(obs, 1)
        rectangle('Position', obs(i,:), 'FaceColor', [0.5 0.5 0.5]);
    end

    % Set limits and labels
    xlim([-2, 2]);
    ylim([-2, 2]);
    xlabel('X');
    ylabel('Y');
    title('Robot Arm and Obstacles');
    grid on;
    hold off;
end
