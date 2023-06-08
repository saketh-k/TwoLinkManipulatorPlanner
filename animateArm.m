function animateArm(len1, len2, thetas, obs)
    figure;

    % Get the number of frames from the length of theta_mat
    num_frames = size(thetas, 1);
    
    while true
        % Iterate over all the frames
        for k = 1:num_frames
            % Clear current figure
            clf;
            
            % Draw the arm at the current theta values
            drawArmAndObstacles(len1, len2, thetas(k, 1), thetas(k, 2), obs);
            pause(0.05);
        end
    end
end
