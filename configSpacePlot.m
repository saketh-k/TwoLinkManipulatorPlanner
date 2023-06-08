function [C, th1, th2] = configSpacePlot(len1, len2, obs)
    res = 50; % Resolution of the configuration space
    num_points = 10; % Number of points to sample along each link
    th1 = linspace(0, 2*pi, res); % All possible theta1 values
    th2 = linspace(0, 2*pi, res); % All possible theta2 values

    % Initialize configuration space to zeros
    C = zeros(res, res);

    % For each theta1 value
    for i = 1:res
        % For each theta2 value
        for j = 1:res
            % Initialize collision flag to false
            collision = false;

            % For each point on link 1
            for k = 0:(1/num_points):1
                % Calculate position
                x = len1*k*cos(th1(i));
                y = len1*k*sin(th1(i));

                % Check for collision with each obstacle
                for m = 1:size(obs, 1)
                    if x > obs(m,1) && x < obs(m,1)+obs(m,3) && y > obs(m,2) && y < obs(m,2)+obs(m,4)
                        collision = true;
                        break;
                    end
                end

                if collision, break; end
            end

            % For each point on link 2
            if ~collision
                for k = 0:(1/num_points):1
                    % Calculate position
                    x = len1*cos(th1(i)) + len2*k*cos(th1(i)+th2(j));
                    y = len1*sin(th1(i)) + len2*k*sin(th1(i)+th2(j));

                    % Check for collision with each obstacle
                    for m = 1:size(obs, 1)
                        if (x > obs(m,1) && x < obs(m,1)+obs(m,3)) && (y > obs(m,2) && y < obs(m,2)+obs(m,4))
                            collision = true;
                            break;
                        end
                    end

                    if collision, break; end
                end
            end

            % If a collision was detected, set C(i,j) to 1
            if collision
                C(i,j) = 1;
            end
        end
    end

    % Plot configuration space
    [Th1, Th2] = meshgrid(th1, th2);
    figure;
    contourf(Th1, Th2, C');
    % colorbar;
    xlabel('\theta_1');
    ylabel('\theta_2');
    title('Configuration Space');
end