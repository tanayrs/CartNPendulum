function retval = animate(solution, z0, tspan, p, move_frame)
    d = p.d;
    x0 = z0(1);
    theta0 = z0(3);
    
    x_vals = solution.y;

    % Initialize figure
    figure;
    shg;
    hold on;
    grid on;

    % Set dynamic axis limits based on the trajectory
    if move_frame == 0
        x_min = min(x_vals(1, :)) - 0.5;
        x_max = max(x_vals(1, :)) + 0.5;
        
        axis equal;
        xlim([x_min, x_max]);
        ylim([-1.5*d,1.5*d]);
    elseif move_frame == 42
        axis equal;
        xlim([-10,10]);
    else
        ylim([-1.5*d,1.5*d]);
        axis equal;
    end


    % Initialize plot objects
    cart = plot([x0-d/4, x0+d/4, x0+d/4, x0-d/4, x0-d/4], [0,0,-d/8,-d/8,0], 'w', 'LineWidth', 2);
    pendulum = plot([x0, x0 + d*sin(-theta0)], [0, d*cos(theta0)], 'b', 'LineWidth', 2);

    % Initialize trajectory plots with initial point
    % trajectory_cart = plot(x0, 'r.');
    % trajectory_pendulum = plot(x0 + d*sin(-theta0), d*cos(theta0), 'b.');

    % Total animation duration
    total_time = tspan(2) - tspan(1);

    % Animation loop using tic-toc
    t_start = tic; % Start timer
    while true
        % Elapsed time since animation started, adjusted by time scale
        elapsed_time = toc(t_start) * p.time_scale;

        % Check if animation is complete
        if elapsed_time + 1e-1 >= total_time
            break;
        end

        % Interpolate the solution at the current time step
        z_current = deval(solution, elapsed_time);

        % Update mass position
        set(cart, 'XData', [z_current(1)-d/4, z_current(1)+d/4, z_current(1)+d/4, z_current(1)-d/4, z_current(1)-d/4], ...
                  'YData', [0, 0, -d/8, -d/8, 0]);
        set(pendulum, 'XData', [z_current(1), z_current(1) + d*sin(-z_current(3))], ...
                     'YData', [0, d*cos(z_current(3))]);



        % Safely update trajectory
        % current_cart_x = get(trajectory_cart, 'XData');
        % current_cart_y = get(trajectory_cart, 'YData');
        % set(trajectory_cart, 'XData', [current_cart_x, z_current(1)], ...
        %                      'YData', [current_cart_y, 0]);
        % 
        % current_pendulum_x = get(trajectory_pendulum, 'XData');
        % current_pendulum_y = get(trajectory_pendulum, 'YData');
        % set(trajectory_pendulum, 'XData', [current_pendulum_x, z_current(1) + d*sin(-z_current(3))], ...
        %                          'YData', [current_pendulum_y, d*cos(z_current(3))]);

        % Pause to create animation effect
        drawnow
    end

    hold off;
end
