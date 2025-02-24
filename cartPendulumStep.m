%% Step Function for Environment
function [nextState, reward, isDone] = cartPendulumStep(state, action)
    % Extract control inputs
    M = action(1);
    F = action(2);
    
    % Define simulation parameters
    dt = 0.01; % Time step
    p = struct('mp', 1.0, 'mc', 5.0, 'd', 0.5, 'g', 9.81, 'I', 0.1, ...
               'rated_torque', 10, 'b', 0.1, 'rated_torque_cart', 10);
    
    % Compute next state using ODE solver
    [~, z_next] = ode45(@(t, z) myrhs(z, t, p, [M; F]), [0 dt], state);
    nextState = z_next(end, :)'; % Take final state from integration
    
    % Reward function: Encourage upright position
    theta = nextState(3);
    reward = -abs(mod(theta, 2*pi) - pi); % Negative reward based on deviation from upright
    
    % Check termination condition
    isDone = abs(mod(theta, 2*pi) - pi) > pi/2; % Episode ends if pendulum falls too far
end