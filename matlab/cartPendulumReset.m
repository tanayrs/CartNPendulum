%% Reset Function for Environment
function [initialState] = cartPendulumReset()
    % Set initial state [x, xdot, theta, thetadot]
    initialState = [0; 0; pi + 0.1*randn; 0]; % Small perturbation
end