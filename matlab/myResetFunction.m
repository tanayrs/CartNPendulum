function [InitialObservation, InitialState] = myResetFunction()
    % Reset function to place custom cart-pole environment into a random
    % initial state.
    
    % Theta (randomize)
    T0 = 2 * 0.05 * rand() - 0.05;
    % Thetadot
    Td0 = 0;
    % X
    X0 = 0;
    % Xdot
    Xd0 = 0;
    
    % Return initial environment state variables as logged signals.
    InitialState = [X0;Xd0;T0;Td0];
    InitialObservation = InitialState;

end