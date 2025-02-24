function [NextObs,Reward,IsDone,NextState] = myStepFunction(Action,State)
    % Custom step function to construct cart-pole environment for the function
    % name case.
    %
    % This function applies the given action to the environment and evaluates
    % the system dynamics for one simulation step.
    
    % Define the environment constants.

    p = params();
    mp = p.mp; mc = p.mc; d = p.d; g = p.g; I = p.I; b= p.b;

    % Sample time
    Ts = 0.02;
    
    % Pole angle at which to fail the episode
    AngleThreshold = 12 * pi/180;
    
    % Cart distance at which to fail the episode
    DisplacementThreshold = 2.4;
    
    % Reward each time step the cart-pole is balanced
    RewardForNotFalling = 1;
    
    % Penalty when the cart-pole fails to balance
    PenaltyForFalling = -10;
    
    % Check if the given action is valid.
    % if ~ismember(Action,[-MaxForce MaxForce])
    %     error('Action must be %g for going left and %g for going right.',...
    %         -MaxForce,MaxForce);
    % end

    M = Action(1);
    F = Action(2);
    
    % Unpack the state vector from the logged signals.
    xdot = State(2);
    theta = State(3);
    thetadot = State(4);
    
    % Apply motion equations.

    xddot = (I*F + d^2*mp*F - I*b*xdot - b*d^2*mp*xdot - d^3*mp^2*thetadot^2*sin(theta) ...
             + d*mp*M*cos(theta) + d^2*g*mp^2*cos(theta)*sin(theta) - I*d*mp*thetadot^2*sin(theta)) ...
            / (I*mc + I*mp + d^2*mp^2 - d^2*mp^2*cos(theta)^2 + d^2*mc*mp);

    thetaddot = (mc*M + mp*M + d*mp*F*cos(theta) + d*g*mp^2*sin(theta) - d^2*mp^2*thetadot^2*cos(theta)*sin(theta) ...
                 - b*d*mp*xdot*cos(theta) + d*g*mc*mp*sin(theta)) ...
                / (I*mc + I*mp + d^2*mp^2 - d^2*mp^2*cos(theta)^2 + d^2*mc*mp);

    
    % Perform Euler integration to calculate next state.
    NextState = State + Ts.*[xdot;xddot;thetadot;thetaddot];
    
    % Copy next state to next observation.
    NextObs = NextState;
    
    % Check terminal condition.
    X = NextObs(1);
    Theta = NextObs(3);
    IsDone = abs(X) > DisplacementThreshold || abs(Theta) > AngleThreshold;
    
    % Calculate reward.
    if ~IsDone
        Reward = RewardForNotFalling;
    else
        Reward = PenaltyForFalling;
    end

end