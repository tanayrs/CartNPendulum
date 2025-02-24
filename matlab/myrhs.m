%% Dynamics Function
function zdot = myrhs(z,t,p,u)
    % State variables: [x, xdot, theta, thetadot]
    x = z(1);
    xdot = z(2);
    theta = z(3);
    thetadot = z(4);

    % Unpacking Parameters
    mp = p.mp; mc = p.mc; d = p.d; g = p.g; I = p.I; 
    rated_torque = p.rated_torque; b = p.b; 
    rated_torque_cart = p.rated_torque_cart;

    M = u(1);
    F = u(2);

    % Clip Control Inputs
    M = max(min(M, rated_torque), -rated_torque);
    F = max(min(F, rated_torque_cart), -rated_torque_cart);

    % Defining Equations of Motion
    xddot = (I*F + d^2*mp*F - I*b*xdot - b*d^2*mp*xdot - d^3*mp^2*thetadot^2*sin(theta) ...
             + d*mp*M*cos(theta) + d^2*g*mp^2*cos(theta)*sin(theta) - I*d*mp*thetadot^2*sin(theta)) ...
            / (I*mc + I*mp + d^2*mp^2 - d^2*mp^2*cos(theta)^2 + d^2*mc*mp);

    thetaddot = (mc*M + mp*M + d*mp*F*cos(theta) + d*g*mp^2*sin(theta) - d^2*mp^2*thetadot^2*cos(theta)*sin(theta) ...
                 - b*d*mp*xdot*cos(theta) + d*g*mc*mp*sin(theta)) ...
                / (I*mc + I*mp + d^2*mp^2 - d^2*mp^2*cos(theta)^2 + d^2*mc*mp);

    % State Derivatives
    zdot(1) = z(2);
    zdot(2) = xddot;
    zdot(3) = z(4);
    zdot(4) = thetaddot;
    
    zdot = zdot';
end