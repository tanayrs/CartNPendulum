function p = params()
    % Define Parameters
    p.mp = 4; p.mc = 2; % in kg
    p.d = 1; % in m
    p.g = 10; % in m/s^2
    p.theta_target = -deg2rad(5); % in rad
    % p.theta_target = 0.1; % in rad
    p.thetadot_target = 0; % in rad/s
    p.x_target = 0; % in m/s
    p.xdot_target = 0; % in m/s
    p.time_scale = 10;
    p.k = 1; % The switching strength for linearized controllers
    p.rated_torque = 100*p.g/100; % Rated Torque of Pendulum Motor
    p.rated_torque_cart = 6*p.g/100; % Rated Torque of Cart Motor
    Ig = 0; % Mass Moment of Inertia of Mass about mass
    p.I = (p.mp * (p.d ^ 2)) + Ig; % Mass Moment of Intertia about G
    p.b = 1; % Coefficient of Motor
end
