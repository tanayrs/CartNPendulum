# CartNPendulum
Segway for RO3002: Reinforcement Learning

LAST STABLE VERSION BEFORE CLEANUP
​
Segway Params:
    
    p.m = 0.841; % in kg
    
    p.d = 0.2; % in m
    
    p.dw = 0.075; % in m
    
    p.R_wheel = 0.05; % in m
    
    p.g = 10; % in m/s^2 
    
    p.time_scale = 0.1;
    
    p.rated_torque = 25*p.g/100; % Rated Torque of Pendulum Motor
    
    Ig = 0; % Mass Moment of Inertia of Mass about mass
    
    p.I = (p.m * (p.d ^ 2)) + Ig; % Mass Moment of Intertia about G
    
    p.b = 0.4; % Viscous Drag
