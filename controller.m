function [M,F] = controller(z,t,p)
    theta_target = p.theta_target; x_target = p.xdot_target;
    thetadot_target = p.thetadot_target; xdot_target = p.xdot_target;
    d = p.d; mp = p.mp; g = p.g; I = p.I; rated_torque = p.rated_torque;

    CONTROLLER = 2;
    
    b = p.b; mc = p.mc;

    x = z(1);
    xdot = z(2);
    theta = z(3);
    thetadot = z(4);

    % M = (I*0dot) - (mp*g*d*sind(0));

    
    M = 0;
    F = 0;

    A = [0,                                           1,                                                0, 0;
         0, -(b*mp*d^2 + b*I)/(mc*mp*d^2 + mc*I + mp*I),           (d^2*g*mp^2)/(mc*mp*d^2 + mc*I + mp*I), 0;
         0,                                           0,                                                0, 1;
         0,         -(b*d*mp)/(mc*mp*d^2 + mc*I + mp*I), (d*g*mp^2 + d*g*mc*mp)/(mc*mp*d^2 + mc*I + mp*I), 0];
    
    B = [                                     0,                                   0;
         (mp*d^2 + I)/(mc*mp*d^2 + mc*I + mp*I),    (d*mp)/(mc*mp*d^2 + mc*I + mp*I);
                                              0,                                   0;
              (d*mp)/(mc*mp*d^2 + mc*I + mp*I), (mc + mp)/(mc*mp*d^2 + mc*I + mp*I)];

    % Use 1 for pidCascade
    if CONTROLLER == 1
        Kp_F = 992;
        Ki_F = 1679;
        Kd_F = 146;

        Kp_M = 992;
        Ki_M = 1679;
        Kd_M = 146;

        F = Kp_F*(theta_target - theta) + Ki_F*(theta_target - theta) + Kd_F*(thetadot_target - thetadot);
        M = Kp_M*(theta_target - theta) + Ki_M*(theta_target - theta) + Kd_M*(thetadot_target - thetadot);
    elseif CONTROLLER == 2
        z_ref = [x_target; xdot_target; theta_target; thetadot_target];
        [M, F] = mylqr(z, z_ref, p);
    else
        F = 0;
        M = 0;
    end
end