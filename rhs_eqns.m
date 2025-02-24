clc;
clear all;

syms x xdot theta thetadot F M g mp mc d I b xddot thetaddot real;
eqn1 = F == (mc*xddot) + (mp*(xddot+(d*thetadot*thetadot*sin(theta)-(d*thetaddot*cos(theta))))) + b*xdot;
eqn2 = M == (thetaddot*(I+(mp*d*d)))-(mp*d*((xddot*cos(theta)+(g*sin(theta)))));

[A,B] = equationsToMatrix([eqn1,eqn2],[xddot,thetaddot]);
X = linsolve(A,B);
X(1);
X(2);

i = [1; 0; 0];
j = [0; 1; 0];
k = [0; 0; 1];
er = [-sin(theta); cos(theta); 0];
et = [-cos(theta); -sin(theta); 0];

arel = (xddot*i + et*d*thetaddot - er*d*thetadot^2);
Ldot_tot = mc*xddot + mp*arel + b*xdot;
Hdot_pend = I*thetaddot*k + cross(d*er, mp*arel);

eqn1 = dot(Ldot_tot,i);
eqn2  = dot(Hdot_pend,k) - dot(cross(d*er, mp*g*(-j)),k);

eqn1 = F == simplify(eqn1);
eqn2 = M == simplify(eqn2);

eqns = [eqn1, eqn2];
variables = [xddot, thetaddot];
[xddot, thetaddot] = solve(eqns, variables);

xddot = simplify(xddot);
thetaddot = simplify(thetaddot);

zdot(1) = xdot;
zdot(2) = xddot;
zdot(3) = thetadot;
zdot(4) = thetaddot;

% Define symbolic variables


% Define unit vectors
i = [1; 0; 0];
j = [0; 1; 0];
k = [0; 0; 1];

% Define position-dependent vectors
er = [-sin(theta); cos(theta); 0];
et = [-cos(theta); -sin(theta); 0];

% Relative acceleration
arel = (xddot*i + et*d*thetaddot - er*d*thetadot^2);

% Linear momentum derivative
Ldot_tot = mc*xddot + mp*arel + b*xdot;

% Angular momentum derivative
Hdot_pend = I*thetaddot*k + cross(d*er, mp*arel);

% Equations of motion
eqn1 = dot(Ldot_tot,i);
eqn2 = dot(Hdot_pend,k) - dot(cross(d*er, mp*g*(-j)),k);

% Create equations in standard form (moved F and M terms)
eq1 = collect(eqn1 - F, [xddot, thetaddot]);
eq2 = collect(eqn2 - M, [xddot, thetaddot]);

% State equations (after solving for xddot and thetaddot)
zdot = [xdot;
        xddot;
        thetadot;
        thetaddot];

% Define state and input vectors
z = [x; xdot; theta; thetadot];
u = [F; M];

% Calculate Jacobians for linearization around equilibrium point
% Note: Need to substitute the solved xddot and thetaddot first
A = jacobian(zdot, z);
B = jacobian(zdot, u);

% Equilibrium point (upward position)
z_eq = [0; 0; 0; 0];
u_eq = [0; 0];

% Substitute equilibrium point
A_linear = subs(A, [z', u'], [z_eq', u_eq'])
B_linear = subs(B, [z', u'], [z_eq', u_eq'])

% To get numerical matrices:
% A_num = double(subs(A_linear, {g, mp, mc, d, I, b}, {9.81, mp_val, mc_val, d_val, I_val, b_val}));
% B_num = double(subs(B_linear, {g, mp, mc, d, I, b}, {9.81, mp_val, mc_val, d_val, I_val, b_val}));