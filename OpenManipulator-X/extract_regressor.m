%% extract_regressor.m
% Run this script in the OpenManipulator-X-main folder.
% It computes the regressor matrix Y and parameter vector pi such that:
%   tau = Y(q, q_dot, q_ddot, p_geom) * pi
% where pi = [m1 m2 m3 m4 Ixx1 Iyy1 Izz1 ... Ixx4 Iyy4 Izz4]' (16x1)
% and p_geom = [a0 d1 a1 d2 a2 ell_e g]' (7x1) are known geometric params.
%
% Outputs: generated_dynamics/Y_fun.m

clc; clear; close all;
disp('Starting regressor extraction...');
disp('This may take a few minutes due to symbolic simplification.');

%% Generalized coordinates (same as dynamics_manipulator_complete.m)
syms q1 q2 q3 q4 real
syms q1p q2p q3p q4p real
syms q1pp q2pp q3pp q4pp real
q   = [q1; q2; q3; q4];
qp  = [q1p; q2p; q3p; q4p];
qpp = [q1pp; q2pp; q3pp; q4pp];

%% Physical parameters
syms m1 m2 m3 m4 g real
syms Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 Ixx4 Iyy4 Izz4 real
syms a0 d1 a1 d2 a2 ell_e real

%% Forward kinematics (same as dynamics_manipulator_complete.m)
T_base = trans_xyz(a0, 0, d1);
A01 = dh_axis(q1, 0,   0,    0, 'z');
A12 = dh_axis(q2, d2,  a1,   0, 'y');
A23 = dh_axis(q3, 0,   a2,   0, 'y');
A34 = dh_axis(q4, 0, ell_e,  0, 'y');

T01 = simplify(T_base*A01);
T02 = simplify(T01*A12);
T03 = simplify(T02*A23);
T04 = simplify(T03*A34);

%% Link CoM positions
T0c1 = simplify(T01*dh_axis(q2, d2/2, a1/2, 0, 'y'));
T0c2 = simplify(T02*dh_axis(q3, 0,    a2/2, 0, 'y'));
T0c3 = simplify(T03*dh_axis(q4, 0,  ell_e/2, 0, 'y'));
T0c4 = T04;

p_c1 = simplify(T0c1(1:3,4));
p_c2 = simplify(T0c2(1:3,4));
p_c3 = simplify(T0c3(1:3,4));
p_c4 = simplify(T0c4(1:3,4));

links = cat(3, p_c1, p_c2, p_c3, p_c4);
mass_total = [m1; m2; m3; m4];

%% Jacobians
Jv1 = jacobian(p_c1, q);
Jv2 = jacobian(p_c2, q);
Jv3 = jacobian(p_c3, q);
Jv4 = jacobian(p_c4, q);
J_t = cat(3, Jv1, Jv2, Jv3, Jv4);

R01 = T01(1:3,1:3);
R02 = T02(1:3,1:3);
R03 = T03(1:3,1:3);

z0 = [0;0;1];
y1 = simplify(R01*[0;1;0]);
y2 = simplify(R02*[0;1;0]);
y3 = simplify(R03*[0;1;0]);

Jw1 = [z0, y1, sym(zeros(3,1)), sym(zeros(3,1))];
Jw2 = [z0, y1, y2,               sym(zeros(3,1))];
Jw3 = [z0, y1, y2,               y3];
Jw4 = [z0, y1, y2,               y3];
J_r = cat(3, Jw1, Jw2, Jw3, Jw4);

I1 = diag([Ixx1, Iyy1, Izz1]);
I2 = diag([Ixx2, Iyy2, Izz2]);
I3 = diag([Ixx3, Iyy3, Izz3]);
I4 = diag([Ixx4, Iyy4, Izz4]);
inertia_total = cat(3, I1, I2, I3, I4);

R0c1 = T0c1(1:3,1:3);
R0c2 = T0c2(1:3,1:3);
R0c3 = T0c3(1:3,1:3);
R0c4 = T0c4(1:3,1:3);
rotationals = cat(3, R0c1, R0c2, R0c3, R0c4);

%% Build symbolic M, C, G
disp('Computing M...');
M_full = simplify(M_matrix(J_t, J_r, mass_total, inertia_total, rotationals, q));
disp('Computing C...');
C_full = simplify(C_matrix(M_full, q, qp));
disp('Computing G...');
g_vec = [0; 0; g];
G_full = simplify(G_matrix(links, mass_total, q, g_vec));

%% Form the full torque expression: tau = M*qpp + C*qp + G
disp('Computing tau = M*qpp + C*qp + G...');
tau_sym = simplify(M_full * qpp + C_full * qp + G_full);

%% Define the parameter vector pi (16 unknown physical parameters)
pi_vec = [m1; m2; m3; m4; ...
          Ixx1; Iyy1; Izz1; ...
          Ixx2; Iyy2; Izz2; ...
          Ixx3; Iyy3; Izz3; ...
          Ixx4; Iyy4; Izz4];

%% Extract regressor: Y = d(tau)/d(pi)
% Since tau is linear in pi, tau = Y * pi exactly.
disp('Extracting regressor Y = jacobian(tau, pi)...');
Y_full = simplify(jacobian(tau_sym, pi_vec));

%% Verify: Y * pi should equal tau
disp('Verifying Y * pi == tau...');
verification = simplify(Y_full * pi_vec - tau_sym);
if all(verification == 0)
    disp('PASSED: Y * pi = tau');
else
    disp('WARNING: verification failed, check manually');
    disp(verification);
end

%% Display dimensions
fprintf('Y is %d x %d (joints x parameters)\n', size(Y_full, 1), size(Y_full, 2));
fprintf('pi is %d x 1\n', length(pi_vec));

%% Known geometric parameters
p_geom = [a0; d1; a1; d2; a2; ell_e; g];

%% Generate Y_fun.m
% Y_fun takes: q (4x1), q_dot (4x1), q_ddot (4x1), p_geom (7x1)
% Returns: Y (4x16)
disp('Generating Y_fun.m...');
matlabFunction(Y_full, 'File', 'generated_dynamics/Y_fun', ...
    'Vars', {q, qp, qpp, p_geom}, 'Optimize', true);

disp('Done! Generated: generated_dynamics/Y_fun.m');
disp('');
disp('Parameter vector pi ordering:');
disp('[m1 m2 m3 m4 Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 Ixx4 Iyy4 Izz4]');

%% ========== Helper functions (same as dynamics_manipulator_complete.m) ==========

function A = dh_axis(theta, d, a, alpha, axis_name)
    if axis_name == 'z'
        Rtheta = rotz_h(theta);
    elseif axis_name == 'y'
        Rtheta = roty_h(theta);
    else
        error('axis_name must be ''z'' or ''y''.');
    end
    A = Rtheta * trans_xyz(0, 0, d) * trans_xyz(a, 0, 0) * rotx_h(alpha);
end

function T = trans_xyz(x, y, z)
    T = [1 0 0 x;
         0 1 0 y;
         0 0 1 z;
         0 0 0 1];
end

function R = rotx_h(a)
    R = [1 0       0      0;
         0 cos(a) -sin(a) 0;
         0 sin(a)  cos(a) 0;
         0 0       0      1];
end

function R = roty_h(a)
    R = [ cos(a) 0 sin(a) 0;
          0      1 0      0;
         -sin(a) 0 cos(a) 0;
          0      0 0      1];
end

function R = rotz_h(a)
    R = [cos(a) -sin(a) 0 0;
         sin(a)  cos(a) 0 0;
         0       0      1 0;
         0       0      0 1];
end
