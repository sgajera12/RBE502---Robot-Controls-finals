function [tau, pi_hat_dot] = adaptive_controller(q, q_dot, ...
                                q_d, q_dot_d, q_ddot_d, ...
                                pi_hat, Kp, Kv, P, R_inv, p_geom)
% adaptive_controller  Adaptive computed torque control (Lecture 24).
%
% Control law:
%   tau = M_hat(q) * aq + C_hat(q,q_dot)*q_dot + G_hat(q)
%   where aq = q_ddot_d + Kv*e_dot + Kp*e
%
% Adaptation law (L24 Slide 4):
%   pi_hat_dot = R_inv * Y' * M_hat^{-T} * w
%   where w = E' * P * z,  z = [e; e_dot]
%
% Rate limiting is applied to pi_hat_dot to prevent numerical blow-up
% when M_hat is ill-conditioned (small masses/inertias).

    n = 4;

    % tracking error
    e = q_d - q;
    e_dot = q_dot_d - q_dot;

    % clamp parameters to stay physical
    pi_hat(1:4) = max(pi_hat(1:4), 1e-4);
    pi_hat(5:16) = max(pi_hat(5:16), 1e-8);

    % build full parameter vector
    p_hat = [p_geom(1:6); pi_hat; p_geom(7)];

    % estimated dynamics
    M_hat = M_fun(q, p_hat);
    C_hat = C_fun(q, q_dot, p_hat);
    G_hat = G_fun(q, p_hat);

    % Kp = pinv(M_hat) * Kp;
    % Kv = pinv(M_hat) * Kv;
    % 
    % commanded acceleration
    aq = q_ddot_d + Kv * e_dot + Kp * e;

    % control torque
    tau = M_hat * aq + C_hat * q_dot + G_hat;

    % --- Adaptation law ---
    z = [e; e_dot];

    % w = E' * P * z = bottom n rows of P times z
    % w = P(n+1:2*n, :) * z;
    E =[zeros(n,n);eye(n)];
    w = E' * P *z;

    % regressor at commanded acceleration
    Y = Y_fun(q, q_dot, aq, p_geom);

    % raw adaptation update
    pi_hat_dot = R_inv * (Y' * ((pinv(M_hat))' * w));

    % rate limit: each element can change at most 100% of its value per second
    % floor prevents division issues for very small parameters
    max_rate = max(abs(pi_hat), 1e-6);
    pi_hat_dot = max(min(pi_hat_dot, max_rate), -max_rate);

    % kill any NaN or Inf that might have leaked through
    pi_hat_dot(~isfinite(pi_hat_dot)) = 0;
end
