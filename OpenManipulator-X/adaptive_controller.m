function [tau, pi_hat_new, pi_hat_dot] = adaptive_controller(q, q_dot, ...
                                q_d, q_dot_d, q_ddot_d, ...
                                Kp, Kv, P, R_inv, ...
                                pi_hat, p_geom, dt)
% adaptive_controller  Adaptive computed torque using regressor form.
%   Matches the real-robot implementation.
%
%   Control law:   tau = Y(q, q_dot, a, p_geom) * pi_hat
%   Adaptation:    pi_hat_dot = R_inv * Y' * pinv(B_hat)' * E' * P * xi
%   B_hat(q) extracted from Y by column perturbation.

    n = length(q);

    % errors
    e    = q_d - q;
    e_dot = q_dot_d - q_dot;
    xi   = [e; e_dot];

    % Build B_hat (estimated inertia) via column extraction from Y
    zer    = zeros(n, 1);
    Y_grav = Y_fun(q, zer, zer, p_geom);
    B_hat  = zeros(n);
    for jj = 1:n
        ej = zer; ej(jj) = 1;
        Y_j = Y_fun(q, zer, ej, p_geom);
        B_hat(:, jj) = (Y_j - Y_grav) * pi_hat;
    end

    % reference acceleration (with pinv(B_hat) on PD terms)
    a = q_ddot_d + pinv(B_hat) * Kv * e_dot + pinv(B_hat) * Kp * e;

    % control law: tau = Y * pi_hat
    Y_a = Y_fun(q, q_dot, a, p_geom);
    tau = Y_a * pi_hat;

    % adaptation law
    E     = [zeros(n); eye(n)];
    Y_dyn = Y_fun(q, q_dot, q_ddot_d, p_geom);
    pi_hat_dot = R_inv * Y_dyn' * pinv(B_hat)' * E' * P * xi;

    % Euler integration
    delta_pi = pi_hat_dot * dt;

    % safety: reject if update is too large
    if norm(delta_pi) > 1.05 * norm(pi_hat)
        delta_pi = zeros(size(pi_hat));
    end

    pi_hat_new = pi_hat + delta_pi;
    pi_hat_new = max(pi_hat_new, 1e-6);
end