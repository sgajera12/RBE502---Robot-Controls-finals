function tau = robust_controller(q, q_dot, q_d, q_dot_d, q_ddot_d, Kp, Kv, p, P, rho)
% robust_controller  Robust computed torque control (no friction, for simulation).
%   tau = M(q)*(q_ddot_d + Kv*e_dot + Kp*e + Delta) + C*q_dot + G
%   Delta uses boundary-layer switching to handle inertia uncertainty.

    n = length(q);

    e = q_d - q;
    e_dot = q_dot_d - q_dot;

    M_hat = M_fun(q, p);
    C_hat = C_fun(q, q_dot, p);
    G_hat = G_fun(q, p);

    z = [e; e_dot];
    B = [zeros(n); eye(n)];

    w = 2* B' * P * z;
 
    w_norm = norm(w);
    if w_norm > 1e-6
        Delta = rho * (w / w_norm);
    else
        Delta = rho * (w / 1e-6);
    end

    aq = q_ddot_d  + Delta;
    tau = M_hat * aq + Kv * e_dot + Kp * e + C_hat * q_dot + G_hat;
end
