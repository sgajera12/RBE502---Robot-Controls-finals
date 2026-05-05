function tau = inverse_controller(q, q_dot, q_d, q_dot_d, q_ddot_d, Kp, Kv, p)
% inverse_controller  Computed torque control (no friction, for simulation).
%   tau = M(q) * (q_ddot_d + Kv*e_dot + Kp*e) + C(q,q_dot)*q_dot + G(q)

    e = q_d - q;
    e_dot = q_dot_d - q_dot;

    M = M_fun(q, p);
    C = C_fun(q, q_dot, p);
    G = G_fun(q, p);

    aq = q_ddot_d;
    tau = M * aq+ Kv * e_dot + Kp * e  + C * q_dot + G;
end
