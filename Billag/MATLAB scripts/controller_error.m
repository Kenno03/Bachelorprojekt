function err = controller_error_fixed_Ni(x, Gs, Ts, b_target, a_target, N_i)
    alpha = x(1);
    gamma_m = x(2);

    if alpha <= 0 || alpha >= 1 || gamma_m <= 0 || gamma_m >= 180
        err = 1e6;
        return;
    end

    % Compute required phase
    phi_d = rad2deg(asin((1 - alpha) / (1 + alpha)));
    phi_i = rad2deg(atan2(-1, N_i));
    pc = gamma_m - 180 - phi_d - phi_i;

    % Frequency sweep
    w = logspace(-2, 2, 3000);
    [~, phase] = bode(Gs, w);
    phase = squeeze(phase);
    n = find(phase > pc, 1, 'last');

    if isempty(n)
        err = 1e6;
        return;
    end

    wc = w(n);
    tau_d = 1 / (sqrt(alpha) * wc);
    tau_i = N_i / wc;

    % Build continuous PI-Lead controller
    Cd = tf([tau_d 1], [alpha * tau_d 1]);
    Ci = tf([tau_i 1], [tau_i 0]);
    Col = Cd * Ci;

    % Compute gain at crossover
    [magc, ~] = bode(Gs * Col, wc);
    Kp = 1 / squeeze(magc);

    % Final controller and discretization
    Col_final = Kp * Col;
    Col_z = c2d(Col_final, Ts, 'tustin');
    [b_actual, a_actual] = tfdata(minreal(Col_z), 'v');

    % --- Toolbox-free padding ---
    b_actual = b_actual(:);
    a_actual = a_actual(:);

    if length(b_actual) < length(b_target)
        b_actual(end+1:length(b_target)) = 0;
    end
    if length(a_actual) < length(a_target)
        a_actual(end+1:length(a_target)) = 0;
    end

    % Error = coefficient mismatch
    err = norm(b_target(:) - b_actual) + norm(a_target(:) - a_actual);
end
