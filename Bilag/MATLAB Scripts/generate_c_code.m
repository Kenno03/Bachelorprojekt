function generate_c_code(b, a)
    % Print numerator coefficients
    fprintf('double b[] = {');
    for i = 1:length(b)
        if i < length(b)
            fprintf('%.4f, ', b(i));
        else
            fprintf('%.4f', b(i));
        end
    end
    fprintf('};    // Numerator\n');

    % Print denominator coefficients
    fprintf('double a[] = {');
    for i = 1:length(a)
        if i < length(a)
            fprintf('%.4f, ', a(i));
        else
            fprintf('%.4f', a(i));
        end
    end
    fprintf('};    // Denominator (a[0] must be 1.0)\n\n');

    % Start control law
    fprintf('u[0] =');

    % Feedback part: -a[1]*u[1] - a[2]*u[2] ...
    for i = 2:length(a)
        fprintf(' - (%.4f * u[%d])', a(i), i-1);
    end

    % Feedforward part: +b[0]*e[0] + b[1]*e[1] ...
    for i = 1:length(b)
        fprintf(' + (%.4f * e[%d])', b(i), i-1);
    end

    % End line
    fprintf(';\n');
end
