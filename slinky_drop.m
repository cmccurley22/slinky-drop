function slinky_drop
%% system parameters
k = 20; % spring constant
N = 10; % number of masses in model
m = .1 / N; % mass of each (kg)
g = 9.81;

collided = 0; % tracker on how many masses have collided

%% initial model
[A, B] = model(k, m, N, collided, g);

A_init = A(N+2:end, 2:N);
B_init = B(N+2:end);

Y_init = [0; linsolve(A_init, -B_init); zeros(N, 1)];

time = linspace(0, 5, 30 / .005);

%% run sim - first collision
options = odeset("Events", @(t, y) mass_collision(t, y, collided));
[t_out, y_out] = ode45(@(t, y) springs(t, y, A, B), time, Y_init, options);
t_total = t_out;
y_total = y_out;

%% loop sim until all masses collide
for collided = 1:N-2
    [A, B] = model(k, m, N, collided, g);

    % new initial condition = previous end condition
    y_0 = y_out(end, :);

    v = (collided * y_out(end, N + collided) + ...
        y_out(end, N + collided + 1)) / (collided + 1);

    y_0(N + 1: N + 1 + collided) = v;

    % run ode45 sim
    options = odeset("Events", @(t, y) mass_collision(t, y, collided));
    [t_out, y_out] = ode45(@(t, y) springs(t, y, A, B), ...
        time, y_0, options);

    % combine into two matrices, t and y
    t_total = [t_total; t_total(end) + t_out];
    y_total = [y_total; y_out];
end
%% last phase: collapsed slinky falls as one mass
time = 0:.01:10;

collided = N - 1;

y_0 = y_out(end, 1);
vy_0 = (collided * y_out(end, N + collided) + ...
        y_out(end, N + collided + 1)) / (collided + 1);
y_init = [y_0; vy_0];

options = odeset("Events", @(t, y) stop_fall(t, y));
[t_out, y_out] = ode45(@(t, y) falling_mass(t, y, g), ...
    time, y_init, options);

t_total = [t_total; t_total(end) + t_out];

% apply final fall to all masses
y_out_all = repmat(y_out(:, 1), 1, N);
vy_out_all = repmat(y_out(:, 2), 1, N);
Y_out_fall = [y_out_all, vy_out_all];

y_total = [y_total; Y_out_fall];

%% plotting
c = cool(N); % pretty cool colors

% animation
playback_speed = 0.005;
tF = 5; % final time (s)
fR = 30 / playback_speed; % frame rate (fps)
time = linspace(0, tF, tF * fR);  

figure
set(gcf,'Position',[50 50 1280 720])

v = VideoWriter('slinky_drop_10_mass.avi', 'Motion JPEG AVI');
v.Quality = 100;

open(v);
for i=1:length(t_total)
    cla
    hold on ; grid on ; box on
    set(gca,'Ydir','reverse')
    set(gca,'Xtick',[])
    
    for j=1:N-1
        ps = plot([0 0], [y_total(i, j) y_total(i, j + 1)], 'k:', ...
            'Displayname', 'Spring');
    end

    for j=1:N
        pm(j) = plot(0, y_total(i, j), 'o', 'color', c(j, :), ...
            'MarkerFaceColor', c(j, :), 'MarkerSize', 6, 'Displayname', ...
            strcat('m',num2str(j)));
    end
    
    legend([ps(end) pm])
    ylabel('Position [m]')
    ylim([0 .4])
    title(strcat('Time=', num2str(time(i), '%.3f'), ...
        ' s (Playback speed=', num2str(.005), ')'))
        
    frame = getframe(gcf);
    writeVideo(v, frame);
end
close(v);

subplot(2, 1, 1)
    cla
    set(gca, 'Ylim', [0 max(t_total(:, 1))])
    hold on ; grid on ; box on
    set(gca, 'Ydir', 'reverse')
    for j=1:N
        plot(t_total, y_total(:, j), 'color', c(j,:), 'LineWidth', 2)
    end
    xlabel('time (s)')
    ylabel('position (m)')
    title("Position of Slinky Modelled as 2 Masses")

subplot(2, 1, 2)
    cla
    set(gca, 'Ylim', [0 max(y_total(:, N+1))]) % Using first element
    hold on ; grid on ; box on
    for j=1:N
        plot(t_total, y_total(:, N+j), 'color', c(j,:), 'LineWidth', 2)
    end
    xlabel('time (s)')
    ylabel('speed (m/s)')
    title("Speed of Slinky Modelled as 2 Masses")

%% functions for equations of motion
function dY = springs(~, Y, A, B)
    dY = A * Y + B;
end

function dY = falling_mass(~, Y, g)
    vy = Y(2);
    ay = g;

    dY = [vy; ay];
end
%% model setup function
function [A, B] = model(k, m, N, collided, g)
    % start with identity matrix
    A_eye = -2 * k / m * eye(N);

    % values for above and below the diagonal
    A_above = k / m * diag(ones(N - 1, 1), +1);
    A_below = k / m * diag(ones(N - 1, 1), -1);

    % sum together
    A_curr = A_eye + A_above + A_below;

    
    % update the first and last elements
    A_curr(1, 1) = -k / m;
    A_curr(end, end) = -k / m;

    % update based on collisions - all points that have collided so far
    % remain together
    if collided >= 1
        A_curr(:, 1:collided) = 0;
        A_curr(1:1+collided, 1+collided) = -k / ((1 + collided) * m);
        A_curr(1:1+collided, 2+collided) = k / ((1 + collided) * m);
    end

    % fill values into NxN matrix
    A = [zeros(N), eye(N); A_curr, zeros(N)];
    B = [zeros(N, 1); g * ones(N, 1)];
end
%% event functions
function [y_pos, is_terminal, dir] = mass_collision(~, y, c)
    y_pos  = y(1 + c) - y(2 + c); % masses 1 and 2 share the same x
    is_terminal = 1;
    dir = 0; % can approach from either direction
end

function [y_pos, is_terminal, dir] = stop_fall(~, y)
    y_pos = y(1) - .2;
    is_terminal = 1;
    dir = 1;
end
end