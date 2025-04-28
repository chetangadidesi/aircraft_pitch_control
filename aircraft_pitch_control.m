%%%%% OPEN LOOP SYSTEM RESPONSE

s = tf('s');
P_pitch = (1.151*s+0.1774)/(s^3+0.739*s^2+0.921*s);

t = [0:0.01:10];
step(0.2*P_pitch,t);
axis([0 10 0 0.8]);
ylabel('pitch angle (rad)');
title('Open-loop Step Response');

pole(P_pitch)


%%%%% FREQUENCY RESPONSE

sys_cl = feedback(P_pitch,1)
figure(2)
margin(P_pitch), grid

K = 10;
alpha = 0.04;
T = 0.55;
%%% lead compensator design
C_lead = K*(T*s + 1) / (alpha*T*s + 1);
%%%% creating bode plot 
margin(C_lead*P_pitch), grid
%%%% closed loop system response
sys_cl = feedback(C_lead*P_pitch,1);
figure(3)
step(0.2*sys_cl), grid
title('Closed-loop Step Response with K = 10, \alpha = 0.04, and T = 0.55')
stepinfo(0.2*sys_cl)



%%%%%%% LQR 
A = [-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0];
B = [0.232; 0.0203; 0];
C = [0 0 1];
D = [0];

tau = 0.1;

% Original A, B, C, D
A_aug = [A, B; 
         zeros(1,3), -1/tau];     % 4x4

B_aug = [zeros(3,1); 1/tau];      % 4x1 input is u (delta_command)

C_aug = [C, 0];                   % Still just measure theta
D_aug = 0;


co = ctrb(A_aug, B_aug);
rank(co)  % Should be 4

Q_aug = C_aug' * C_aug * 50;  % Penalize theta
R = 1;

K_aug = lqr(A_aug, B_aug, Q_aug, R);

Nbar = rscale(A_aug, B_aug, C_aug, D_aug, K_aug);
sys_cl = ss(A_aug - B_aug * K_aug, B_aug * Nbar, C_aug, D_aug);
figure(4)
step(0.2 * sys_cl)
ylabel('pitch angle (rad)')
title('Closed-Loop Step Response with Actuator Lag')



% Define a more realistic airfoil shape (NACA 2412)
x = linspace(0, 1, 200);  % Increased resolution for smoother curve
y = 0.12 * (0.2969 * sqrt(x) - 0.1260 * x - 0.3516 * x.^2 + 0.2843 * x.^3 - 0.1036 * x.^4);  % NACA 2412 airfoil

% Airfoil coordinates (upper and lower surfaces)
airfoil_upper = [x' y'];
airfoil_lower = [x' -y'];

% Concatenate upper and lower surface to create full airfoil shape
airfoil = [airfoil_upper; flipud(airfoil_lower)];

% Shift the airfoil such that the origin is at the center of the airfoil
airfoil(:, 1) = airfoil(:, 1) - 0.5;  % Shift x-coordinates by -0.5 to center the airfoil

% Define the rotation angle (in radians)
theta = -0.2;  % 0.5 radians (can be replaced with input)

% Rotation matrix for 2D
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];

% Apply the rotation to each point of the airfoil
rotated_airfoil = (R * airfoil')';

% Plot the original and rotated airfoils
figure(5);
hold on;
plot(airfoil(:,1), airfoil(:,2), 'b', 'LineWidth', 1.5);  % Original airfoil
plot(rotated_airfoil(:,1), rotated_airfoil(:,2), 'r', 'LineWidth', 1.5);  % Rotated airfoil
xlabel('x');
ylabel('y');
axis equal;
xlim([-1 1]);
ylim([-1 1]);
title('Airfoil Shape and Rotation with Centered Origin');
legend('Original Airfoil', 'Rotated Airfoil');
grid on;
hold off;


% Time vector
t = 0:0.01:10;
r = 0.2;  % Reference pitch input (step of 0.2 radians)
x0 = zeros(4,1);  % Initial state

% Simulate closed-loop response
[y, t, x] = lsim(sys_cl, r * ones(size(t)), t, x0);

% Compute control input u(t) = -K_aug * x(t)' + Nbar * r
u = -K_aug * x';  % K_aug: 1x4, x': 4xN → u: 1xN
u = u + Nbar * r; % Add feedforward term

% Plot control input
figure(6)
plot(t, u, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Control Input u(t)')
title('Control Input over Time (LQR with Actuator Lag)')
grid on

%%%%%% Starting from non zero position.

% Set the initial condition for the states (3 states + actuator state)
x0 = [0; 0; 0; 0];  % q_0, alpha_0, theta_0 (initial pitch), actuator_state

% Desired reference signal for theta (0.2 radians)
r = 0.2 * ones(size(t));  % Create a step reference at 0.2 radians

% Simulate the system with lsim
[y, t, x] = lsim(sys_cl, r, t, x0);

% Plot the pitch angle (theta) response
figure;
plot(t, y);  
title('Pitch Angle Response');
xlabel('Time (s)');
ylabel('Pitch Angle (rad)');
grid on;
