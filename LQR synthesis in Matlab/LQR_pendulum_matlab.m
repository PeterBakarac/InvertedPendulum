% Linear model of the inverted pendulum and the synthesis of LQR controller
%
% Description of the states:
% 	x1: angle of the pendulum [radians]
%	x2: angular velocity of the pendulum [radians/s]
% 	x3: position of the cart [m]
% 	x4: velocity of the cart [m/s]
%

% Constants definition
Ts = 0.008;     % sampling time
g = 9.80665;    % gravitational acceleration
b = 1;          % friction
L = 0.20;       % half-length of the penulum [m]

% Linear model of the inverted pendulum
A = [0, 1, 0, 0;
      3*g/4/L, -b, 0, 0;
      0, 0, 0, 1;
      0, 0, 0, 0];
B = [0; 3/4/L; 0; 1];
C = [1 0 0 0];
D = [0];

% Conversion to state space representation
sys_con = ss(A,B,C,D);

% Convesion to discrete-time domain
sd = c2d(sys_con, Ts);

% Declaring of weighting matrices
Q = diag([1e3 1 100 1]);
R = 10;

% Obtaining LQR controller
K = dlqr(sd.A, sd.B, Q, R)