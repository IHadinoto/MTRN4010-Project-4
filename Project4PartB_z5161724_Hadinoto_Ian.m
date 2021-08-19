clear all;
clc; close all;

%% Question 1
x = [0;10;0;100]; % Initial Condition
y = [0;0;0;5000]; % Output is x1, x2->0
dt = 0.25; % sampling Time
u = 0; % initial control signal
T_end = 30; % ending simulation

% States
A = [-1.28 0 0.98 0;0 0 1 0;-5.43 0 -1.84 0;-128.2 -128.2 0 0];
B = [-0.3;0;-17;0];
C = eye(4);

%% Constrained MPC
N1 = 5;
N2 = 10;
[u0,x0,y0,t0] = ConstrainedMPC(A,B,C,x,y,N1,dt,T_end);
[u1,x1,y1,t1] = ConstrainedMPC(A,B,C,x,y,N2,dt,T_end);

%% Plotting
figure;
subplot(3,2,1)
plot(t1,x1(1,1:end-1),'b-',t0,x0(1,1:end-1),'r--')
title('Angle of Attack');legend('N=10','N=5');
xlabel('Time/s');ylabel('Angle/rad')
grid on;
subplot(3,2,2)
plot(t1,x1(2,1:end-1),'b-',t0,x0(2,1:end-1),'r--')
title('Pitch Angle');legend('N=10','N=5');
xlabel('Time/s');ylabel('Angle/rad')
grid on;
subplot(3,2,3)
plot(t1,x1(3,1:end-1),'b-',t0,x0(3,1:end-1),'r--')
title('Pitch Rate');legend('N=10','N=5');
xlabel('Time/s');ylabel('Angle/rads^-^1')
grid on;
subplot(3,2,4)
plot(t1,x1(4,1:end-1),'b-',t0,x0(4,1:end-1),'r--')
title('Altitude');legend('N=10','N=5');
xlabel('Time/s');ylabel('Altitude/m')
grid on;
subplot(3,2,[5,6])
plot(t1,u1,'b-',t0,u0,'r--')
xlabel('Time/s'); ylabel('Control Action U')
title('Constrained MPC u = Kx');legend('N=10','N=5');
grid on;

%% Question 2
x0 = [0;10;0;100]; % Initial Condition
x1 = [10;0;100;0]; % Initial Condition
y = [0;0;0;5000]; % Output is x1, x2->0
dt = 0.25; % sampling Time
u = 0; % initial control signal
T_end = 30; % ending simulation

% States
A = [-1.28 0 0.98 0;0 0 1 0;-5.43 0 -1.84 0;-128.2 -128.2 0 0];
B = [-0.3;0;-17;0];
C = eye(4);

%% Constrained MPC
N = 10;
[u0,x0,y0,t0] = ConstrainedMPC(A,B,C,x0,y,N,dt,T_end);
[u1,x1,y1,t1] = ConstrainedMPC(A,B,C,x1,y,N,dt,T_end);

%% Plotting
figure;
subplot(3,2,1)
plot(t1,x1(1,1:end-1),'b-',t0,x0(1,1:end-1),'r--')
title('Angle of Attack');legend('x(0)=[10,0,100,0]','x(0)=[0,10,0,100]');
xlabel('Time/s');ylabel('Angle/rad')
grid on;
subplot(3,2,2)
plot(t1,x1(2,1:end-1),'b-',t0,x0(2,1:end-1),'r--')
title('Pitch Angle');legend('x(0)=[10,0,100,0]','x(0)=[0,10,0,100]');
xlabel('Time/s');ylabel('Angle/rad')
grid on;
subplot(3,2,3)
plot(t1,x1(3,1:end-1),'b-',t0,x0(3,1:end-1),'r--')
title('Pitch Rate');legend('x(0)=[10,0,100,0]','x(0)=[0,10,0,100]');
xlabel('Time/s');ylabel('Angle/rads^-^1')
grid on;
subplot(3,2,4)
plot(t1,x1(4,1:end-1),'b-',t0,x0(4,1:end-1),'r--')
title('Altitude');legend('x(0)=[10,0,100,0]','x(0)=[0,10,0,100]');
xlabel('Time/s');ylabel('Altitude/m')
grid on;
subplot(3,2,[5,6])
plot(t1,u1,'b-',t0,u0,'r--')
xlabel('Time/s'); ylabel('Control Action U')
title('Constrained MPC u = Kx');legend('x(0)=[10,0,100,0]','x(0)=[0,10,0,100]');
grid on;

%% Function
function [u,x,y,t] = ConstrainedMPC(A,B,C,x,y,N,dt,T_end)
t = 0:dt:T_end; % total time duration
    Nsim = length(t) - 1; % Total Samples

    nx=size(A,2); % number of states
    nu=size(B,2); % number of inputs
    R = 10; % Input penalization 
    Q = diag([0,1,0,1]); %State Penalization
    Q_bar = kron(eye(N),Q); %Running over the Prediction Horizon
    R_bar = kron(eye(N),R); %Running over the Preidction Horizon

    % Build Sx Build Su for Equation 5 (X = x_0*Sx + u*Su)
    Su=zeros(N*nx,N*nu);
    for i=1:N
       Sx((i-1)*nx+1:i*nx,:)=A^i; %Sx
       for j=1:i
          Su((i-1)*nx+1:(i)*nx,(j-1)*nu+1:j*nu)=A^(i-j)*B; %Su
       end
    end

    F = Su'*Q_bar*Sx; %Matrix F for the Equation 20-21
    G = R_bar + Su'*Q_bar*Su; %Matrix G
    z = zeros(1,N-1); %Zeros for gain multiplication
    K = -[1 z]*inv(G)*F; %MPC Gain
    
    % Constrained MPC Part
    H = 2*G; % Cost vector H
    E1 = eye(N,N); E2 = eye(N,N);
    E = [E1;-E2]; % For Constraints Ez <= b
    u_max = 0.262;
    u_min = -0.262;
    
    b_unit = [u_max; -u_min]; % Constraints on Input
    b = [];

    % Create vector b based of N
    for i = 1:N
        b = [b;b_unit];
    end

    % Simulation
    for k = 1 : Nsim + 1
        xp = x(:,k); % Measure the state x0 at time instant k
        f = 2*(F *xp); %update cost vector
        options = optimoptions('quadprog','MaxIterations',300);
        Aeq = [];
        beq = [];
        lb = [];
        ub = [];
        x0 = [];
        uopt = quadprog(H,f,E,b,Aeq,beq,lb,ub,x0,options); %Compute optimal control U^*
        u(:,k) = uopt(1); %Apply the first element u^*[0] of U to the platform
        x(:, k + 1) = A*x(:,k) + B*u(:, k);
        y(:,k) = C*x(:,k);
    end
end