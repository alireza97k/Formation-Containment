% in this test I want use 3 mobile robot to make a formation 


% point stabilization + Single shooting
clear all
close all
clc

% CasADi v3.4.5
% addpath('C:\Users\mehre\OneDrive\Desktop\CasADi\casadi-windows-matlabR2016a-v3.4.5')
% CasADi v3.5.5
addpath('C:\Users\Alireza\Desktop\MPC_Mobile_Robot')
import casadi.*

T = 0.1; % sampling time [s]
N = 5; % prediction horizon
rob_diam = 0.3;

v_max = 0.6; v_min = -v_max;
omega_max = pi/4; omega_min = -omega_max;

x1 = SX.sym('x1'); y1 = SX.sym('y1'); theta1 = SX.sym('theta1');
x2 = SX.sym('x2'); y2 = SX.sym('y2'); theta2 = SX.sym('theta2');
x3 = SX.sym('x2'); y3 = SX.sym('y2'); theta3 = SX.sym('theta2');
states = [x1;y1;theta1;x2;y2;theta2;x3;y3;theta3]; n_states = length(states);

v1 = SX.sym('v1'); omega1 = SX.sym('omega1');
v2 = SX.sym('v2'); omega2 = SX.sym('omega2');
v3 = SX.sym('v3'); omega3 = SX.sym('omega3');
controls = [v1;omega1;v2;omega2;v3;omega3]; n_controls = length(controls);


rhs = [v1*cos(theta1);v1*sin(theta1);omega1;v2*cos(theta2);v2*sin(theta2);omega2;v3*cos(theta3);v3*sin(theta3);omega3]; % system r.h.s
f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)


U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + 3 + 3 + 3);
% parameters (which include the initial and the reference state of the robot)

X = SX.sym('X',n_states,(N+1));
% A Matrix that represents the states over the optimization problem.

% compute solution symbolically
X(:,1) = P(1:n_states); % initial state
for k = 1:N
    st = X(:,k);  con = U(:,k);
    f_value  = f(st,con);
    st_next  = st+ (T*f_value);
    X(:,k+1) = st_next;
end
% this function to get the optimal trajectory knowing the optimal solution
ff=Function('ff',{U,P},{X});

obj = 0; % Objective function
g = [];  % constraints vector


Q_ = [10,0,0;0,10,0;0,0,0.1];
Q = blkdiag(Q_,Q_,Q_);
R_ =  [0.5,0 ; 0,0.05];
R = blkdiag(R_,R_,R_);

% compute objective
for k=1:N
    st = X(:,k);  con = U(:,k);
    obj = obj+(st(1:3)-st(4:6)-P(n_states+1:n_states+3))'*Q_*(st(1:3)-st(4:6)-P(n_states+1:n_states+3));
    obj = obj+(st(4:6)-st(7:9)-P(n_states+4:n_states+6))'*Q_*(st(4:6)-st(7:9)-P(n_states+4:n_states+6));
    obj = obj+(st(7:9)-st(1:3)-P(n_states+7:n_states+9))'*Q_*(st(7:9)-st(1:3)-P(n_states+7:n_states+9));
    obj = obj + con'*R*con; % calculate obj
end

% compute constraints
for k = 1:N+1   % box constraints due to the map margins
    g = [g ; X(1,k)];   %state x1
    g = [g ; X(2,k)];   %state y1
    g = [g ; X(4,k)];   %state x2
    g = [g ; X(5,k)];   %state y2
    g = [g ; X(7,k)];   %state x2
    g = [g ; X(8,k)];   %state y2
end

% make the decision variables one column vector
OPT_variables = reshape(U,3*N*2,1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);


args = struct;
% inequality constraints (state constraints)
args.lbg = -20;  % lower bound of the states x and y
args.ubg = 20;   % upper bound of the states x and y 

% input constraints
args.lbx(1:2:3*2*N-1,1) = v_min; args.lbx(2:2:3*2*N,1)   = omega_min;
args.ubx(1:2:3*2*N-1,1) = v_max; args.ubx(2:2:3*2*N,1)   = omega_max;


%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SETTING UP


% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0 = 0;
x0 = [-2 ; -2 ; 0; 2 ; 2 ; pi ;1 ; 1; 0 ];    % initial condition.


tol_tri = 2;
xs = [-tol_tri/2 ; -sqrt(3)*tol_tri/2 ; 0; -tol_tri/2; + sqrt(3)*tol_tri/2; 0 ; tol_tri ; 0 ;0]; % Reference posture.

xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,6);  % two control inputs 

sim_tim = 20; % Maximum simulation time

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];


% the main simulaton loop... it works as long as the error is greater
% than 10^-2 and the number of mpc steps is less than its maximum
% value.
main_loop = tic;
while ( mpciter < sim_tim / T)
    args.p   = [x0;xs]; % set the values of the parameters vector
    args.x0 = reshape(u0',6*N,1); % initial value of the optimization variables
    %tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
    %toc
    u = reshape(full(sol.x)',6,N)';
    ff_value = ff(u',args.p); % compute OPTIMAL solution TRAJECTORY
    xx1(:,1:9,mpciter+2)= full(ff_value)';
    
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    [t0, x0, u0] = shift(T, t0, x0, u,f); % get the initialization of the next optimization step
    
    xx(:,mpciter+2) = x0;  
    mpciter
    mpciter = mpciter + 1;
end;
main_loop_time = toc(main_loop)

figure

p1 = plot(xx(1,:),xx(2,:),'b');
hold on
p2 = plot(xx(4,:),xx(5,:),'r');
hold on
p3 = plot(xx(7,:),xx(8,:),'g');
hold on

figure
x_r_1 = [];
y_r_1 = [];
x_r_2 = [];
y_r_2 = [];
x_r_3 = [];
y_r_3 = [];

for k = 1:size(xx,2)
    h_t = 0.2; w_t=0.15;    
    x1 = xx(1,k,1); y1 = xx(2,k,1); th1 = xx(3,k,1);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    
    x2 = xx(4,k,1); y2 = xx(5,k,1); th2 = xx(6,k,1);
    x_r_2 = [x_r_2 x2];
    y_r_2 = [y_r_2 y2];
    x2_tri = [ x2+h_t*cos(th2), x2+(w_t/2)*cos((pi/2)-th2), x2-(w_t/2)*cos((pi/2)-th2)];%,x1+(h_t/3)*cos(th1)];
    y2_tri = [ y2+h_t*sin(th2), y2-(w_t/2)*sin((pi/2)-th2), y2+(w_t/2)*sin((pi/2)-th2)];%,y1+(h_t/3)*sin(th1)];    
    
    
    x3 = xx(7,k,1); y3 = xx(8,k,1); th3 = xx(9,k,1);
    x_r_3 = [x_r_3 x3];
    y_r_3 = [y_r_3 y3];
    x3_tri = [ x3+h_t*cos(th3), x3+(w_t/2)*cos((pi/2)-th3), x3-(w_t/2)*cos((pi/2)-th3)];%,x1+(h_t/3)*cos(th1)];
    y3_tri = [ y3+h_t*sin(th3), y3-(w_t/2)*sin((pi/2)-th3), y3+(w_t/2)*sin((pi/2)-th3)];%,y1+(h_t/3)*sin(th1)];
    
    % plot(x_r_1,y_r_1,'-r');
    % plot exhibited trajectory
    fill(x1_tri, y1_tri, 'r'); % plot robot position
    hold on 
    fill(x2_tri, y2_tri, 'b'); % plot robot position
    hold on 
    fill(x3_tri, y3_tri, 'g'); % plot robot position
    hold off
    axis([-3,3,-3,3]);
    grid on
    pause(0.1)
    drawnow    
end
