% Hopping leg with boom
% Author: Yanran Ding and Joao Ramos 
% Last modified: 2020/03/05

global GRFz
global u_control
global counter
global t_control

u_control = zeros(1000,2);
t_control = zeros(1000,1);
counter = 0;


clc
close all
addpath gen
addpath fcns

% --- parameters ---
p = get_params;     % Getting physical parameters of the robot
Nstep = 10;          % number of desired hops

% Initial condition
q0 = [0; 0; pi/3; -pi/2]; %Joint angles
dq0 = [0; 0; 0; 0];       %Joint velocities
ic = [q0; dq0];

%Ploting the robot in the initial configuration:
%plotRobot(ic,p);

% Recording
tstart = 0;
tfinal = 100;   %Maximum simulation time
tout = tstart;
Xout = ic';


for istep = 1:Nstep
    %% aerial phase
    options = odeset('Events',@(t,X)event_touchDown(t,X,p),'MaxStep',1e-3);

    [t,X] = ode45(@(t,X)dyn_aerial(t,X,p),[tstart, tfinal], Xout(end,:),options);

    p.tTD = t(end);                             % touchdown time
    p.ptTD = fcn_p_toe(X(end,1:4),p.params);    % touchdown toe pos
    GRFz = 1;

    nt = length(t);
    tout = [tout; t(2:nt)];
    Xout = [Xout; X(2:nt,:)];
    tstart = tout(end);

    %% Impact map (hard contact)
    X_prev = Xout(end,:);
    X_post = fcn_impactMap(X_prev,p);
    Xout(end,:) = X_post';

    %% stance phase
    options = odeset('Events',@(t,X)event_liftOff(t,X,p),'MaxStep',1e-3);

    [t,X] = ode45(@(t,X)dyn_stance(t,X,p),[tstart, tfinal], Xout(end,:), options);

    nt = length(t);
    tout = [tout; t(2:nt)];
    Xout = [Xout; X(2:nt,:)];
    tstart = tout(end);

    disp(istep);
end

%% For plotting control input
figure
plot(t_control, u_control(:,1));
title('Control Input to Hip joint', 'FontSize', 15);
xlabel('Time (s)', 'FontSize', 15);
ylabel('Control Input (V)', 'FontSize', 15);

figure
plot(t_control, u_control(:,2));
title('Control input to Knee joint', 'FontSize', 15);
xlabel('Time (s)', 'FontSize', 15);
ylabel('Control Input (V)', 'FontSize', 15);

figure
plot(tout, Xout(:,1));
hold on
plot(tout, Xout(:,2));
title('Joint angles of 10 hops for 2b', 'FontSize', 15);
xlabel('Time (s)', 'FontSize', 15);
ylabel('Joint angle', 'FontSize', 15);
legend('$\theta_1$','$\theta_2$','Interpreter','Latex');

%% Visualing the motion
animateRobot(tout,Xout,p)






