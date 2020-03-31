% Flag for animation
animate = 0;

global GRFz t_prev i_prev
clc
close all
addpath gen
addpath fcns

t_prev = 0;
i_prev = 0;

% --- parameters ---
p = get_params;     % Getting physical parameters of the robot
Nstep = 5;          % number of desired hops
step_size = 1e-3;
% Initial condition
q0 = [0; 0; pi/3; -pi/2]; %Joint angles
dq0 = [0; 0; 0; 0];       %Joint velocities
ic = [q0; dq0];
flag = '1a'; % Change this flag to either "1a" "1b" "1c" 

%Ploting the robot in the initial configuration:
%plotRobot(ic,p);

% Recording
tstart = 0;
tfinal = 100;   %Maximum simulation time
tout = tstart;
Xout = ic';

for istep = 1:Nstep
    %% aerial phase
    options = odeset('Events',@(t,X)event_touchDown(t,X,p),'MaxStep',step_size);

    [t,X] = ode45(@(t,X)dyn_aerial(t,X,p,flag),[tstart, tfinal], Xout(end,:),options);

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
    options = odeset('Events',@(t,X)event_liftOff(t,X,p),'MaxStep',step_size);

    [t,X] = ode45(@(t,X)dyn_stance(t,X,p, flag),[tstart, tfinal], Xout(end,:), options);

    nt = length(t);
    tout = [tout; t(2:nt)];
    Xout = [Xout; X(2:nt,:)];
    tstart = tout(end);

end


%% Visualing the motion
if animate == 1
    animateRobot(tout,Xout,p)
end



%% Milestone 3 processing 
if strcmp(flag,'1a') == 1

    % Getting necessary linkage lengths
    HB = p.params(2);
    LB = p.params(3);
    DB = p.params(4);


    % Defining position of the frames 
    r0 = [0; 0; HB];
    r1 = [LB; DB; 0];

    % Defining joint angles from simulation output data
    q1 = Xout(:,1);
    q2 = Xout(:,2);


    hip_height_1 = zeros(length(q1),1);

    for i = 1:length(q1)    
        R01 = rz(q1(i));
        R12 = ry(q2(i));
        T01= [R01 R01*r0;
           0, 0, 0, 1];       
        T12 = [R12 R12*r1;
           0, 0, 0, 1];    

        hip_height_1(i) = T12(3,4);
    end

    figure(2)
    plot(tout, hip_height_1*1000);
    xlabel('Time (sec)');
    ylabel('Height (mm)');
    
elseif strcmp(flag,'1b') == 1
    angular_speed = Xout(:,5);

    figure(3)
    plot(tout, angular_speed);
    ylabel('Angular speed (rad/s)');
    xlabel('Time (sec)');
    
elseif strcmp(flag,'1c') == 1
    q1 = Xout(:,1);
    figure(4)
    plot(tout,q1);
    hold on;
    hline(pi/2);
    hline(pi/2+0.1);
    hline(pi/2-0.1);
    xlabel('Time (sec)');
    ylabel('Angle (rad)'); 
  
    
end







