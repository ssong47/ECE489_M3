function dXdt = dyn_aerial(t,X,p)

global u_control
global counter 
global t_control
global torque_spring

params = p.params;
q = X(1:4); %joint positions
dq = X(5:8);    %joint velocities

De = fcn_De(q,params); %inertia matrix
Ce = fcn_Ce(q,dq,params); %Coriolis matrix
Ge = fcn_Ge(q,params); %gravity vector
Be = fcn_Be(q,params); %actuation selection matrix
Ts = [0;0;0;2*fcn_Ts(q,params)]; 

% swing controller
qd = [pi/3; -90*pi/180];     % desired joint position
q1 = q(3);
q2 = q(4);
dq1 = dq(3);
dq2 = dq(4);

% joint PD control during aerial phase
u = [5*(qd(1)-q1) + .2*(0-dq1);...
     5*(qd(2)-q2) + .2*(0-dq2)];

% Compensation for spring
u = u - Ts(3:4); 
 
u(1) = u(1)/Be(3,1);
u(2) = u(2)/Be(4,2);

% Trying Saturation
saturate = 20;

% if abs(u(1)) > saturate
%     u(1) = sign(u(1))*saturate;
% end
% 
% if abs(u(2)) > saturate
%     u(2) = sign(u(2))*saturate;
% end

%For plotting control input
counter = counter+1;
u_control(counter,1) = u(1);
u_control(counter,2) = u(2);
t_control(counter) = t;
torque_spring(counter) = Ts(4);

ddq = De \ (Be * u + Ts - Ce*dq - Ge);

dXdt = [dq;ddq];
