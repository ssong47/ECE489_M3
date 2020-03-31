function dXdt = dyn_aerial(t,X,p, flag)

params = p.params;
q = X(1:4); %joint positions
dq = X(5:8);    %joint velocities

De = fcn_De(q,params); %inertia matrix
Ce = fcn_Ce(q,dq,params); %Coriolis matrix
Ge = fcn_Ge(q,params); %gravity vector
Be = fcn_Be(q,params); %actuation selection matrix

% swing controller
if strcmp(flag,'1a') == 1
    qd = [pi/3; -pi/2];     % desired joint position

elseif strcmp(flag,'1b') == 1
    qd = [70 * pi/180; -100 * pi/180];     % desired joint position

elseif strcmp(flag,'1c') == 1
    qd = [70 * pi/180; -100 * pi/180];     % desired joint position

end
q1 = q(3);
q2 = q(4);
dq1 = dq(3);
dq2 = dq(4);

% joint PD control during aerial phase
u = [5*(qd(1)-q1) + .2*(0-dq1);...
     5*(qd(2)-q2) + .2*(0-dq2)];

ddq = De \ (Be * u - Ce*dq - Ge);

dXdt = [dq;ddq];
