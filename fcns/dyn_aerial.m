function dXdt = dyn_aerial(t,X,p)

params = p.params;
q = X(1:4); %joint positions
dq = X(5:8);    %joint velocities

De = fcn_De(q,params); %inertia matrix
Ce = fcn_Ce(q,dq,params); %Coriolis matrix
Ge = fcn_Ge(q,params); %gravity vector
Be = fcn_Be(q,params); %actuation selection matrix

% swing controller
qd = [pi/3; -pi/2];     % desired joint position
q1 = q(3);
q2 = q(4);
dq1 = dq(3);
dq2 = dq(4);

% joint PD control during aerial phase
u = [5*(qd(1)-q1) + .2*(0-dq1);...
     5*(qd(2)-q2) + .2*(0-dq2)];

ddq = De \ (Be * u - Ce*dq - Ge);

dXdt = [dq;ddq];
