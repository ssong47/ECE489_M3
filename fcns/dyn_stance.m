function dXdt = dyn_stance(t,X,p)
global GRFz 
params = p.params;
tTD = p.tTD;            % touchdown time
ptTD = p.ptTD;          % touchdown toe position
Tst = p.Tst;

q = X(1:4);
dq = X(5:8);

De = fcn_De(q,params);
Ce = fcn_Ce(q,dq,params);
Ge = fcn_Ge(q,params);
Be = fcn_Be(q,params);
J = fcn_J_toe(q,params);

% Holonomic constraints
Jhc = fcn_Jhc(q,ptTD,params);
dJhc = fcn_dJhc(q,dq,ptTD,params);

%% Controller
% Feedforward force
s = (t - tTD) / Tst;        % stance phase parametrization s = [0, 1]
% Force profile using Bezier polynomials
F = polyval_bz([0 0 60 0 0], s);

%Joint-level control
Jc_HIP = fcn_J_toe_HIP(q,params);
u = -Jc_HIP'*[0; F];    %Torques of joints 3 (hip) and 4 (knee)

% Solve the linear system:
% De * ddq + Ce * dq + Ge = J' * GRF + Be * u (4 eqns)
% Jhc * ddq + dJhc * dq = 0 (2 eqns)
% [De  -Jhc'] * [ddq] = [Be*u - Ce*dq - Ge]
% [Jhc  0   ]   [GRF]   [-dJhc*dq         ]
% unknowns: ddq(4x1), GRF(3x1) 
% control: u(2x1)
Amat = [De -Jhc'; 
        Jhc zeros(2,2)];
bvec = [Be*u - Ce*dq - Ge; 
        -dJhc*dq];

ddqu = Amat \ bvec;
ddq = ddqu(1:4);
GRF = ddqu(5:6);

GRFz = GRF(end); %Vertical component of ground reaction force
dXdt = [dq; ddq];



