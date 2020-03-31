%% Milestone 3 Part 2(c,d,e)

% Robot Parameters (mm). 
Dk = 52; Lk = 155; Lh = 96;
Db = 48; Lb = 500; Hb = 240;

% Spring ends physical parameters (mm). See milestone 3 guidelines part 2(c)
a = 32; b = 32;
c = 52; d = 46;

% For solving spring length using Analytical Solution
gamma = atan2(c, Lk);
end2_F = [0; (Lk+d)*sin(gamma); (Lk+d)*cos(gamma); 1];
end1_K = [0; b; a; 1];

theta4 = [-pi:0.001:60*pi/180];
%theta4 = [-pi:0.001:-63.65*pi/180];
L = zeros(1, length(theta4));

% For solving spring length using Numerical solution
offset = pi - atan2(Lk, Dk) - atan2(b,a) - atan2(d,c);
l1 = sqrt(a^2+b^2);
l2 = sqrt(c^2+d^2);

spLength = @(theta)sqrt(l1^2+l2^2-2*l1*l2*cos(-theta+offset));

L2 = zeros(1, length(theta4));

% Finding the values of spring length 
for i = 1:length(theta4)
    end2_H = footToHip(0, theta4(i), Dk, Lk, Lh)*end2_F;
    end1_H = kneeToHip(0, Lh)*end1_K;
    
    L(i) = sqrt((end2_H - end1_H)'*(end2_H - end1_H));
    L2(i) = spLength(theta4(i));
end

%% Plotting the values
figure(1)
line_width = 2.0;
plot(theta4*180/pi, L,'b-', 'Linewidth',line_width);
hold on;
plot(theta4*180/pi, L2, 'r--', 'Linewidth',line_width); 
title('Length');
legend('analytical','numerical');
xlabel('$\theta_3 (deg)$','Interpreter', 'Latex');
ylabel('Spring Length (mm)');


%% Fitting the data using polyfit
coef = polyfit(theta4, L, 30);
plot(theta4*180/pi, polyval(coef, theta4)); 

% Determining error between real and approximate spring length 
disp('Maximum error between approximate value and real value of length:');
disp(max(polyval(coef,theta4)-L));

%% Determining derivative of spring length change 

% Solution from Numerical fitting
dL = polyval(polyder(coef), theta4); 

% Solution from Analytical solution
spLength_der = @(theta)(-l1*l2*sin(-theta+offset))./spLength(theta);
dL2 = spLength_der(theta4);

% Plotting derivatives
figure
plot(theta4*180/pi, dL); hold on;
title('Derivative');
plot(theta4*180/pi, dL2);

disp('Maximum error between approximate value and exact value of derivative:');
disp(abs(max(dL-dL2)));


