function HTM_K2H = kneeToHip(theta3,Lh)
%This function calculates the homogenous transformation matrix from 
%the knee frame to hip frame.
l = Lh;

R_K2H = [1 0 0; 0 cos(theta3) -sin(theta3); 0 sin(theta3) cos(theta3)];
o_K2H = [0; l*sin(theta3); -l*cos(theta3)];

HTM_K2H = [R_K2H, o_K2H; 0 0 0 1];

end

