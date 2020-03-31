function HTM_F2K = footToKnee(theta4,Dk, Lk)
%This function calculates the homogenous transformation matrix from 
%the foot frame to knee frame. 
l = sqrt(Dk^2 + Lk^2);

R_F2K = [1 0 0; 0 cos(theta4) -sin(theta4); 0 sin(theta4) cos(theta4)];
o_F2K = [0; l*sin(theta4); -l*cos(theta4)];

HTM_F2K = [R_F2K, o_F2K; 0 0 0 1];

end

