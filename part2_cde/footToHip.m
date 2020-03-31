function HTM_F2H = footToHip(theta3, theta4, Dk, Lk, Lh)
%This function calculates the homogenous transformation matrix from 
%the foot frame to hip frame.

HTM_F2H = kneeToHip(theta3, Lh)*footToKnee(theta4, Dk, Lk);

end

