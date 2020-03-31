function [p_toe] = fcn_p_toe(q,p)

p_toe = zeros(3,1);

  p_toe(1,1)=p(3)*cos(q(1))*cos(q(2)) - p(4)*sin(q(1)) - p(5)*sin(q(1))*sin(q(3)) - cos(q(4))*...
         (sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2) - sin(q(4))*(cos(q(3))*...
         sin(q(1)) - cos(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 + p(7)^2)^(1/2) - p(5)*cos(q(1))*cos(q(3))*sin(q(2));
  p_toe(2,1)=p(4)*cos(q(1)) + p(3)*cos(q(2))*sin(q(1)) + p(5)*cos(q(1))*sin(q(3)) + cos(q(4))*...
         (cos(q(1))*sin(q(3)) - cos(q(3))*sin(q(1))*sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2) + sin(q(4))*(cos(q(1))*...
         cos(q(3)) + sin(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 + p(7)^2)^(1/2) - p(5)*cos(q(3))*sin(q(1))*sin(q(2));
  p_toe(3,1)=p(2) - p(3)*sin(q(2)) - p(5)*cos(q(2))*cos(q(3)) + cos(q(2))*sin(q(3))*sin(q(4))*...
         (p(6)^2 + p(7)^2)^(1/2) - cos(q(2))*cos(q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2);

 