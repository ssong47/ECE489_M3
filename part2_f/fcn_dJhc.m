function [dJhc] = fcn_dJhc(q,dq,pt,p)

dJhc = zeros(2,4);

  dJhc(1,1)=- dq(3)*((pt(1)*(p(5)*cos(q(3))*sin(q(1)) + cos(q(4))*(cos(q(3))*sin(q(1)) - cos(q(1))*...
         sin(q(2))*sin(q(3)))*(p(6)^2 + p(7)^2)^(1/2) - sin(q(4))*(sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(3))*...
         sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2) - p(5)*cos(q(1))*sin(q(2))*sin(q(3))))/(abs(pt(1))^2 +...
          abs(pt(2))^2)^(1/2) - (pt(2)*(p(5)*cos(q(1))*cos(q(3)) + cos(q(4))*(cos(q(1))*cos(q(3)) + sin(q(1))*sin(q(2))*sin(q(3)))*...
         (p(6)^2 + p(7)^2)^(1/2) - sin(q(4))*(cos(q(1))*sin(q(3)) - cos(q(3))*sin(q(1))*sin(q(2)))*(p(6)^2 +...
          p(7)^2)^(1/2) + p(5)*sin(q(1))*sin(q(2))*sin(q(3))))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2)) - dq(2)*((pt(1)*(p(3)*...
         cos(q(1))*sin(q(2)) + p(5)*cos(q(1))*cos(q(2))*cos(q(3)) + cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*(p(6)^2 +...
          p(7)^2)^(1/2) - cos(q(1))*cos(q(2))*sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2) +...
          (pt(2)*(p(3)*sin(q(1))*sin(q(2)) + p(5)*cos(q(2))*cos(q(3))*sin(q(1)) + cos(q(2))*cos(q(3))*cos(q(4))*...
         sin(q(1))*(p(6)^2 + p(7)^2)^(1/2) - cos(q(2))*sin(q(1))*sin(q(3))*sin(q(4))*(p(6)^2 +...
          p(7)^2)^(1/2)))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2)) - dq(4)*((pt(1)*(cos(q(4))*(cos(q(3))*sin(q(1)) - cos(q(1))*sin(q(2))*...
         sin(q(3)))*(p(6)^2 + p(7)^2)^(1/2) - sin(q(4))*(sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)))*(p(6)^2 +...
          p(7)^2)^(1/2)))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2) - (pt(2)*(cos(q(4))*(cos(q(1))*cos(q(3)) + sin(q(1))*sin(q(2))*sin(q(3)))*...
         (p(6)^2 + p(7)^2)^(1/2) - sin(q(4))*(cos(q(1))*sin(q(3)) - cos(q(3))*sin(q(1))*sin(q(2)))*(p(6)^2 +...
          p(7)^2)^(1/2)))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2)) - dq(1)*((pt(1)*(p(4)*cos(q(1)) + p(3)*cos(q(2))*sin(q(1)) + p(5)*cos(q(1))*...
         sin(q(3)) + cos(q(4))*(cos(q(1))*sin(q(3)) - cos(q(3))*sin(q(1))*sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2) +...
          sin(q(4))*(cos(q(1))*cos(q(3)) + sin(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 + p(7)^2)^(1/2) - p(5)*cos(q(3))*...
         sin(q(1))*sin(q(2))))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2) + (pt(2)*(p(4)*sin(q(1)) - p(3)*cos(q(1))*...
         cos(q(2)) + p(5)*sin(q(1))*sin(q(3)) + cos(q(4))*(sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)))*...
         (p(6)^2 + p(7)^2)^(1/2) + sin(q(4))*(cos(q(3))*sin(q(1)) - cos(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 +...
          p(7)^2)^(1/2) + p(5)*cos(q(1))*cos(q(3))*sin(q(2))))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2));
  dJhc(1,2)=dq(4)*((pt(1)*(cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) +...
          cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 +...
          abs(pt(2))^2)^(1/2) - (pt(2)*(cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + cos(q(1))*cos(q(2))*cos(q(4))*...
         sin(q(3))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2)) - dq(1)*((pt(1)*(p(3)*cos(q(1))*...
         sin(q(2)) + p(5)*cos(q(1))*cos(q(2))*cos(q(3)) + cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*(p(6)^2 +...
          p(7)^2)^(1/2) - cos(q(1))*cos(q(2))*sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2) +...
          (pt(2)*(p(3)*sin(q(1))*sin(q(2)) + p(5)*cos(q(2))*cos(q(3))*sin(q(1)) + cos(q(2))*cos(q(3))*cos(q(4))*...
         sin(q(1))*(p(6)^2 + p(7)^2)^(1/2) - cos(q(2))*sin(q(1))*sin(q(3))*sin(q(4))*(p(6)^2 +...
          p(7)^2)^(1/2)))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2)) + dq(2)*((pt(2)*(p(3)*cos(q(1))*cos(q(2)) - p(5)*cos(q(1))*cos(q(3))*...
         sin(q(2)) - cos(q(1))*cos(q(3))*cos(q(4))*sin(q(2))*(p(6)^2 + p(7)^2)^(1/2) + cos(q(1))*sin(q(2))*sin(q(3))*sin(q(4))*...
         (p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2) - (pt(1)*(p(3)*cos(q(2))*sin(q(1)) - p(5)*...
         cos(q(3))*sin(q(1))*sin(q(2)) - cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2))*(p(6)^2 + p(7)^2)^(1/2) + sin(q(1))*...
         sin(q(2))*sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2)) - dq(3)*((pt(2)*...
         (p(5)*cos(q(1))*cos(q(2))*sin(q(3)) + cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) +...
          cos(q(1))*cos(q(2))*cos(q(4))*sin(q(3))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 +...
          abs(pt(2))^2)^(1/2) - (pt(1)*(p(5)*cos(q(2))*sin(q(1))*sin(q(3)) + cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4))*(p(6)^2 +...
          p(7)^2)^(1/2) + cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2));
  dJhc(1,3)=- dq(3)*((pt(1)*(p(5)*cos(q(1))*sin(q(3)) + cos(q(4))*(cos(q(1))*sin(q(3)) - cos(q(3))*...
         sin(q(1))*sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2) + sin(q(4))*(cos(q(1))*cos(q(3)) + sin(q(1))*sin(q(2))*...
         sin(q(3)))*(p(6)^2 + p(7)^2)^(1/2) - p(5)*cos(q(3))*sin(q(1))*sin(q(2))))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2) +...
          (pt(2)*(p(5)*sin(q(1))*sin(q(3)) + cos(q(4))*(sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)))*...
         (p(6)^2 + p(7)^2)^(1/2) + sin(q(4))*(cos(q(3))*sin(q(1)) - cos(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 +...
          p(7)^2)^(1/2) + p(5)*cos(q(1))*cos(q(3))*sin(q(2))))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2)) - dq(1)*((pt(1)*(p(5)*...
         cos(q(3))*sin(q(1)) + cos(q(4))*(cos(q(3))*sin(q(1)) - cos(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 +...
          p(7)^2)^(1/2) - sin(q(4))*(sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2) - p(5)*cos(q(1))*...
         sin(q(2))*sin(q(3))))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2) - (pt(2)*(p(5)*cos(q(1))*cos(q(3)) + cos(q(4))*...
         (cos(q(1))*cos(q(3)) + sin(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 + p(7)^2)^(1/2) - sin(q(4))*(cos(q(1))*...
         sin(q(3)) - cos(q(3))*sin(q(1))*sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2) + p(5)*sin(q(1))*sin(q(2))*sin(q(3))))/(abs(pt(1))^2 +...
          abs(pt(2))^2)^(1/2)) - dq(2)*((pt(2)*(p(5)*cos(q(1))*cos(q(2))*sin(q(3)) + cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4))*(p(6)^2 +...
          p(7)^2)^(1/2) + cos(q(1))*cos(q(2))*cos(q(4))*sin(q(3))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 +...
          abs(pt(2))^2)^(1/2) - (pt(1)*(p(5)*cos(q(2))*sin(q(1))*sin(q(3)) + cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4))*(p(6)^2 +...
          p(7)^2)^(1/2) + cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 +...
          abs(pt(2))^2)^(1/2)) - dq(4)*((pt(1)*(cos(q(4))*(cos(q(1))*sin(q(3)) - cos(q(3))*sin(q(1))*sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2) +...
          sin(q(4))*(cos(q(1))*cos(q(3)) + sin(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 +...
          abs(pt(2))^2)^(1/2) + (pt(2)*(cos(q(4))*(sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2) +...
          sin(q(4))*(cos(q(3))*sin(q(1)) - cos(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2));
  dJhc(1,4)=dq(2)*((pt(1)*(cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) +...
          cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 +...
          abs(pt(2))^2)^(1/2) - (pt(2)*(cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + cos(q(1))*cos(q(2))*cos(q(4))*...
         sin(q(3))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2)) - dq(3)*((pt(1)*(cos(q(4))*...
         (cos(q(1))*sin(q(3)) - cos(q(3))*sin(q(1))*sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2) + sin(q(4))*(cos(q(1))*...
         cos(q(3)) + sin(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2) +...
          (pt(2)*(cos(q(4))*(sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2) +...
          sin(q(4))*(cos(q(3))*sin(q(1)) - cos(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 +...
          abs(pt(2))^2)^(1/2)) - dq(1)*((pt(1)*(cos(q(4))*(cos(q(3))*sin(q(1)) - cos(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 +...
          p(7)^2)^(1/2) - sin(q(4))*(sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 +...
          abs(pt(2))^2)^(1/2) - (pt(2)*(cos(q(4))*(cos(q(1))*cos(q(3)) + sin(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 +...
          p(7)^2)^(1/2) - sin(q(4))*(cos(q(1))*sin(q(3)) - cos(q(3))*sin(q(1))*sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 +...
          abs(pt(2))^2)^(1/2)) - dq(4)*((pt(1)*(cos(q(4))*(cos(q(1))*sin(q(3)) - cos(q(3))*sin(q(1))*sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2) +...
          sin(q(4))*(cos(q(1))*cos(q(3)) + sin(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 +...
          abs(pt(2))^2)^(1/2) + (pt(2)*(cos(q(4))*(sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)))*(p(6)^2 + p(7)^2)^(1/2) +...
          sin(q(4))*(cos(q(3))*sin(q(1)) - cos(q(1))*sin(q(2))*sin(q(3)))*(p(6)^2 + p(7)^2)^(1/2)))/(abs(pt(1))^2 + abs(pt(2))^2)^(1/2));
  dJhc(2,1)=0;
  dJhc(2,2)=dq(2)*(p(3)*sin(q(2)) + p(5)*cos(q(2))*cos(q(3)) - cos(q(2))*sin(q(3))*sin(q(4))*...
         (p(6)^2 + p(7)^2)^(1/2) + cos(q(2))*cos(q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2)) - dq(3)*(p(5)*sin(q(2))*...
         sin(q(3)) + cos(q(3))*sin(q(2))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + cos(q(4))*sin(q(2))*sin(q(3))*(p(6)^2 +...
          p(7)^2)^(1/2)) - dq(4)*(cos(q(3))*sin(q(2))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + cos(q(4))*sin(q(2))*sin(q(3))*(p(6)^2 + p(7)^2)^(1/2));
  dJhc(2,3)=dq(3)*(p(5)*cos(q(2))*cos(q(3)) - cos(q(2))*sin(q(3))*sin(q(4))*(p(6)^2 +...
          p(7)^2)^(1/2) + cos(q(2))*cos(q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2)) - dq(4)*(cos(q(2))*sin(q(3))*sin(q(4))*...
         (p(6)^2 + p(7)^2)^(1/2) - cos(q(2))*cos(q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2)) - dq(2)*(p(5)*sin(q(2))*...
         sin(q(3)) + cos(q(3))*sin(q(2))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + cos(q(4))*sin(q(2))*sin(q(3))*(p(6)^2 + p(7)^2)^(1/2));
  dJhc(2,4)=- dq(3)*(cos(q(2))*sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) - cos(q(2))*cos(q(3))*...
         cos(q(4))*(p(6)^2 + p(7)^2)^(1/2)) - dq(4)*(cos(q(2))*sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) - cos(q(2))*...
         cos(q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2)) - dq(2)*(cos(q(3))*sin(q(2))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) +...
          cos(q(4))*sin(q(2))*sin(q(3))*(p(6)^2 + p(7)^2)^(1/2));

 