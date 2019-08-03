function sys = delftbikeHeading(v)

% Coefficient matrices;

whipple=getbicycleEOM();
steerbyWireParam
O = zeros(2);
I = eye(2); % Some easy notations

% State Space description of uncontrolled bicycle
A = [-whipple.M0 \ whipple.C1 * v, -whipple.M0 \ (whipple.K0 + whipple.K2 * v^2); I, O];
B = [whipple.M0 \ [I, whipple.Hfw]; zeros(2, 3)];

E=[0 trail*cos(lambda)/wheelbase 0 v*cos(lambda)/wheelbase];
NA=[A;E];
NA=[NA zeros(5,1)];
NB=[ B ;zeros(1,size(B,2))];
NC=eye(5) ;
ND=zeros(5,3);
% Combine A,B,C and D matrices into a state space object.
sys = ss(NA, NB, NC, ND);

end
