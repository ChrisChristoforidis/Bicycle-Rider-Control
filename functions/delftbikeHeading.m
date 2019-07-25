function sys = delftbikeHeading(v,whipple)

% Coefficient matrices;

M0 = whipple.M0;
C1 = whipple.C1;
K0 = whipple.K0;
Hfw = whipple.Hfw;
K2 = whipple.K2;
trail=0.0665;
wheelbase=1.03;
tilt=(17)*pi/180;

O = zeros(2);
I = eye(2); % Some easy notations

% State Space description of uncontrolled bicycle
A = [-M0 \ C1 * v, -M0 \ (K0 + K2 * v^2); I, O];
B = [M0 \ [I, Hfw]; zeros(2, 3)];

E=[0 trail*cos(tilt)/wheelbase 0 tilt*cos(tilt)/wheelbase];
NA=[A;E];
NA=[NA zeros(5,1)];
NB=[ B ;zeros(1,size(B,2))];
NC=eye(5) ;
ND=zeros(5,3);
% Combine A,B,C and D matrices into a state space object.
sys = ss(NA, NB, NC, ND);

end
