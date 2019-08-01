function sys = delftbikeHeading(v)

% Coefficient matrices;

load('JBike6MCK.mat', 'C1', 'M0', 'K2', 'K0')

a = -fliplr(eye(2)) + eye(2);
M0 = a .* M0;
C1 = C1 .* a;
K0 = K0 .* a;
K2 = K2 .* a;
Hfw = [0.84; 0.014408]; % dfx/dTq


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
