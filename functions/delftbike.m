function sys = delftbike(v,whipple)

% Coefficient matrices;

M0 = whipple.M0;
C1 = whipple.C1;
K0 = whipple.K0;
Hfw = whipple.Hfw;
K2 = whipple.K2;


O = zeros(2);
I = eye(2); % Some easy notations

% State Space description of uncontrolled bicycle
A = [-M0 \ C1 * v, -M0 \ (K0 + K2 * v^2); I, O];
B = [M0 \ [I, Hfw]; zeros(2, 3)];
C = eye(4);
D = zeros(4, 3);

% Combine A,B,C and D matrices into a state space object.
sys = ss(A, B, C, D);

end
