function [y,extra] = KalmanFuse(u,z,Fs)
% inputs
%   y:  signal to be integrated
%   z:  pseudo-absolute measurements
%   Fs: sampling frequency of signals
% outputs
%   x: filtered signal
%   extra: Includes Covariance , Kalman Gains and bias

dt=1/Fs;
N=length(u);
% Plant Discription
F = [1, -dt; 0, 1];
H = [1, 0];
G = [dt; 0];
% Process And Measurement Noise
Qp = [9E-04, 0; 0, 3E-04]*dt;
Qs = 0.5;

%Initialization of some cells and matrices

Pk = cell(N, 1); %covariance matrix showing uncertainty in model output
K  = cell(N, 1); %kalman gains
X  = cell(N, 1); %State containing the integrated input signal and the bias
ok = zeros(N, 1); %Estimation  based on the absolute 'measurements'
yk = zeros(N-1, 1); %Mismatch  between the expected 'sensor readings' and the model output
Sk = zeros(N-1, 1); %Uncertainty in the system state projected via the sensor function
U = u; % input of the system is the roll rate
Ig = [1, 0; 0, 1];

%Intial Conditions

Pk{1} = [0.1, 0; 0, 0.1]; %zero covariance matrix means we put 100% in the model output at the start
X{1} = [mean(z(1:10)); 0]; % zero roll angle at the start and zero bias (no integration error)


% Combined State Estimation with Kalman Filter implementation
for jj = 1:N - 1
  %Prediction Step
  X{jj+1} = F * X{jj} + G * U(jj);
  Pk{jj+1} = F * Pk{jj} * F.' +Qp;
  

  % Pseudo-absolute Measurments
  ok(jj+1) = z(jj+1);

  %KALMAN GAIN
  yk(jj) = ok(jj+1) - H * X{jj+1};
  Sk(jj) = H * Pk{jj+1} * H.' +Qs;
  K{jj+1} = Pk{jj+1} * H.' / Sk(jj);
  %UPDATE STEP
  X{jj+1} = X{jj+1} + K{jj+1} * yk(jj);
  Pk{jj+1} = (Ig - K{jj+1} * H) * Pk{jj+1};
end

y = zeros(N, 1);
for jj = 1:N
  y(jj) = X{jj}(1);
end

bias = zeros(N, 1);
for jj = 1:N
  bias(jj) = X{jj}(2);
end

KALMAN = zeros(N-1, 2);
for jj = 2:N
  KALMAN(jj, :) = K{jj}(1:2);
end

COV = zeros(N-1, 4);
for jj = 2:N
  COV(jj, 1:2) = Pk{jj}(1:2);
  COV(jj, 3:4) = Pk{jj}(3:4);
  
end



extra.bias = bias;
extra.KALMAN = KALMAN;
extra.COV = COV;

end