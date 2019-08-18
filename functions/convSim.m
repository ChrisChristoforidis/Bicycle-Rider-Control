function y= convSim (ww,np)
% Convolves the IRF with the given input disturbance to produce the black
% box model output.
%         Inputs
%               ww : input disturbance signal
%               np : non parametric model output
%           Outputs
%               y : steer and roll angle output

dt=0.001;
M = length(np.h); % length of IRF
tau = (0:M - 1)' * dt;

w = [zeros(M-1, 1); ww];

y = zeros(length(ww), size(np.h,2));

for ii = 1:size(np.h,2)
  ht = np.h(1:M, ii); % IRF
  % h(tau) * u(t-tau)
  for loop = 0:length(ww) - 1
    u_h = w(1+loop:M+loop);
    z_h = u_h .* flipud(ht);
    y(loop+1, ii) = sum(z_h) * dt;
  end
  
end

end