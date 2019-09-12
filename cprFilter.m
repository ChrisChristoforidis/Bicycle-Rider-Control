function x = cprFilter(u,z,a)
% inputs
%   u:  signal to be integrated
%   z:  pseudo-absolute measurements
%   a: complimentary filter coeficient
% outputs
%   x: filtered signal
x=zeros(length(u),1);
x(1)=mean(z(1:10));
for i =1:length(u)-1
  x(i+1)=(1-a)*(x(i)+u(i)/1000)+a*z(i);
end

end