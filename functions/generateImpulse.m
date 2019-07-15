function [t,w]=generateImpulse(m,d)


t=[0:0.001:d].';
w=smooth([zeros(200,1);(m*2/d)*t(1:round(end/2));-(m*2/d)*t(round(end/2)+1:end)+2*m;zeros(2500,1)],0.01);
t=[0:0.001:(length(w)-1)*0.001].';
end