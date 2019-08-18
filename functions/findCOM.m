function [COMx,COMy,idx]=findCOM(x,y,a,b)

%Find COM of area inscribed by signal y. X denotes the signal that y
%depends on (y(x)). a and b are the boundaries of the area.

     idx=find(x>=a & x<=b);
     COMx=trapz(x(idx),x(idx).*y(idx))/trapz(x(idx),y(idx));
     COMy=trapz(x(idx),0.5*y(idx).^2)/trapz(x(idx),y(idx));


end

