function [rm]=rotmat(uv,angle);
s=[0 -uv(3) uv(2);uv(3) 0 -uv(1);-uv(2) uv(1) 0];
rm=cos(angle)*eye(3)+(1-cos(angle))*uv*uv'+sin(angle)*s;
    
