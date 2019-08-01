function  simAnim(dt,K,v,w,varargin)
% Simulates and animates the reaction of the rider-bicycle model to the
% disturbance signal w. Simulation uses RK4 with specified time step dt.
% Inputs 
%     dt      :   Integration timestep
%     K       :   The gains of the controller
%     whipple :   The parameters of the whipple model
%     v       :   The speed of the bicycle in m/s  (scalar)
%     w       :   The disturbance input (signal)
%    varargin :   camera view
% Outputs
%     F       :   Array with animation frames saved
%     Y       :   Output of the simulation (all 9 states)
% state=[dphi,ddelta,phi,delta,psi,Tdelta,dTdelta,xp,yp];

bike=delftbikeHeading(v);
omegac= 2 * pi * 2.17;
Gp=plantModel(bike,omegac);

y0=zeros(9,1); %initial condition

nstep = floor((length(w)*0.001-0)/dt);
sim=y0;
te=0;


%bike
wb=1.03;%m
lamda=pi/10;%rad
CD=0.75;
e3=[0;0;1];
e2=[0;1;0];
e1=[1;0;0];
radr=0.6858/2; radf=0.6858/2;  %m
theta=linspace(0,2*pi,60);%rad
f=figure();
tic
count=0;
for i = 1: nstep
we=w(floor((te+0.001)*1000)+1);
[te,sim]=rk4step(te,sim,dt,v,Gp,K,we);
nw=we/max(abs(w));
count=count+1;
if(count>4)
  count=0;
  thr=te*v/radr;
  r1=rotmat(-e3,sim(5));r2=rotmat(r1*e1,sim(3));rrf=r2*r1;
  r3=rotmat(e2,-lamda);
  r4=rotmat(r3*(-e3),sim(4));
  q2=r4*r3;
  q=rrf;
  rrw=rotmat(e2,thr);

  x=sim(8);
  y=sim(9);
  z=0;
  xr=radr*cos(theta);zr=radr*sin(theta)+radr; yr=0*theta;
  xf=radf*cos(theta)+wb;zf=radf*sin(theta)+radf;yf=0*theta;
  c1=rrw*[xr;yr;zr-radr];
  c1=q*[c1(1,:) ;c1(2,:); c1(3,:)+radr]; c2m=rotmat(q2*e2,thr)*q2*[xf-wb;yf;zf-radf];c2=q*[c2m(1,:)+wb;c2m(2,:);c2m(3,:)+radf];
  c1=[c1(1,:)+x ; c1(2,:)+y ; c1(3,:)+z];c2=[c2(1,:)+x ; c2(2,:)+y ; c2(3,:)+z];

  %Segment 1 (fork)
  p1s=[wb;0;radf];

  p1e=[-sin(lamda)*CD+wb;0;cos(lamda)*CD+radf];

  s1=q*[p1s p1e];
  s1=[s1(1,:)+x ; s1(2,:)+y; s1(3,:)+z];
  %Segment 2 (frame)

  p2s=[0;0;0+radr];

  p2e=[-sin(lamda)*3/4*CD+wb;0;cos(lamda)*3/4*CD+radr];

  s2=q*[p2s p2e];
  s2=[s2(1,:)+x ; s2(2,:)+y ; s2(3,:)+z];

  %Segment 3 (saddle frame)

  p3s=[-sin(lamda)*CD/4+wb/3;0;cos(lamda)*CD/4+radr];

  p3e=p3s;
  p3e(3)=p3s(3)+0.3;

  s3=q*[p3s p3e];
  s3=[s3(1,:)+x ; s3(2,:)+y ; s3(3,:)+z];

  %Segment 4 (handlebar)

  p4s=[p1e(1)-0.1;p1e(2)-0.3;p1e(3)];

  p4l=[p1e(1);p1e(2)-0.3;p1e(3)];

  p4r=[p1e(1); p1e(2)+0.3 ;p1e(3)];

  p4e=[p1e(1)-0.1; p1e(2)+0.3 ;p1e(3)];

  s4=[p4s p4l p4r p4e];
  s4(3,:)=s4(3,:)-CD*cos(lamda)-radr;
  s4(2,:)=s4(2,:);
  s4(1,:)=s4(1,:)+CD*sin(lamda)-wb;
  s4=q2*s4;
  s4(3,:)=s4(3,:)+CD*cos(lamda)+radr;
  s4(2,:)=s4(2,:);
  s4(1,:)=s4(1,:)-CD*sin(lamda)+wb;
  s4=q*s4;
  s4=[s4(1,:)+x ; s4(2,:)+y ; s4(3,:)+z];

  %Segment 5 (saddle)

  p5s=[p3e(1)-0.03;p3e(2)-0.07;p3e(3)];

  p5m=[p3e(1)+0.15; p3e(2); p3e(3)];

  p5e=[p3e(1)-0.03; p3e(2)+0.07; p3e(3)];

  s5=q*[p5s p5m p5e];
  s5=[s5(1,:)+x ; s5(2,:)+y ; s5(3,:)+z];

  %Segment 6 (force vector)
  p6s=p3e;

  p6m=p6s+[0;-1*nw-0.01;0];
  p6l=p6m+[0.1*nw;0.08*nw;0];
  p6r=p6m+[-0.1*nw;0.08*nw;0];

  s6=q*[p6s p6m p6l p6r p6m];
  s6=[s6(1,:)+x ; s6(2,:)+y ; s6(3,:)+z];

  %Segment 7 ( wheel rods)
   o7r=zeros(3,6);
   p7r=q*[o7r(1,:) ; o7r(2,:)   ;    o7r(3,:)+radr];
   s7r=[p7r(1,:)+x ; p7r(2,:)+y ;    p7r(3,:)+z   ];

   p7f=q*[o7r(1,:)+wb ; o7r(2,:)   ;    o7r(3,:)+radr];
   s7f=[p7f(1,:)+x ; p7f(2,:)+y ;    p7f(3,:)+z   ];
  subplot(3,2,[2 4 6])

  plot3(s1(1,:),s1(2,:),s1(3,:),'b','LineWidth',3)
  hold on
  box on
  patch(c1(1,1:end),c1(2,1:end),c1(3,1:end),'cyan','LineWidth',2)
  patch(c2(1,1:end),c2(2,1:end),c2(3,1:end),'cyan','LineWidth',2)
  plot3(s2(1,:),s2(2,:),s2(3,:),'b','LineWidth',3)
  plot3(s3(1,:),s3(2,:),s3(3,:),'b','LineWidth',3)
  plot3(s4(1,:),s4(2,:),s4(3,:),'b','LineWidth',3)
  for jj=10:10:60
    plot3([s7r(1,jj/10) c1(1,jj)],[s7r(2,jj/10) c1(2,jj)],[s7r(3,jj/10) c1(3,jj)],'b','LineWidth',2)
    plot3([s7f(1,jj/10) c2(1,jj)],[s7f(2,jj/10) c2(2,jj)],[s7f(3,jj/10) c2(3,jj)],'b','LineWidth',2)
  end
  patch(s5(1,:),s5(2,:),s5(3,:),'cyan','LineWidth',3,'EdgeColor','blue')
  patch(s6(1,:),s6(2,:),s6(3,:),'r','LineWidth',2.5,'EdgeColor','red')
  %plot3(sim(1:i,8),sim(1:i,9),zeros(i,1),'m','LineWidth',1.5)
  if (isempty(varargin))
    view(60,30)
  elseif(varargin{1}=="topdown")
     view(-90,72)
  elseif (varargin{1}=="back")
     view(-90,0)
  else
     view(60,30)
  end
  ylim([y-1.2 y+0.7])
  xlim([x-1 x+2.5])
  zlim([0 1.3])
  ax=gca;
  ax.XGrid='on';

  set(gca,'DataAspectRatio',[1 1 1])
  hold off

  subplot(321)
  plot(te,we,'b.');
  hold on;
  grid on;
  ylabel('w (N)');
  subplot(323)
  plot(te,sim(4)*180/pi,'b.');
  hold on;
  grid on;
  ylabel('Steer Angle (deg)');
  subplot(325)
  plot(te,sim(3)*180/pi,'b.');
  hold on;
  grid on;
  ylabel('Roll Angle (deg)');
  drawnow limitrate
end

end
toc
te
end