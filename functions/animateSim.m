n=1;

bike=delftbikeHeading(data(n).v,whipple);
omegac= 2 * pi * 2.17;
Gp=plantModel(bike,omegac);

y0=zeros(9,1); %initial condition


%% RK 4  SIMULATION
tic
dt = 0.01; 
nstep = floor((dat.t(end)-0)/dt);
ye=y0;
te=0;
y=zeros(nstep,9);
t=zeros(nstep,1);
w=zeros(nstep,1);

y(1,:)=ye;

for i=2:nstep
  %w=dat.w(floor(i*dt*dat.Fs));
  [te,ye,we]=rk4step(te,ye,dt,data(n).v,Gp,final_model(n).K,data(n).w);
  w(i,:)=we;
  y(i,:)=ye;
  t(i)=te;
end
toc
%%
% theta=linspace(0,4*pi,length(y));
% k=zeros(length(y),9);
% k(:,4)=theta.';
%%
F=animate(y,length(y),0.01,w,data(n).v);

%%
v = VideoWriter('mean_9.avi');
open(v);
writeVideo(v,F)
close(v)
%%
%%
% rotcircle=q*periphery;
% 
% 
% figure;hold on;
% plot3([origin(1) point1(1)],[origin(2) point1(2)],[origin(3) point1(3)],'LineWidth',5);
% plot3([origin(1) point2(1)],[origin(2) point2(2)],[origin(3) point2(3)],'LineWidth',5);
% plot3([origin(1) point3(1)],[origin(2) point3(2)],[origin(3) point3(3)],'LineWidth',5);
% grid on;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% %plot3(rotcircle(1,:),rotcircle(2,:),rotcircle(3,:))
% hold on
% plot3(periphery(1,:),periphery(2,:),periphery(3,:))
% plot3(rotcircle(1,:),rotcircle(2,:),rotcircle(3,:))
% 
% grid on
% point1 = [1,0,0];
% point2 = [0,1,0];
% point3 = [0 0 1];
% origin = [0,0,0];
% axis equal
% set(gca,'CameraPosition',[2 2 2]);


%%
n=1;
w=rand(10000,1);
w=(w-mean(w))*1875;
simAnim(0.01,final_model(n).K,whipple,data(n).v,data(n).w)
%%
function F=animate(sim,nstep,dt,w,v)

% flat plane
[xp ,yp] = meshgrid(-10:2:round( max(sim(:,8))/ 2 )*2); % Generate x and y data
zp = zeros(size(xp, 1)); % Generate z data
%bike
wb=1.03;%m
lamda=pi/10;%rad
CD=0.75;
e3=[0;0;1];
e2=[0;1;0];
e1=[1;0;0];
radr=0.6858/2; radf=0.6858/2;  %m
theta=linspace(0,2*pi,60);%rad
nw=(w-min(w))/(max(w)-min(w));
f=figure('units','normalized','outerposition',[0 0 1 1]);
%surf(xp, yp, zp) % Plot the surface
start=1000;
F(length(start:4:nstep)) = struct('cdata',[],'colormap',[]);
loop=1;
for i = start:4: nstep
thr=i*dt*v/radr;
r1=rotmat(-e3,sim(i,5));r2=rotmat(r1*e1,sim(i,3));rrf=r2*r1;
r3=rotmat(e2,-lamda);
r4=rotmat(r3*(-e3),sim(i,4));
q2=r4*r3;
q=rrf;
rrw=rotmat(e2,thr);

x=sim(i,8);
y=sim(i,9);
z=0;
xr=radr*cos(theta);zr=radr*sin(theta)+radr; yr=0*theta;
xf=radf*cos(theta)+wb;zf=radf*sin(theta)+radf;yf=0*theta;
c1=rrw*[xr;yr;zr-radr];
c1=q*[c1(1,:) ;c1(2,:); c1(3,:)+radr]; c2m=rotmat(q2*(-e2),thr)*q2*[xf-wb;yf;zf-radf];c2=q*[c2m(1,:)+wb;c2m(2,:);c2m(3,:)+radf];
c1=[c1(1,:)+x ; c1(2,:)+y ; c1(3,:)+z];c2=[c2(1,:)+x ; c2(2,:)+y ; c2(3,:)+z];

%Segment 1 (fork)
p1s=[wb;0;radf];

p1e=[-sin(lamda)*CD+wb;0;cos(lamda)*CD+radf];

s1=q*[p1s p1e];
s1=[s1(1,:)+x ; s1(2,:)+y; s1(3,:)+z];
%Segment 2 (frame)

p2s=[0;0;0+radr];

p2e=[0-sin(lamda)*3/4*CD+wb;0;0+cos(lamda)*3/4*CD+radr];

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

p6m=p6s+[0;-1*nw(i)-0.01;0];
p6l=p6m+[0.1*nw(i);0.08*nw(i);0];
p6r=p6m+[-0.1*nw(i);0.08*nw(i);0];

s6=q*[p6s p6m p6l p6r p6m];
s6=[s6(1,:)+x ; s6(2,:)+y ; s6(3,:)+z];

%Segment 7 ( wheel rods)
 o7r=zeros(3,6);
 p7r=q*[o7r(1,:) ; o7r(2,:)   ;    o7r(3,:)+radr];
 s7r=[p7r(1,:)+x ; p7r(2,:)+y ;    p7r(3,:)+z   ];
 
 p7f=q*[o7r(1,:)+wb ; o7r(2,:)   ;    o7r(3,:)+radr];
 s7f=[p7f(1,:)+x ; p7f(2,:)+y ;    p7f(3,:)+z   ];

 % plots
% subplot(221)
% plot3(s1(1,:),s1(2,:),s1(3,:),'b','LineWidth',3)
% hold on
% patch(c1(1,:),c1(2,:),c1(3,:),'cyan','LineWidth',2)
% patch(c2(1,:),c2(2,:),c2(3,:),'cyan','LineWidth',2)
% plot3(s2(1,:),s2(2,:),s2(3,:),'b','LineWidth',3)
% plot3(s3(1,:),s3(2,:),s3(3,:),'b','LineWidth',3)
% plot3(s4(1,:),s4(2,:),s4(3,:),'b','LineWidth',3)
% patch(s5(1,:),s5(2,:),s5(3,:),'cyan','LineWidth',3,'EdgeColor','blue')
% patch(s6(1,:),s6(2,:),s6(3,:),'r','LineWidth',2.5,'EdgeColor','red')
% xlim([x-1 x+1])
% ylim([y-1 y+1])
% zlim([0 1.5])
% %axis equal
% view(-90,0)
% set(gca,'DataAspectRatio',[1 1 1])
% hold off
% 
% subplot(222)
% plot3(s1(1,:),s1(2,:),s1(3,:),'b','LineWidth',3)
% hold on
% patch(c1(1,:),c1(2,:),c1(3,:),'cyan','LineWidth',2)
% patch(c2(1,:),c2(2,:),c2(3,:),'cyan','LineWidth',2)
% plot3(s2(1,:),s2(2,:),s2(3,:),'b','LineWidth',3)
% plot3(s3(1,:),s3(2,:),s3(3,:),'b','LineWidth',3)
% plot3(s4(1,:),s4(2,:),s4(3,:),'b','LineWidth',3)
% patch(s5(1,:),s5(2,:),s5(3,:),'cyan','LineWidth',3,'EdgeColor','blue')
% patch(s6(1,:),s6(2,:),s6(3,:),'r','LineWidth',2.5,'EdgeColor','red')
% surf(xp, yp, zp,'FaceColor' ,'#D1736B') 
% plot3(sim(1:i,8),sim(1:i,9),zeros(i,1),'m','LineWidth',1.5)
% plot3(xp(1,:),ones(length(xp),1)*2.5,zeros(length(xp),1),'k','LineWidth',15);
% plot3(xp(1,:),-ones(length(xp),1)*2.5,zeros(length(xp),1),'k','LineWidth',15);
% plot3(xp(1,:),zeros(length(xp),1),zeros(length(xp),1),'w--','LineWidth',5);
% hold off
% 
% xlim([x-2 x+2])
% ylim([-2.55 2.55])
% zlim([0 1.5])
% %axis equal
% view(-90,90)
% set(gca,'DataAspectRatio',[1 1 1])
% 
% subplot(2,2,3)
% plot3(s1(1,:),s1(2,:),s1(3,:),'b','LineWidth',3)
% hold on
% box on
% patch(c1(1,:),c1(2,:),c1(3,:),'cyan','LineWidth',2)
% patch(c2(1,:),c2(2,:),c2(3,:),'cyan','LineWidth',2)
% plot3(s2(1,:),s2(2,:),s2(3,:),'b','LineWidth',3)
% for jj=10:10:60
%   plot3([s7r(1,jj/10) c1(1,jj)],[s7r(2,jj/10) c1(2,jj)],[s7r(3,jj/10) c1(3,jj)],'b','LineWidth',2)
%   plot3([s7f(1,jj/10) c2(1,jj)],[s7f(2,jj/10) c2(2,jj)],[s7f(3,jj/10) c2(3,jj)],'b','LineWidth',2)
% end
% plot3(s3(1,:),s3(2,:),s3(3,:),'b','LineWidth',3)
% plot3(s4(1,:),s4(2,:),s4(3,:),'b','LineWidth',3)
% patch(s5(1,:),s5(2,:),s5(3,:),'cyan','LineWidth',3,'EdgeColor','blue')
% patch(s6(1,:),s6(2,:),s6(3,:),'r','LineWidth',2.5,'EdgeColor','red')
% surf(xp, yp, zp,'FaceColor' ,'#D1736B') 
% plot3(sim(1:i,8),sim(1:i,9),zeros(i,1),'m','LineWidth',1.5)
% plot3(xp(1,:),ones(length(xp),1)*2.5,zeros(length(xp),1),'k','LineWidth',15);
% plot3(xp(1,:),-ones(length(xp),1)*2.5,zeros(length(xp),1),'k','LineWidth',15);
% plot3(xp(1,:),zeros(length(xp),1),zeros(length(xp),1),'w--','LineWidth',5);
% 
% xlim([x-10 x+2])
% ylim([-2.55 2.55])
% zlim([0 1.5])
% %axis equal
% %view(-60,62)
% set(gca,'DataAspectRatio',[1 1 1])
% hold off
% 
% subplot(2,2,4)
plot3(s1(1,:),s1(2,:),s1(3,:),'b','LineWidth',3)
hold on
box on
patch(c1(1,1:end),c1(2,1:end),c1(3,1:end),'cyan','LineWidth',3)
patch(c2(1,1:end),c2(2,1:end),c2(3,1:end),'cyan*','LineWidth',2)
plot3(s2(1,:),s2(2,:),s2(3,:),'b','LineWidth',3)
plot3(s3(1,:),s3(2,:),s3(3,:),'b','LineWidth',3)
plot3(s4(1,:),s4(2,:),s4(3,:),'b','LineWidth',3)
for jj=10:10:60
  plot3([s7r(1,jj/10) c1(1,jj)],[s7r(2,jj/10) c1(2,jj)],[s7r(3,jj/10) c1(3,jj)],'b','LineWidth',2)
  plot3([s7f(1,jj/10) c2(1,jj)],[s7f(2,jj/10) c2(2,jj)],[s7f(3,jj/10) c2(3,jj)],'b','LineWidth',2)
end
patch(s5(1,:),s5(2,:),s5(3,:),'cyan','LineWidth',3,'EdgeColor','blue')
patch(s6(1,:),s6(2,:),s6(3,:),'r','LineWidth',2.5,'EdgeColor','red')
surf(xp, yp, zp,'FaceColor' ,'#D1736B') 
plot3(sim(1:i,8),sim(1:i,9),zeros(i,1),'m','LineWidth',1.5)
% plot3(xp(1,:),ones(length(xp),1)*2.5,zeros(length(xp),1),'k','LineWidth',15);
% plot3(xp(1,:),-ones(length(xp),1)*2.5,zeros(length(xp),1),'k','LineWidth',15);
% plot3(xp(1,:),zeros(length(xp),1),zeros(length(xp),1),'w--','LineWidth',5);
view(60,30)
ylim([y-1.2 y+0.7])
xlim([x-1 x+2.5])
zlim([0 1.3])

set(gca,'DataAspectRatio',[1 1 1])

%drawnow()
F(loop)=getframe(f);
hold off
loop=loop+1;
end

end
