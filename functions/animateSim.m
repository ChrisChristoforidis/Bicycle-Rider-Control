bike=delftbikeHeading(dat.v,whipple);
omegac= 2 * pi * 2.17;
Gp=plantModel(bike,omegac);

y0=zeros(9,1); %initial condition


%% RK 4  SIMULATION

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
  [te,ye,we]=rk4step(te,ye,dt,dat.v,Gp,K,dat.w);
  w(i,:)=we;
  y(i,:)=ye;
  t(i)=te;
end

%% ODE 45 SIM
tspan=[0 dat.t(end)];
[t,y]=ode45(@(t,z) funhan(t,z,dat.v,Gp,K,dat.w),tspan,y0);

figure
plot(t,y(:,4))
hold on
ylim([-0.4 0.4])
%%
theta=linspace(0,4*pi,length(y));
k=zeros(length(y),9);
k(:,4)=theta.';
animate(y,length(y),w)
%%
function animate(sim,nstep,w)

% flat plane
[xp ,yp] = meshgrid(-10:50:200); % Generate x and y data
zp = zeros(size(xp, 1)); % Generate z data
%bike
wb=1.03;%m
lamda=pi/10;%rad
CD=0.75;
e3=[0;0;1];
e2=[0;1;0];
e1=[1;0;0];
radr=0.6858/2; radf=0.6858/2;  %m
theta=linspace(0,2*pi,50);%rad
nw=(w-min(w))/(max(w)-min(w));
%surf(xp, yp, zp) % Plot the surface
f=figure();

for i = 1:2: nstep

r1=rotmat(e3,sim(i,5));r2=rotmat(r1*e1,sim(i,3));rrf=r2*r1;
r3=rotmat(e2,-lamda);
r4=rotmat(r3*e3,sim(i,4));
q2=r4*r3;
q=rrf;
x=sim(i,8);
y=sim(i,9);
z=0;
xr=x+radr*cos(theta);zr=z+radr*sin(theta)+radr; yr=y+0*theta;
xf=x+radf*cos(theta)+wb;zf=z+radf*sin(theta)+radf;yf=y+0*theta;
c1=q*[xr ;yr; zr]; c2m=q2*[xf-x-wb;yf-y;zf-z-radf];c2=q*[c2m(1,:)+x+wb;c2m(2,:)+y;c2m(3,:)+z+radf];

%Segment 1 (fork)
p1s=[x+wb;y;z+radf];

p1e=[x-sin(lamda)*CD+wb;y;z+cos(lamda)*CD+radf];

s1=q*[p1s p1e];
%Segment 2 (frame)

p2s=[x;y;z+radr];

p2e=[x-sin(lamda)*3/4*CD+wb;y;z+cos(lamda)*3/4*CD+radr];

s2=q*[p2s p2e];
%Segment 3 (saddle frame)

p3s=[x-sin(lamda)*CD/4+wb/3;y;z+cos(lamda)*CD/4+radr];

p3e=p3s;
p3e(3)=p3s(3)+0.3;

s3=q*[p3s p3e];
%Segment 4 (handlebar)

p4s=[p1e(1)-0.1;p1e(2)-0.3;p1e(3)];

p4l=[p1e(1);p1e(2)-0.3;p1e(3)];

p4r=[p1e(1); p1e(2)+0.3 ;p1e(3)];

p4e=[p1e(1)-0.1; p1e(2)+0.3 ;p1e(3)];

 s4=[p4s p4l p4r p4e];
s4(3,:)=s4(3,:)-CD*cos(lamda)-radr-z;
s4(2,:)=s4(2,:)-y;
s4(1,:)=s4(1,:)+CD*sin(lamda)-wb-x;
s4=q2*s4;
s4(3,:)=s4(3,:)+CD*cos(lamda)+radr+z;
s4(2,:)=s4(2,:)+y;
s4(1,:)=s4(1,:)-CD*sin(lamda)+wb+x;
s4=q*s4;
%Segment 5 (saddle)

p5s=[p3e(1)-0.03;p3e(2)-0.07;p3e(3)];

p5m=[p3e(1)+0.15; p3e(2); p3e(3)];

p5e=[p3e(1)-0.03; p3e(2)+0.07; p3e(3)];

s5=q*[p5s p5m p5e];
%Segment 6 (force vector)
p6s=p3e;
% if (nw(i)>0.25)
%  a=1;
% end
p6m=p6s+[0;-1*nw(i)-0.01;0];
p6l=p6m+[0.1*nw(i);0.08*nw(i);0];
p6r=p6m+[-0.1*nw(i);0.08*nw(i);0];

s6=q*[p6s p6m p6l p6r p6m];
plot3(s1(1,:),s1(2,:),s1(3,:),'b','LineWidth',3)
hold on
patch(c1(1,:),c1(2,:),c1(3,:),'cyan','LineWidth',2)
hold on
patch(c2(1,:),c2(2,:),c2(3,:),'cyan','LineWidth',2)
plot3(s2(1,:),s2(2,:),s2(3,:),'b','LineWidth',3)
plot3(s3(1,:),s3(2,:),s3(3,:),'b','LineWidth',3)
plot3(s4(1,:),s4(2,:),s4(3,:),'b','LineWidth',3)
patch(s5(1,:),s5(2,:),s5(3,:),'cyan','LineWidth',3,'EdgeColor','blue')
patch(s6(1,:),s6(2,:),s6(3,:),'r','LineWidth',2.5,'EdgeColor','red')
%surf(xp, yp, zp) % Plot the surface
%plot3(sim(1:i,8),sim(1:i,9),zeros(i,1),'g')
% xlim([x-4 x+8])
% ylim([y-3 y+3])
% zlim([0 1.5])
axis equal
%view(90,72)
% set(gca,'DataAspectRatio',[1 1 1])
drawnow()

clf(f)
end


end
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