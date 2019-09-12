n=1;
load('gains_6param_steer.mat')
bike=delftbikeHeading(2.6);
omegac= 2 * pi * 2.17;
Gp=plantModel(bike,omegac);

y0=zeros(9,1); %initial condition


%% RK 4  SIMULATION
tic
dt = 0.01; 
nstep = floor((mean_rider.fb.data(1).t(end)-0)/dt);
ye=y0;
te=0;
y=zeros(nstep,9);
t=zeros(nstep,1);
w=zeros(nstep,1);

y(1,:)=ye;

for i=2:nstep
  we=mean_rider.fb.data(1).w(floor(i*dt*1000));
  [te,ye]=rk4step(te,ye,dt,2.6,Gp,Gains(n,:),we);
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
F=animate(y,length(y)/3.5,0.01,w,2.6);

%%
v = VideoWriter('mean_8.avi');
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
