function rider=getRiderModel(bike,omegac,delay,dat)

[Gp,Gnm]=plantModel(bike,omegac);
bike.u(1)=cellstr('Tlean'); %rider lean torque
bike.u(2)=cellstr('Tdelta'); 
bike.u(3)=cellstr('w'); %lateral force

Gnm.y={'Tc'};

Gp2=ss(Gp.A,Gp.B(:,2:3),eye(7),zeros(7,2));
Gp2.u={'a','w'};
Gp2.y='y_un';
Ihb = 0.0960; %Inertia of handlebars (calculated from handlebari_inertia_motor_damping.m) 
Gp3=ss(Gp.A,Gp.B(:,2:3),[Gp2.C;Ihb*Gp2.A(2,:)],[Gp2.D;Ihb*Gp2.B(2,:)]);
Gp3.u={'a','w'};
Gp3.y='y_un';


GPC=eye(7);
GPC=[GPC;0 0 0 0 0 1 0];
GPD= zeros(8,3);
GPD(6,:)=[0 0 1];
Btint=[bike.B(:,2);0;0];
Gp4=ss(Gp.A,[Gp.B(:,2:3) Btint],GPC,GPD);
Gp4.u={'a','w','Tint'};
Gp4.y='y_un';


B=ss(bike.A,bike.B(:,2:3),[bike.C ;bike.A(1,:) ;bike.A(2,:)],[bike.D(:,2:3);bike.B(1,2:3);bike.B(2,2:3)]);
B.u={'Td','w'};
B.y={'y'};
whipple=getbicycleEOM();
Km=[whipple.C1(2,1)*dat.v  whipple.C1(2,2)*dat.v whipple.K0(2,1)+whipple.K2(2,1)*dat.v^2 whipple.K0(2,2)+whipple.K2(2,2)*dat.v^2 0 whipple.M0(2,1) whipple.M0(2,2)-Ihb];
%Km=zeros(1,7);

MotorDynamics =ss([],[],[],Km);
MotorDynamics.u = 'y';
MotorDynamics.y = 'Tm';
DistDynamics =ss([],[],[],whipple.Hfw(2));
DistDynamics.u='w';
DistDynamics.y='SW';
Sum=sumblk('Td=Tint+Tr+Tm-SW');
Gnm2.A=[0 1; -omegac^2 -2*sqrt(1/2)*omegac];
Gnm2.B=[0;omegac^2];
Gnm2.C=[1 0];
Gnm2.D=0;
Gnm2=ss(Gnm2.A,Gnm2.B,Gnm2.C,Gnm2.D);
Gnm2.u='a'; %neural input
Gnm2.y='Tr';%rider steer torque
plant=connect(B,Gnm2,MotorDynamics,Sum,DistDynamics,{'a','w','Tint'},{'y'});

Gp_fb_off=ss(plant.A,plant.B,[eye(7); plant.C(7,:)], [GPD(1:7,:); plant.D(7,:)]);
Gp_fb_off.u={'a','w','Tint'};
Gp_fb_off.y={'y_un'};
% Gp_fb_off.y(9)={'Td'};

rider.Gpd4=c2d(Gp_fb_off,1/dat.Fs);

% if max(abs(dat.w)) < 20
%   Gp.B(:,3)=[bike.B(:,2) ;0 ;0];
% end


Del=ss([],[],[],eye(8),1/dat.Fs,'OutputDelay',delay);
% Del.OutputDelay(1)=delay;
% Del.OutputDelay(2)=delay/2;
% Del.OutputDelay(3)=delay*4;
% Del.OutputDelay(4)=delay/2;
% Del.OutputDelay(6)=delay/2;
% Del.OutputDelay(5)=delay*4;
Del.OutputDelay(1)=delay;
Del.OutputDelay(2)=delay;
Del.OutputDelay(3)=delay;
Del.OutputDelay(4)=delay;
Del.OutputDelay(6)=delay;
Del.OutputDelay(5)=delay;
Del.OutputDelay(8)=0;
Del.InputName='y_un';
Del.OutputName='y';
Gpd=c2d(Gp2,1/dat.Fs); %NM+bike normal
Gpd2=c2d(Gp3,1/dat.Fs);%hacky acceleration feedback
Gpd3=c2d(Gp4,1/dat.Fs);%intrinsic stiffeness and damping

%Predictor
if delay==0
  Predictor=ss([],[],[],eye(7),1/dat.Fs);
  Predictor.u='y';
  Predictor.y={convertStringsToChars("y"+(delay))};
else

  Bp=ss([],[],[],Gpd.B(:,1),1/dat.Fs,'OutputDelay',delay);
  Bp.u='a';
  Bp.y='ad';

  Ap=ss([],[],[],Gpd.A,1/dat.Fs);
  Ap.u='y';
  Ap.y='yD';
  Sum2=sumblk('y1=yD+ad',7);
  Predictor=connect(Ap,Bp,Sum2,{'a','y'},'y1');
end
  
for n=1:delay-1  
  Bp=ss([],[],[],Gpd.B(:,1),1/dat.Fs,'OutputDelay',delay-n);
  Bp.u='a';
  Bp.y='ad';

  Ap=ss([],[],[],Gpd.A,1/dat.Fs);
  Ap.u=convertStringsToChars("y"+(n));
  Ap.y='yD';
  Sum2=sumblk(convertStringsToChars("y"+(n+1)+"=yD+ad"),7);
  Predictor2=connect(Ap,Bp,Sum2,{'a',convertStringsToChars("y"+n)},convertStringsToChars("y"+(n+1)));
  Predictor=connect(Predictor,Predictor2,{'a','y'},{convertStringsToChars("y"+(n+1))});
  
end
rider.Predictor=Predictor;

%Predictor2
if delay==0
  Predictor=ss([],[],[],eye(7),1/dat.Fs);
  Predictor.u='y';
  Predictor.y={convertStringsToChars("y"+(delay))};
else

  Bp=ss([],[],[],Gpd3.B(:,1:2:3),1/dat.Fs,'OutputDelay',delay);
  Bp.u={'a','Tint'};
  Bp.y='ad';

  Ap=ss([],[],[],Gpd3.A,1/dat.Fs);
  Ap.u='y';
  Ap.y='yD';
  Sum2=sumblk('x1=yD+ad',7);
  Cp=ss([],[],[],[eye(7) [0;0;0;0;0;1;0]],1/dat.Fs);
  Cp.InputDelay(end)=delay;
  Cp.u={'x1'};
  Cp.u(end)={'Tint'};
  Cp.y='y1';
  Predictor=connect(Ap,Bp,Cp,Sum2,{'a','Tint','y'},'y1');
end
  
for n=1:delay-1  
  Bp=ss([],[],[],Gpd3.B(:,1:2:3),1/dat.Fs,'OutputDelay',delay-n);
  Bp.u={'a','Tint'};
  Bp.y='ad';

  Ap=ss([],[],[],Gpd3.A,1/dat.Fs);
  Ap.u=convertStringsToChars("y"+(n));
  Ap.y='yD';
  Sum2=sumblk(convertStringsToChars("x"+(n+1)+"=yD+ad"),7);
  Cp=ss([],[],[],[eye(7) [0;0;0;0;0;1;0]],1/dat.Fs);
  Cp.InputDelay(end)=delay-n;
  Cp.u=convertStringsToChars("x"+(n+1));
  Cp.u(end)={'Tint'};
  Cp.y=convertStringsToChars("y"+(n+1));
  Predictor2=connect(Ap,Bp,Cp,Sum2,{'a','Tint',convertStringsToChars("y"+n)},convertStringsToChars("y"+(n+1)));
  Predictor=connect(Predictor,Predictor2,{'a','Tint','y'},{convertStringsToChars("y"+(n+1))});
  
end

rider.Predictor2=Predictor;

Del2=ss([],[],[],eye(7),1/dat.Fs,'OutputDelay',delay);
Del2.u = Predictor.OutputName{1}(1:end-3);
Del2.y = 'ym_d';
rider.Gp2 = Gp_fb_off;
rider.Gpd3=Gpd3;
rider.Gpd2=Gpd2;
rider.Gpd=Gpd;
rider.Del=Del;
rider.Del2=Del2;
rider.Gp=Gp;
rider.bike=bike;
rider.Gnm=Gnm;
end