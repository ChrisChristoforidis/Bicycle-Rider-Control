function rider=getRiderModel(bike,omegac,delay,dat)

Gp=plantModel(bike,omegac);

Gp2=ss(Gp.A,Gp.B(:,2:3),eye(7),zeros(7,2));
Gp2.u={'a','w'};
Gp2.y='y_un';
Ihb = 0.0960; %Inertia of handlebars (calculated from handlebari_inertia_motor_damping.m) 
Gp3=ss(Gp.A,Gp.B(:,2:3),[Gp2.C;Ihb*Gp2.A(2,:)],[Gp2.D;Ihb*Gp2.B(2,:)]);
Gp3.u={'a','w'};
Gp3.y='y_un';


% if max(abs(dat.w)) < 20
%   Gp.B(:,3)=[bike.B(:,2) ;0 ;0];
% end


Del=ss([],[],[],eye(8),1/dat.Fs,'OutputDelay',delay);
Del.OutputDelay(5)=delay*4;
Del.OutputDelay(8)=0;
Del.InputName='y_un';
Del.OutputName='y';
Gpd=c2d(Gp2,1/dat.Fs);
Gpd2=c2d(Gp3,1/dat.Fs);
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

Del2=ss([],[],[],eye(7),1/dat.Fs,'OutputDelay',delay);
Del2.u = Predictor.OutputName{1}(1:end-3);
Del2.y = 'ym_d';


rider.Gpd2=Gpd2;
rider.Gpd=Gpd;
rider.Predictor=Predictor;
rider.Del=Del;
rider.Del2=Del2;
rider.Gp=Gp;
rider.bike=bike;
end