function out = modelSim3(X,rider,dat)
% Simulates the response of the measured input with the full closed loop model.
% Inputs 
%     K   :   The gains of the controller
%     bike: The EOM of the whipple model
%     dat :  The full measured dataset
% Outputs
%     out :  contains all 4 states of bicycle.(dphi,ddelta,phi,delta) +
%     Rider Torque


Gpd=rider.Gpd2;
Predictor=rider.Predictor;
Del=rider.Del;
K=[X(1:5) X(6) 0 0];
%Q=diag(X(1:7));
% 0
%  K=lqr(rider.Gp.A,rider.Gp.B(:,2),Q,X(8));
%  K=[K 0];
%   
%K=[K 0 0];
Cd=ss([],[],[],-K);
Cd.u={Predictor.OutputName{1}(1:end-3)};
Cd.InputName(8)={'y(8)'};
Cd.y='a';

% C=ss([],[],[],[0 -X(7) 0 -X(8) 0 0 0]);
% C.u={convertStringsToChars("y"+(delay))};
% C.y='a2';

input={'w'};
% Sum=sumblk('a=a1+a2');
% C_total=connect(Cd,C,Sum,'a','y');
Del2=rider.Del2;

Sum1=sumblk('ye=y-ym_d',7);
% F=ss([],[],[],eye(7)*0.8,1/dat.Fs);
% F.u='ye'
% F.y='ye'
Sum2=sumblk(convertStringsToChars("y_f="+Predictor.OutputName{1}(1:end-3)+"+ye"),7);
Cd.u={'y_f(1)','y_f(2)','y_f(3)','y_f(4)',convertStringsToChars(convertCharsToStrings(Predictor.OutputName{1}(1:end-3))+"(5)"),'y(6)','y(7)','y(8)'};
% F = 1/(20*tf('s')+1);
% F.InputName = {'yf(1)','yf(2)','yf(3)','yf(4)','yf(5)','yf(6)','yf(7)'};
% F.OutputName = {'yee(1)','yee(2)','yee(3)','yee(4)','yee(5)','yee(6)','yee(7)'};
% F=c2d(F,1/dat.Fs);

Gcl=connect(Gpd,Del,Sum2,Del2,Sum1,Predictor,Cd,input,{'y_un','y',Predictor.OutputName{1}(1:end-3),'ye','y_f'});
%Gcl=connect(Gpd,Del,Predictor,Cd,input,{'y_un'});


output=lsim(Gcl,dat.w,dat.t);


% Output assignment
if ( size(Gpd.A,1)==7)
  out.heading=output(:,5);
  out.steer_torque = output(:,6);
else
out.steer_torque = output(:,5);
end
out.steer_angle = output(:,4);
out.roll_angle = output(:,3);
out.roll_rate = output(:,1);
out.steer_rate = output(:,2);
out.Input= -K*output(:,1:length(K)).';

end