function out = modelSim2(X,rider,dat)
% Simulates the response of the measured input with the full closed loop model.
% Inputs 
%     K   :   The gains of the controller
%     bike: The EOM of the whipple model
%     dat :  The full measured dataset
% Outputs
%     out :  contains all 4 states of bicycle.(dphi,ddelta,phi,delta) +
%     Rider Torque


Gpd=rider.Gpd;
Predictor=rider.Predictor;
Del=rider.Del;

K=[X(1:6) 0];


  
Cd=ss([],[],[],-K);
Cd.u=Predictor.OutputName{1}(1:end-3);
Cd.y='a';

% C=ss([],[],[],[0 -X(7) 0 -X(8) 0 0 0]);
% C.u={convertStringsToChars("y"+(delay))};
% C.y='a2';

input={'w'};
% Sum=sumblk('a=a1+a2');
% C_total=connect(Cd,C,Sum,'a','y');

Gcl=connect(Gpd,Del,Predictor,Cd,input,{'y_un','y',Predictor.OutputName{1}(1:end-3)});
%Gcl=connect(Gpd,Del,Predictor,Cd,input,{'y_un'});

tic 
output=lsim(Gcl,dat.w,dat.t);
toc

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