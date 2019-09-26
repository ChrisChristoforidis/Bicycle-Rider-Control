function fig =plotSimDataSteer(out,np,dat)
% Fucntion to plot the simulated data and compare them to the 
% non-parametric model ouput and the measured signals
% 
% Inputs 
% 
%     out : simulation output.Contains all the states of the bicycle.
%     np  : non-parametric model.Contains the steer angle and roll angle outputs.
%     dat : all measured signals.
%out.tout=dat.t;

out.tout=dat.t;
figure('units','normalized','outerposition',[0 0 1 1])
clf;
subplot(411)
plot(dat.t,dat.w);
ylabel("Lateral Force (N)");
legend("Measurement")
subplot(412)
plot(dat.t,resample(dat.Tdelta,1,5));
ylabel("Torque");
hold on;
plot(out.tout,out.steer_torque);
legend("Raw Data","Parametric Model")
subplot(413)
plot(np.t,np.y(:,2)*180/pi);
hold on;
plot(out.tout,out.steer_angle*180/pi);
ylabel("Steer Angle (deg)");
legend("Non Parametric Model","Parametric Model")
subplot(414)
plot(dat.t,np.y(:,1)*180/pi);
hold on;
plot(out.tout,out.roll_angle*180/pi);
xlabel("Time (s)");
ylabel("Roll Angle (deg)");
legend("Non Parametric Model","Parametric Model")


fig.hf=gcf;

end