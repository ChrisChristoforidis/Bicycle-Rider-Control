function fig =plotSimData(out,np,dat)
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
figure();
clf;
subplot(511)
plot(dat.t,dat.w);
ylabel("Lateral Force (N)");
legend("Measurement")
subplot(512)
plot(np.t,np.y(:,1)*180/pi);
ylabel("Roll Rate (deg/s)");
hold on;
plot(out.tout,out.roll_rate*180/pi);
legend("Non Parametric Model","Parametric Model")
subplot(513)
plot(np.t,np.y(:,3)*180/pi);
hold on;
plot(out.tout,out.steer_angle*180/pi);
ylabel("Steer Angle (deg)");
legend("Non Parametric Model","Parametric Model")
subplot(514)
plot(dat.t,np.y(:,2)*180/pi);
hold on;
plot(out.tout,out.roll_angle*180/pi);
xlabel("Time (s)");
ylabel("Roll Angle (deg)");
legend("Measurement","Parametric Model")
subplot(515)
plot(dat.t,np.y(:,4)*180/pi);
hold on;
plot(out.tout,out.heading*180/pi);
xlabel("Time (s)");
ylabel("Yaw Angle (deg)");
legend("Measurement","Parametric Model")

fig.hf=gcf;

end