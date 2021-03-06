%% DATA ANALYSIS

%CHRISTOS CHRISTOFORIDIS
%12/6/2018
clear;
 addpath('functions', genpath("Measurements\3_7_2019_koen"));
close all;

%% 
files = dir('Measurements\3_7_2019_koen\lateral_pull\');
filenames = [];
for ixx=3:length(files)
  filenames{ixx-2}=files(ixx,1).name;
end
header = readcell(filenames{1}, 'Range', 'A1:T1').';



% %%
% for ix=1:length(filenames)
%   data = readmatrix(filenames{ix});
%   data(data>10000)=0;
%     dt = 1 / 1000;
%     
%   raw.count = data(:, 2);
%   raw.vel = data(:, 10);
%   raw.N = length(raw.count);
%   raw.time = (0:raw.N - 1) * dt;
%   raw.w = data(:, 20);
%   raw.w(isnan(raw.w))=0;
%   figure()
%   plot(raw.w)
%   figure()
%   legend(filenames{ix})
%  pause()
% end
%%

for ix=1:length(filenames)
  data = readmatrix(filenames{ix});
  data(data>10000)=0;
  raw.Fs = 1000; %hz
  dt = 1 / raw.Fs;
  raw.count = data(:, 2);
  raw.vel = data(:, 10);
  raw.N = length(raw.count);
  raw.time = (0:raw.N - 1) * dt;
  raw.w = data(:, 20);
  raw.w(isnan(raw.w))=0;
  


   ub=60;
   raw = filterLatForce(raw,40,ub);

%
figure(1)
pause(0.00001);
frame_h = get(handle(gcf), 'JavaFrame');
set(frame_h, 'Maximized', 1);
plot(raw.time, raw.vel);
grid on
yyaxis right
plot(raw.time,raw.w*0.003,'r');hold on
plot(raw.time, data(:,6));
legend(filenames{ix})
xlabel('Time (s)');
ylabel('Bike Velocity (m/s)');
title('Choose the time points between which the analysis is going to be performed ');
[x, ~] = ginput(2);
close(1)
i1 = round(x(1)/dt);
i2 = round(x(2)/dt);

raw.w=raw.w(i1:i2);
raw.time = raw.time(i1:i2);
raw.time = raw.time - min(raw.time);
raw.SteerAngle = data(i1:i2, 5);
pureForce=data(i1:i2,20);

R_C = [  0.9938   -0.0058   -0.1112;
         0         0.9986   -0.0525;
         0.1114    0.0522    0.9924];
raw.Tmotor = data(i1:i2, 8); %Nm
  fc = 10; %hz (cutoff frequency)
  Wn = pi * fc / (2 * raw.Fs);
  [c, d] = butter(4, Wn);
%Filtering of Motor Input and SteerAngle
raw.Tmotor = filtfilt(c, d, raw.Tmotor);
raw.SteerAngle = filtfilt(c, d, raw.SteerAngle);
raw.SteerAngle = raw.SteerAngle - median(raw.SteerAngle);
%IMU Gyro data
raw.GyroX = data(i1:i2, 16);
raw.GyroY = data(i1:i2, 17);
raw.GyroZ = data(i1:i2, 18);
%Mean  forward speed
v = data(i1:i2, 10);
raw.v = mean(v);
raw.velocity=v;
%
t=(0:raw.N - 1) * dt;
figure(1)
pause(0.00001);
frame_h = get(handle(gcf), 'JavaFrame');
set(frame_h, 'Maximized', 1);
plot(t,data(:, 10));
grid on
yyaxis right
plot(t,data(:,20)*0.005,'y:');hold on
plot(t, data(:, 16));
legend(filenames{ix})
xlabel('Time (s)');
ylabel('Bike Velocity (m/s)');
title('Choose the time point you think roll angle is zero ');
[x, ~] = ginput(1);
close(1)
y1 = round(x(1)/dt);
observer_struct.GyroX=data(y1:end, 16);
observer_struct.GyroY=data(y1:end, 17);
observer_struct.GyroZ=data(y1:end, 18);
observer_struct.v=raw.vel(y1:end);
observer_struct.time=t(y1:end);

raw.RollAngle = Roll_Observer(observer_struct);
raw.RollAngle=raw.RollAngle(i1-y1:i2-y1);
dat=raw;

roll=zeros(dat.N-y1+1,1);
roll(1)=0;
for i=1:dat.N-y1
  roll(i+1)=roll(i)+observer_struct.GyroX(i)/dat.Fs;
  
end
roll=roll(i1-y1:i2-y1);


% FB status
if abs(median(dat.Tmotor))<=1e-4
  mod.FB=false;
else
  mod.FB=true;
end
%


y = dat.SteerAngle.';
[Trider, v, dv, ddv, ~] = TorqueEstimation(y, dat.Tmotor.',mod.FB);


dat.Omega = filtfilt(c,d,dat.GyroX);
RollAccel = (dat.GyroX(2:end) - dat.GyroX(1:end-1))*dat.Fs;
RollAccel(end+1)=RollAccel(end);

mod.accel = [RollAccel,ddv.'];
mod.Tdelta = Trider(1:end).';
mod.y = [dat.RollAngle(1:end), dat.SteerAngle(1:end)];
mod.t = dat.time(1:end);
mod.N = length(mod.Tdelta);
mod.Fs = dat.Fs;
mod.w = dat.w(1:end);
mod.Rates = [dat.Omega, dv(1:end).'];
mod.v = dat.v;
mod.rawForce=pureForce;
v = v.';
dat=mod;
dat.roll=highpass(roll,0.05,dat.Fs);
clearvars -except raw dat ix filenames alldata
alldata(ix)=dat;
end
%%

 T = struct2table(alldata); % convert the struct array to a table
 sortedT = sortrows(T, 'FB'); % sort the table by 'DOB'
 alldata = table2struct(sortedT) ;

 j=1;
 k=1;

 for i=1:length(alldata)
   if alldata(i).FB==false
     nofb_runs(j)=alldata(i);
     j=j+1;
   else
     fb_runs(k)=alldata(i);
     k=k+1;
   end
 end

 T = struct2table(nofb_runs); % convert the struct array to a table
 sortedT = sortrows(T, 'v'); % sort the table by 'DOB'
 nofb_runs = table2struct(sortedT) ;
 
 T = struct2table(fb_runs); % convert the struct array to a table
 sortedT = sortrows(T, 'v'); % sort the table by 'DOB'
 fb_runs = table2struct(sortedT) ;
 
clearvars -except    nofb_runs   fb_runs

% for i=1:length(nofb_runs)
%   nofb_runs(i).w=nofb_runs(i).w*1.2;
% end
% for i=1:length(fb_runs)
%   fb_runs(i).w=fb_runs(i).w*1.2;
% end


%save('C:\dev\bicycle-rider-control-identification\Measurements\all_data_thomas.mat');

fb_results.data=fb_runs;

nofb_results.data=nofb_runs;
%%

for i=1:length(fb_results.data)
dat=fb_results.data(i);
dat.y=[dat.Rates(:,1),dat.y(:,2)];
np = nonparaID(dat);
fb_results.black_box(i)=np;
end

for i=1:length(nofb_results.data)
dat=nofb_results.data(i); 
np = nonparaID(dat);
nofb_results.black_box(i)=np;
end
%%
results.fb=fb_results;
results.nofb=nofb_results;
results.Name='Name';

clearvars -except results

%%

i=1;
dat=fb_results.data(i); 
np=fb_results.black_box(i); 

figure('units','normalized','outerposition',[0 0 1 1])
plot(dat.t,dat.Rates(:,1)*180/pi);
hold on
plot(dat.t,np.y(:,1)*180/pi)
dat=fb_results.data(i); 
plot(dat.t,zeros(length(dat.w),1))
ylim([-0.3*180/pi 0.3*180/pi])
yyaxis right


plot(dat.t,dat.w)
ylim([-max(dat.w)-1 max(dat.w)+1]);


%%
fb_results = results.fb;
nofb_results =results.nofb;

for i=1:length(fb_results.data)
 delta= fb_results.data(i).y(:,2);
 Fs=fb_results.data(i).Fs;
 dt = 1 / Fs;
 U=fft(delta);
 Suu=1/length(delta)*U.*conj(U);
 f = (0:length(delta) / 2).' / length(delta) / dt;
 fb_results.data(i).Suu=real(Suu(1:length(delta)/2+1));
 fb_results.data(i).f =f;
 [fb_results.data(i).PSD, ~] = periodogram(delta, hann(length(delta)), length(delta),Fs);
end


for i=1:length(nofb_results.data)
 delta= nofb_results.data(i).y(:,2);
 Fs=nofb_results.data(i).Fs;
 dt = 1 / Fs;
 U=fft(delta);
 Suu=1/length(delta)*U.*conj(U);
 f = (0:length(delta) / 2).' / length(delta) / dt;
 nofb_results.data(i).Suu=real(Suu(1:length(delta)/2+1));
 nofb_results.data(i).f =f;
 [nofb_results.data(i).PSD, ~] = periodogram(delta, hann(length(delta)),length(delta),Fs);
end

results.fb=fb_results;
results.nofb=nofb_results;
results.Name='Name';

clearvars -except results



%%
for i=19:length(r)
  
fb_results=r(i).results.fb;
nofb_results=r(i).results.nofb;
Name=r(i).results.Name;




plotPSD(fb_results.data,nofb_results.data,Name)

% 
% plotSpeedIRF(fb_results.black_box,fb_results.data,Name)
% plotSpeedIRF(nofb_results.black_box,nofb_results.data,Name)

plotFBstatusIRF(fb_results.black_box,nofb_results.black_box,fb_results.data,nofb_results.data,Name)


end