%% DATA ANALYSIS

%CHRISTOS CHRISTOFORIDIS
%12/6/2018
clear;
addpath('functions', genpath("Measurements/iris"));
close all;

%% 
files = dir('Measurements/iris/steer');
filenames = [];
for ixx=3:length(files)
  filenames{ixx-2}=files(ixx,1).name;
end
header = readcell(filenames{1}, 'Range', 'A1:T1').';

%%

for ix=1:length(filenames)
  data = readmatrix(filenames{ix});
  raw.ID=str2double(filenames{ix}(1:end-4));
  raw.Fs = 1000; %hz
  dt = 1 / raw.Fs;
  raw.count = data(:, 2);
  raw.vel = data(:, 10);
  raw.N = length(raw.count);
  raw.time = (0:raw.N - 1) * dt;
  raw.w=data(:,8);
  raw.w(abs(raw.w)<10.5)=0;
  raw.w=smooth(raw.w,0.0005);
%%
figure(1)
pause(0.00001);
frame_h = get(handle(gcf), 'JavaFrame');
set(frame_h, 'Maximized', 1);
plot(raw.time, raw.vel);
grid on
yyaxis right
plot(raw.time,raw.w*0.01,'r');hold on
plot(raw.time, data(:,5));

xlabel('Time (s)');
ylabel('Bike Velocity (m/s)');
legend(filenames{ix})
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
%%

dat = Roll_Observer(raw);

%% FB status
if abs(median(dat.Tmotor))<=1e-4
  mod.FB=false;
else
  mod.FB=true;
end
%%


y = dat.SteerAngle.';
[Trider, v, dv, ddv, ~] = TorqueEstimation(y, dat.Tmotor.',mod.FB);


dat.Omega(:, 1) = filtfilt(c,d,dat.Omega(:, 1));
RollAccel = (dat.Omega(2:end, 1) - dat.Omega(1:end-1, 1))*dat.Fs;
RollAccel(end+1)=RollAccel(end);

mod.accel = [RollAccel,ddv.'];
mod.Tdelta = Trider(1:end).';
mod.y = [dat.RollAngle(1:end), dat.SteerAngle(1:end)];
mod.t = dat.time(1:end);
mod.N = length(mod.Tdelta);
mod.Fs = dat.Fs;
mod.w = dat.w(1:end);
mod.Rates = [dat.Omega(1:end, 1), dv(1:end).'];
mod.v = dat.v;
mod.rawForce=pureForce;
mod.ID=raw.ID;
v = v.';
dat=mod;
clearvars -except raw dat ix filenames alldata
%%
roll=zeros(dat.N,1);
roll(1)=0;
for i=1:dat.N-1
  roll(i+1)=roll(i)+dat.Rates(i,1)/dat.Fs;
  
end

dat.roll=highpass(roll,0.05,dat.Fs);

% dat.y(:,1)=dat.roll;

alldata(ix)=dat;
end
%%

 T = struct2table(alldata); % convert the struct array to a table
 sortedT = sortrows(T, 'v'); % sort the table by 'DOB'
 alldata = table2struct(sortedT) ;
for i=1:length(alldata)
  alldata(i).w=-alldata(i).w;
end
fb_results.data=alldata;
clearvars -except fb_results
%save('C:\dev\bicycle-rider-control-identification\Measurements\all_data_steer_dani.mat');


