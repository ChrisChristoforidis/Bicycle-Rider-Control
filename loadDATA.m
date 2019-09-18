%% DATA ANALYSIS

%CHRISTOS CHRISTOFORIDIS
%12/6/2018
% clear;
% close all;

%% 
files = dir('subjects');

for i=3:length(files)
  subject_folders{i-2}=files(i,1).name;
end

for k=5
  
  addpath("subjects/"+subject_folders{k}+"/lateral_pull");
  files=dir("subjects/"+subject_folders{k}+"/lateral_pull");
  filenames=[];
for ixx=3:length(files)
  filenames{ixx-2}=files(ixx,1).name;
end
header = readcell(filenames{1}, 'Range', 'A1:T1').';



results(k).Name=subject_folders{k};
%%
results(k).fb.data=[];
results(k).nofb.data=[];
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
  plot(raw.time, raw.vel/10);
  hold on
  plot(raw.time, data(:,6));
  grid on
  yyaxis right
  plot(raw.time,raw.w,'r');hold on
  ylim([0 350])
  legend(filenames{ix})
  xlabel('Time (s)');
  ylabel('Bike Velocity (m/s)');
  title('Choose the time points between which the analysis is going to be performed ');
  [x, ~] = ginput(2);
  close(1)
  i1 = round(x(1)/dt);
  i2 = round(x(2)/dt);
  bounds=[i1, i2];
  raw.w=raw.w(i1:i2);
  raw.N=length(raw.w);
  raw.time = raw.time(i1:i2);
  raw.time = raw.time - min(raw.time);
  steer_angle = data(i1:i2, 5);

  R_e =[0.993852724155588,-0.006106115160415,-0.110541747978598;
  0.006068692321752,0.999981357505053,-6.749932267486786e-04;
  0.110543808790994,0,0.993871252395390];
  Tmotor = data(i1:i2, 8); %Nm
  fc = 10; %hz (cutoff frequency)
  Wn = pi * fc / (2 * raw.Fs);
  [c, d] = butter(4, Wn);
  %Filtering of Motor Input and SteerAngle
  Tmotor = filtfilt(c, d, Tmotor);
  steer_angle = filtfilt(c, d, steer_angle);
  steer_angle = steer_angle - median(steer_angle);
  %IMU Gyro data
  gyro=data(i1:i2,16:18)-median(data(i1:i2,16:18));
  gyro_corrected=(R_e*gyro')';
  gyro_filtered=filtfilt(c, d, gyro_corrected);
  %Lin Accel
  linaccel=-data(i1:i2,13:15);
  linaccel_corrected=(R_e*linaccel')';
  linaccel_filtered=filtfilt(c, d, linaccel_corrected);

  %Mean  forward speed
  v = data(i1:i2, 10);
  raw.v = mean(v);
  raw.velocity=v;


  %Eueler Integration of Roll and Yaw


  roll=zeros(raw.N,1);
  yaw_rate=zeros(raw.N,1);
  yaw=zeros(raw.N,1);
  roll(1)=0;
  yaw(1)=0;
    for i=1:raw.N-1
      roll(i+1)=roll(i)+gyro_filtered(i,1)/raw.Fs;
      yaw_rate(i)=gyro_filtered(i,2).*sin(roll(i))+gyro_filtered(i,3).*cos(roll(i));
      yaw(i+1)=yaw(i)+yaw_rate(i)/raw.Fs;
    end

  roll=highpass(roll,0.05,1000);
  roll=roll-median(roll);

  yaw=highpass(yaw,0.05,1000);
  yaw=yaw-median(yaw);
  % FB status
    if abs(median(Tmotor))<=1e-4
      dat.FB=false;
    else
      dat.FB=true;
    end
  % Steer PSD
  
 f = (0:length(steer_angle) / 2).' / length(steer_angle) / dt;
 [PSD, ~] = periodogram(steer_angle, hann(length(steer_angle)), length(steer_angle),raw.Fs);


  y = steer_angle.';
  [Trider, v, dv, ddv, ~] = TorqueEstimation(y,  Tmotor.',dat.FB);


  dat.Omega = gyro_filtered;
  RollAccel = (gyro_filtered(2:end,1) - gyro_filtered(1:end-1,1))*raw.Fs;
  RollAccel(end+1)=RollAccel(end);

  dat.accel = [RollAccel,ddv.'];
  dat.Tdelta = Trider(1:end).';
  dat.LinAccel=linaccel_filtered;
  dat.y = [roll,steer_angle,yaw];
  dat.t = raw.time(1:end);
  dat.N = length(dat.Tdelta);
  dat.Fs = raw.Fs;
  dat.w = raw.w;
  dat.N=raw.N;
  dat.fork_motor=data(i1:i2,7);
  dat.f=f;
  dat.PSD=PSD;
  dat.Rates = [gyro_filtered(:,1),yaw_rate, dv.'];
  dat.v = raw.v;
  dat.ID=filenames{ix}(1:end-4);
  dat.bounds=bounds;
  if ( dat.FB ==true)
    results(k).fb.data=[ results(k).fb.data  dat];
  else
    results(k).nofb.data=[ results(k).nofb.data  dat];
  end
  
end

 T = struct2table(results(k).fb.data); % convert the struct array to a table
 sortedT = sortrows(T, 'v'); % sort the table by 'DOB'
 results(k).fb.data = table2struct(sortedT) ;
 T = struct2table(results(k).nofb.data); % convert the struct array to a table
 sortedT = sortrows(T, 'v'); % sort the table by 'DOB'
 results(k).nofb.data = table2struct(sortedT) ;
 rmpath("subjects/"+subject_folders{k}+"/lateral_pull");
end





%% FIR black box filtering
% 
for k=4
  for i=1:length(results(k).fb.data)
    dat=results(k).fb.data(i);
    dat.y=[dat.Rates(:,1),dat.y(:,1),dat.y(:,2),dat.y(:,3)];
    results(k).fb.black_box(i) = nonparaID(dat);  
  end
  for i=1:length(results(k).nofb.data)
    dat=results(k).nofb.data(i); 
    dat.y=[dat.Rates(:,1),dat.y(:,1),dat.y(:,2),dat.y(:,3)];
    results(k).nofb.black_box(i)= nonparaID(dat);  
  end

end
% 

%%
load('steer\all_data_steer_camilo.mat')
resultss(1)=results;
load('steer\all_data_steer_iris.mat');
resultss(2)=results;
load('steer\all_data_steer_jork.mat');
resultss(3)=results;
load('steer\all_data_steer_christodoulos.mat');
resultss(4)=results;
load('steer\all_data_steer_katerina.mat');
resultss(5)=results;
load('steer\all_data_steer_koen.mat');
resultss(6)=results;
load('steer\all_data_steer_lars.mat');
resultss(7)=results;
load('steer\all_data_steer_leo.mat');
resultss(8)=results;
load('steer\all_data_steer_marko.mat');
resultss(9)=results;
load('steer\all_data_steer_michael.mat');
resultss(10)=results;
load('steer\all_data_steer_nikos.mat');
resultss(11)=results;
load('steer\all_data_steer_pier.mat');
resultss(12)=results;
load('steer\all_data_steer_pranav.mat');
resultss(13)=results;
load('steer\all_data_steer_thomas.mat');
resultss(14)=results;

results=resultss;
clear resultss
%%
for k=1:length(results)
  while(length(results(k).fb.data)>4)
    vel=zeros(length(results(k).fb.data),1);
    for j=1:length(results(k).fb.data)
      vel(j) =results(k).fb.data(j).v;
    end
    [~,idx]=min(diff(vel));
    v1=mean(vaf([ results(k).fb.data(idx).y],results(k).fb.black_box(idx).y));
    v2=mean(vaf([ results(k).fb.data(idx+1).y],results(k).fb.black_box(idx+1).y));
    if (v1<v2)
      results(k).fb.data(idx)=[];
      results(k).fb.black_box(idx)=[];
    else
      results(k).fb.data(idx+1)=[];
      results(k).fb.black_box(idx+1)=[];
    end
  end

end


%% Keep 4 runs per condition per participant.
% runs are removed  depending on how good the average FIT was for all out put signals.


for k=1:length(results)
  while(length(results(k).fb.data)>4)
    vel=zeros(length(results(k).fb.data),1);
    for j=1:length(results(k).fb.data)
      vel(j) =results(k).fb.data(j).v;
    end
    [~,idx]=min(diff(vel));
    v1=mean(vaf([results(k).fb.data(idx).Rates(:,1) results(k).fb.data(idx).y],results(k).fb.black_box(idx).y));
    v2=mean(vaf([results(k).fb.data(idx+1).Rates(:,1) results(k).fb.data(idx+1).y],results(k).fb.black_box(idx+1).y));
    if (v1<v2)
      results(k).fb.data(idx)=[];
      results(k).fb.black_box(idx)=[];
    else
      results(k).fb.data(idx+1)=[];
      results(k).fb.black_box(idx+1)=[];
    end
  end
  
   while(length(results(k).nofb.data)>4)
    vel=zeros(length(results(k).nofb.data),1);
    for j=1:length(results(k).nofb.data)
      vel(j) =results(k).nofb.data(j).v;
    end
    [~,idx]=min(diff(vel));
    v1=mean(vaf([results(k).nofb.data(idx).Rates(:,1) results(k).nofb.data(idx).y],results(k).nofb.black_box(idx).y));
    v2=mean(vaf([results(k).nofb.data(idx+1).Rates(:,1) results(k).nofb.data(idx+1).y],results(k).nofb.black_box(idx+1).y));
    if (v1<v2)
      results(k).nofb.data(idx)=[];
      results(k).nofb.black_box(idx)=[];
    else
      results(k).nofb.data(idx+1)=[];
      results(k).nofb.black_box(idx+1)=[];
    end
  end
end

%%
model_fit_fb=zeros(4,20,4);
model_fit_nofb=zeros(4,20,4);

for k=1:length(results)
  for i=1:4
    v1=vaf([results(k).nofb.data(i).Rates(:,1) results(k).nofb.data(i).y],results(k).nofb.black_box(i).y);
    v2=vaf([results(k).fb.data(i).Rates(:,1) results(k).fb.data(i).y],results(k).fb.black_box(i).y);
  
    model_fit_fb(:,k,i)=v1;
    model_fit_nofb(:,k,i)=v2;
    
  end

end


 C = permute(model_fit_fb,[1 3 2]);
 C = reshape(C,[],size(A,2),1);
 
best_fit=[ mean(C); std(C)];
%%
roll_rate_h_fb=zeros(length(results(1).fb.black_box(1).h(:,1)),20,4);
roll_rate_h_nofb=zeros(length(results(1).fb.black_box(1).h(:,1)),20,4);
roll_angle_h_fb=zeros(length(results(1).fb.black_box(1).h(:,1)),20,4);
roll_angle_h_nofb=zeros(length(results(1).fb.black_box(1).h(:,1)),20,4);
steer_angle_h_fb=zeros(length(results(1).fb.black_box(1).h(:,1)),20,4);
steer_angle_h_nofb=zeros(length(results(1).fb.black_box(1).h(:,1)),20,4);
yaw_angle_h_fb=zeros(length(results(1).fb.black_box(1).h(:,1)),20,4);
yaw_angle_h_nofb=zeros(length(results(1).fb.black_box(1).h(:,1)),20,4);
speed_fb=zeros(20,4);
speed_nofb=zeros(20,4);

for k=1:length(results)
  for i=1:4
    roll_rate_h_fb(:,k,i)=results(k).fb.black_box(i).h(:,1);
    roll_rate_h_nofb(:,k,i)=results(k).nofb.black_box(i).h(:,1);
    roll_angle_h_fb(:,k,i)=results(k).fb.black_box(i).h(:,2);
    roll_angle_h_nofb(:,k,i)=results(k).nofb.black_box(i).h(:,2);
    steer_angle_h_fb(:,k,i)=results(k).fb.black_box(i).h(:,3);
    steer_angle_h_nofb(:,k,i)=results(k).nofb.black_box(i).h(:,3);
    yaw_angle_h_fb(:,k,i)=results(k).fb.black_box(i).h(:,4);
    yaw_angle_h_nofb(:,k,i)=results(k).nofb.black_box(i).h(:,4);    
    speed_fb(k,i)=results(k).fb.data(i).v;
    speed_nofb(k,i)=results(k).nofb.data(i).v;   
  end
    
%     figure(2)
%     subplot(141)
%   %  plot(results(k).fb.black_box(1).h(:,2),'k')
%     hold on
%    plot(results(k).nofb.black_box(1).h(:,2),'r')
%     subplot(142)
%  %   plot(results(k).fb.black_box(2).h(:,2),'k')
%     hold on
%     plot(results(k).nofb.black_box(2).h(:,2),'r')
%     subplot(143)
%  %   plot(results(k).fb.black_box(3).h(:,2),'k')
%     hold on
%     plot(results(k).nofb.black_box(3).h(:,2),'r')
%     subplot(144)
%  %   plot(results(k).fb.black_box(4).h(:,2),'k')
%     hold on
%     plot(results(k).nofb.black_box(4).h(:,2),'r')
  
end


%%

a=results(18).fb.data(4).w ;
b=length(a);
c=0:0.001:0.001*(length(a)-1);
% [t,w]=generateImpulse(200,0.2);
% w=repmat(w,8);
% a=w(:,1);
% b=length(w);
% c=0:0.001:0.001*(length(a)-1);
for i=1:4

mean_rider.fb.data(i).w = a;
mean_rider.fb.data(i).N = b;
mean_rider.fb.data(i).t =c;
mean_rider.fb.data(i).v =mean(speed_fb(:,i));

mean_rider.fb.black_box(i).N = b;
mean_rider.fb.black_box(i).t =c;

mean_rider.fb.black_box(i).h=[ mean(roll_rate_h_fb(:,:,i),2) mean(roll_angle_h_fb(:,:,i),2) ...
                            mean(steer_angle_h_fb(:,:,i),2) mean(yaw_angle_h_fb(:,:,i),2)];
mean_rider.fb.black_box(i).y = convSim(mean_rider.fb.data(i).w,mean_rider.fb.black_box(i));


mean_rider.nofb.data(i).w = a;
mean_rider.nofb.data(i).N = b;
mean_rider.nofb.data(i).t = c;
mean_rider.nofb.data(i).v =mean(speed_nofb(:,i));

mean_rider.nofb.black_box(i).N = b;
mean_rider.nofb.black_box(i).t =c;

mean_rider.nofb.black_box(i).h=[ mean(roll_rate_h_nofb(:,:,i),2) mean(roll_angle_h_nofb(:,:,i),2) ...
                                 mean(steer_angle_h_nofb(:,:,i),2) mean(yaw_angle_h_nofb(:,:,i),2)];
                          
mean_rider.nofb.black_box(i).y = convSim(mean_rider.nofb.data(i).w,mean_rider.nofb.black_box(i));

end



%%
% for sub=1:20
K3=[-120.700051250482,4.89856371477059,-249.802220370850,26.3141690734109,-92.4215987014822,3.32477589106087];
%K3=[1.325389027142608e-04,0.004874206072009,2.920027722718003e-04,1.000170517451996,-3.329751949399863e-06,9.613144448287691e-05]*100;
% K3=[-2.471879640219054e+02,19.151950368870505,-4.988194664604191e+02,-25.030527464060246,-1.853824710309890e+02,12.043496275409352];
%for sub=18:18
% mean_rider=results(sub);
options = optimoptions('ga', "PopulationSize", 80, ...
  'CrossoverFraction', 0.85,'FitnessLimit',0.2, ...
  'InitialPopulationRange', [-110; 110]);% ...
  %'InitialPopulationMatrix',K3);
lb=-ones(6,1)*250;
ub=ones(6,1)*250;
delay=10;
% mean_rider=results(sub);
i=4;
while(i<5)
 dat=mean_rider.fb.data(i);
 np=mean_rider.fb.black_box(i);
 %resample for faster computation.
 np.y=resample(np.y,1,5);
 np.t=0:0.005:(length(np.y)-1)*0.005;
 dat.t=np.t;
 dat.w=resample(dat.w,1,5);
 dat.Fs=200;
 bike=delftbikeHeading(dat.v); 
 rider=getRiderModel(bike,2 * pi * 2.17,delay,dat);
 [X0, ~, ~, ~] = ga(@(X)statefbError2(X,np,rider, dat,3,1),6, [], [], [],[], lb,ub, [],options);
 [K2 , fval2, ~, ~] = fmincon(@(X)statefbError2(X,np,rider, dat,3,1),X0,[], [], [],[], lb,ub, [],[]); 
%   if( fval2>7e-4)
%     continue
%   else
   output =modelSim3(K2,rider,dat,"on");
   output.K=K2;
   r(24).final_model(i)= output; 
   i=i+1;
%   end
end
%%
% for sub=1:20
K3=[-120.700051250482,4.89856371477059,-249.802220370850,26.3141690734109,-92.4215987014822,3.32477589106087];
%K3=[1.325389027142608e-04,0.004874206072009,2.920027722718003e-04,1.000170517451996,-3.329751949399863e-06,9.613144448287691e-05]*100;
% K3=[-2.471879640219054e+02,19.151950368870505,-4.988194664604191e+02,-25.030527464060246,-1.853824710309890e+02,12.043496275409352];
%for sub=18:18
% mean_rider=results(sub);
options = optimoptions('ga', "PopulationSize", 80, ...
  'CrossoverFraction', 0.85,'FitnessLimit',0.02, ...
  'InitialPopulationRange', [-110; 110]);% ...
  %'InitialPopulationMatrix',K3);
lb=[-250 -0 -250 -250 -250 -250];
ub=[0 250 0 250 0 250 ];
delay=10;
% mean_rider=results(sub);
i=4;
while(i<5)
 dat=mean_rider.fb.data(i);
 np=mean_rider.fb.black_box(i);
 %resample for faster computation.
  np.y=[r(30).final_model(i).roll_rate r(30).final_model(i).roll_angle r(30).final_model(i).steer_angle r(30).final_model(i).heading] ;
%  np.y=resample(np.y,1,5);
 
np.t=0:0.005:(length(np.y)-1)*0.005;
 dat.t=np.t;
 dat.w=resample(dat.w,1,5);
 dat.Fs=200;
 bike=delftbikeHeading(dat.v); 
 rider=getRiderModel(bike,2 * pi * 2.17,delay,dat);
 [X0, ~, ~, ~] = ga(@(X)statefbError2(X,np,rider, dat,3,2),6, [], [], [],[], lb,ub, [],options);
 [K2 , fval2, ~, ~] = fmincon(@(X)statefbError2(X,np,rider, dat,3,2),X0,[], [], [],[], lb,ub, [],[]); 
  if( fval2>2e-04)
    continue
  else
   output =modelSim3(K2,rider,dat,"off");
   output.K=K2;
   r(32).final_model(i)= output; 
   i=i+1;
   end
end

%end

%%
for i=1
%   mean_rider=results(3);
 dat=mean_rider.fb.data(i);
 np=mean_rider.fb.black_box(i);
 np.y=resample(np.y,1,5);
 np.t=0:0.005:(length(np.y)-1)*0.005;
 dat.t=np.t;
 dat.w=resample(dat.w,1,5);
 dat.Fs=200;
 bike=delftbikeHeading(dat.v); 
 rider=getRiderModel(bike,2 * pi * 2.17,delay,dat);
 output =modelSim3(r(32).final_model(i).K,rider,dat,"off");

 plotSimData(output,np,dat)
 
end


%%
 for sub=2
%for sub=18:18
% mean_rider=results(sub);
options = optimoptions('ga', "PopulationSize", 80, ...
  'CrossoverFraction', 0.85,'FitnessLimit',0.2, ...
  'InitialPopulationRange', [-110; 110]);%, ...
  %'InitialPopulationMatrix',K3);
lb=[-250 -250 -250 -250 -250 -250 0 0];
lb(1:6)=lb(1:6)*inf;
ub=[250 250 250 250 250 250 inf inf ]*inf;
delay=10;
mean_rider=results(sub);
i=2;
while(i<5)
 dat=mean_rider.fb.data(i);
 np=mean_rider.fb.black_box(i);
 %resample for faster computation.
 np.y=resample(np.y,1,5);
 np.t=0:0.005:(length(np.y)-1)*0.005;
 dat.t=np.t;
 dat.y=resample(dat.y,1,5);
 dat.w=resample(dat.w,1,5);

 dat.Fs=200;
 bike=delftbikeHeadingSteer(dat.v); 
 rider=getRiderModel(bike,2 * pi * 2.17,delay,dat);
 [X0, ~, ~, ~] = ga(@(X)statefbError3(X,np,rider, dat,2,1),5, [], [], [],[], lb,ub, [],options);
 [K2 , fval2, ~, ~] = fmincon(@(X)statefbError3(X,np,rider, dat,2,1),X0,[], [], [],[], lb,ub, [],[]); 
%   if( fval2>0.0025)
%     continue
%   else
   output =modelSim3(K2,rider,dat,"on");
   output.K=K2;
   r(26).final_model(i)= output; 
   i=i+1;
%    end
end
 end
%%
for i=3
  mean_rider=results(2);
 dat=mean_rider.fb.data(i);
 np=mean_rider.fb.black_box(i);
 np.y=resample(np.y,1,5);
 np.t=0:0.005:(length(np.y)-1)*0.005;
 dat.t=np.t;
 dat.w=resample(dat.w,1,5);
 dat.Fs=200;
 bike=delftbikeHeadingSteer(dat.v); 
 rider=getRiderModel(bike,2 * pi * 2.17,delay,dat);
 output =modelSim4(r(2).final_model(i).K,rider,dat,"on");

 plotSimDataSteer(output,np,dat)
 
end