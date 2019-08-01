
clear;
addpath(genpath('functions'), genpath('Measurements'),genpath('simulink'));
close all;
%%
%save('C:\dev\Bicycle-Rider-Control\Measurements\steer\all_data_steer_pranav.mat')
% load('C:\dev\Bicycle-Rider-Control\Measurements\all_data_rope.mat');
r(1)=load('Measurements\roll\all_data_lars.mat');
r(2)=load('Measurements\roll\all_data_leo.mat');
r(3)=load('Measurements\roll\all_data_dani.mat');
r(4)=load('Measurements\roll\all_data_pier.mat');
r(5)=load('Measurements\roll\all_data_koen.mat');
r(6)=load('Measurements\roll\all_data_prodromos.mat');
r(7)=load('Measurements\roll\all_data_wylke.mat');
r(8)=load('Measurements\roll\all_data_nikos.mat');
r(9)=load('Measurements\roll\all_data_katerina.mat');
r(10)=load('Measurements\roll\all_data_marko.mat');
r(11)=load('Measurements\roll\all_data_jork.mat');
r(12)=load('Measurements\roll\all_data_thomas.mat');
r(13)=load('Measurements\roll\all_data_camilo.mat');
r(14)=load('Measurements\roll\all_data_michael.mat');
r(15)=load('Measurements\roll\all_data_pranav.mat');
r(16)=load('Measurements\roll\all_data_naveen.mat');
r(17)=load('Measurements\roll\all_data_iris.mat');
r(18)=load('Measurements\roll\all_data_vinayak.mat');
r(19)=load('Measurements\roll\all_data_vasilis.mat');
r(20)=load('Measurements\roll\all_data_leon.mat');

%save('C:\dev\Bicycle-Rider-Control\Measurements\all_subjects');
for i=1:length(r)
  if (i==1)
    r(i).results.nofb.data(2)=[];
    r(i).results.nofb.black_box(2)=[];

    r(i).results.nofb.data(5)=[];
    r(i).results.nofb.black_box(5)=[];

  elseif(i==2)
    r(i).results.nofb.data(2)=[];
    r(i).results.nofb.black_box(2)=[];

    r(i).results.fb.data(5)=[];
    r(i).results.fb.black_box(5)=[];

  elseif(i==3)
    r(i).results.nofb.data(3)=[];
    r(i).results.nofb.black_box(3)=[];

    r(i).results.fb.data(2)=[];
    r(i).results.fb.black_box(2)=[];

    r(i).results.fb.data(5)=[];
    r(i).results.fb.black_box(5)=[];
  elseif(i==5)
    r(i).results.nofb.data(2)=[];
    r(i).results.nofb.black_box(2)=[];
    r(i).results.nofb.data(2)=[];
    r(i).results.nofb.black_box(2)=[];
  elseif(i==6)
    r(i).results.nofb.data(4)=[];
    r(i).results.nofb.black_box(4)=[];
  elseif(i==10)
    r(i).results.nofb.data(4)=[];
    r(i).results.nofb.black_box(4)=[];
  elseif(i==11)
    r(i).results.nofb.data(2)=[];
    r(i).results.nofb.black_box(2)=[];
    r(i).results.nofb.data(3)=[];
    r(i).results.nofb.black_box(3)=[];
  elseif(i==12)
    r(i).results.nofb.data(2)=[];
    r(i).results.nofb.black_box(2)=[];
    r(i).results.fb.data(3)=[];
    r(i).results.fb.black_box(3)=[];
  elseif(i==13)
    r(i).results.fb.data(4)=[];
    r(i).results.fb.black_box(4)=[];
  elseif(i==14)
    r(i).results.fb.data(4)=[];
    r(i).results.fb.black_box(4)=[];
  elseif(i==16)
    r(i).results.nofb.data(4)=[];
    r(i).results.nofb.black_box(4)=[];

  elseif(i==19)
    r(i).results.fb.data(3)=[];
    r(i).results.fb.black_box(3)=[];
  elseif(i==20)
    r(i).results.fb.data(1)=[];
    r(i).results.fb.black_box(1)=[];
  end
end


%%
fb_results.data=fb_runs;

nofb_results.data=nofb_runs;
%%

for i=1:length(fb_results.data)
dat=fb_results.data(i); 
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
results.Name='Leon';

clearvars -except results

%%

i=4;
dat=nofb_results.black_box(i); 
%np=fb_results.black_box(i); 

figure('units','normalized','outerposition',[0 0 1 1])
plot(dat.t,dat.y(:,2)*180/pi);
hold on
plot(dat.t,dat.y(:,1)*180/pi)
plot(dat.t,dat.roll*180/pi,'r--')
dat=fb_results.data(i); 
plot(dat.t,zeros(length(dat.w),1))
ylim([-0.3*180/pi 0.3*180/pi])
yyaxis right


plot(dat.t,dat.w)
ylim([-max(dat.w)-1 max(dat.w)+1]);



%% Create Generic Impulse
[fb_results,nofb_results] = getMeanResponse(r(1:19));
 data=cmpIRoutput(r,fb_results,nofb_results);
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
%% Steer-by-wire Bicycle-Whipple model 
load('JBike6MCK.mat', 'C1', 'M0', 'K2', 'K0')

a = -fliplr(eye(2)) + eye(2);
M0 = a .* M0;
C1 = C1 .* a;
K0 = K0 .* a;
K2 = K2 .* a;
Hfw = [0.84; 0.014408]; % dfx/dTq

whipple.M0 = M0;
whipple.C1 = C1;
whipple.K0 = K0;
whipple.K2 = K2;
whipple.Hfw = Hfw;



%%

options = optimoptions('ga', "PopulationSize", 80, ...
  'EliteCount', 4, 'CrossoverFraction', 0.85, ...
  'InitialPopulationRange', [-50; 50],...
'InitialPopulationMatrix',[-27.134431050886300,3.849868526005710,2.866153138996350,-0.202837584515700,0]);


 lb=[-250;-250;-250;-250];
 ub=[250;250;250;250];
 %X0=[-17.4842654843009,2.26661177007119,-1.36578045475365,2.72081589849312 ];
%X0=[-9.905998751998742e-05,-9.817081242020056e-04,3.876577264354629e-07,-1.000010935768665];
X0=[-27.134431050886300,3.849868526005710,2.866153138996350,-0.202837584515700,0.];
for ii=12:12
  
fb_results=r(ii).results.fb;
nofb_results=r(ii).results.nofb; 
 
for i=2:2
 np=fb_results.black_box(i);
 dat=fb_results.data(i);
 bike = delftbikeHeading(dat.v,whipple); 
 [K, fval, ~, ~] =fmincon(@(X)statefbError2(X,np,bike,dat),X0,[], [], [], [], lb, ub, [],options);
 output =modelSim(K,bike,dat);
 output.K=K;
 output.v=dat.v;
 fb_results.final_model(i)= output;
end

r(ii).results.fb=fb_results;
r(ii).results.nofb=nofb_results;
end
%%
for i=2:2
output= fb_results.final_model(i) ;
np=fb_results.black_box(i);
dat=fb_results.data(i);
plotSimData(modelSim(output.K,bike,dat),np,dat)
end
%%
options = optimoptions('ga', "PopulationSize", 80, ...
  'EliteCount', 4, 'CrossoverFraction', 0.85, ...
  'InitialPopulationRange', [-50; 50] ...
);
%  lb=zeros(7,1);
%  ub=ones(7,1)*inf;

 lb=[-inf;-inf;-inf;-inf];
 ub=[inf;inf;inf;inf;-30];
X0=[-27.134431050886300,3.849868526005710,2.866153138996350,-0.202837584515700,0.];
for i=2:2
 dat=data(i);
 dat.N=length(data(i).w);
 np=dat;
 bike=delftbikeHeading(dat.v,whipple);
 [K, fval, ~, ~] =ga(@(X)statefbError2(X,np,bike, dat),5,[], [], [], [],[], [], [],options);
 output =modelSim(K,bike,dat);
 output.K=K;
 final_model(i)= output;
end
%%
for i=2:2
output= final_model(i) ;
np=data(i);
dat=data(i);
plotSimData(output,np,dat)
end