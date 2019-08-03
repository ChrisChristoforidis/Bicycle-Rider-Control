
clear;
addpath(genpath('functions'), genpath('Measurements'),genpath('simulink'));
close all;
%%

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




%% Create Generic Impulse
[fb_results,nofb_results] = getMeanResponse(r(1:20));
%data2=cmpIRoutput(r,fb_results,nofb_results);
w=r(12).results.fb.data(3).w;
t=r(12).results.fb.data(3).t.';
n=length(fb_results.black_box);
for i=1:n
np=fb_results.black_box(i); 
y_1=convSim(w,np);
data(i).y=y_1;
data(i).v=fb_results.data(i).v;
data(i).t=t;
data(i).w=w;
end


%% Steer-by-wire Bicycle-Whipple model 
whipple=getbicycleEOM();


%%

% options = optimoptions('ga', "PopulationSize", 80, ...
%   'EliteCount', 4, 'CrossoverFraction', 0.85, ...
%   'InitialPopulationRange', [-50; 50],...
% 'InitialPopulationMatrix',[-27.134431050886300,3.849868526005710,2.866153138996350,-0.202837584515700,0]);
% 
% 
%  lb=[-250;-250;-250;-250];
%  ub=[250;250;250;250];
%  %X0=[-17.4842654843009,2.26661177007119,-1.36578045475365,2.72081589849312 ];
% %X0=[-9.905998751998742e-05,-9.817081242020056e-04,3.876577264354629e-07,-1.000010935768665];
% X0=[-27.134431050886300,3.849868526005710,2.866153138996350,-0.202837584515700,0.];
% for ii=12:12
%   
% fb_results=r(ii).results.fb;
% nofb_results=r(ii).results.nofb; 
%  
% for i=2:2
%  np=fb_results.black_box(i);
%  dat=fb_results.data(i);
%  bike = delftbikeHeading(dat.v); 
%  [K, fval, ~, ~] =fmincon(@(X)statefbError2(X,np,bike,dat),X0,[], [], [], [], lb, ub, [],options);
%  output =modelSim(K,bike,dat);
%  output.K=K;
%  output.v=dat.v;
%  fb_results.final_model(i)= output;
% end
% 
% r(ii).results.fb=fb_results;
% r(ii).results.nofb=nofb_results;
% end
%
% for i=2:2
% output= fb_results.final_model(i) ;
% np=fb_results.black_box(i);
% dat=fb_results.data(i);
% plotSimData(modelSim(output.K,bike,dat),np,dat)
% end
%%
options = optimoptions('ga', "PopulationSize", 80, ...
  'CrossoverFraction', 0.85,'FitnessLimit',0.02, ...
  'InitialPopulationRange', [-50; 50] ...
);
lb=-ones(6,1)*500;
up=ones(6,1)*500;
X0=[-27.134431050886300,3.849868526005710,2.866153138996350,-0.202837584515700,0.,0.];
for i=1:4
 dat=data(i);
 dat.N=length(data(i).w);
 np=dat;
 bike=delftbikeHeading(dat.v); 
 [X0, fval, ~, ~] = ga(@(X)statefbError2(X,np,bike, dat),6, [], [], [],[], lb,ub, [],options);
 [K , fval, ~, ~] = fmincon(@(X)statefbError2(X,np,bike, dat),X0,[], [], [],[], lb,ub, [],[]);
 output =modelSim(K,bike,dat);
 output.K=K;
 final_model(i)= output;
end
%%
for i=1:4
output= final_model(i) ;
np=data(i);
dat=data(i);
plotSimData(output,np,dat)
end

