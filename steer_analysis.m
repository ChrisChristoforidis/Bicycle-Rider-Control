clear;
addpath(genpath('functions'), genpath('Measurements'),genpath('simulink'));
close all;
%%
r(1)=load('Measurements\steer\all_data_steer_lars.mat');
r(2)=load('Measurements\steer\all_data_steer_leo.mat');
r(3)=load('Measurements\steer\all_data_steer_pier.mat');
r(4)=load('Measurements\steer\all_data_steer_koen.mat');
r(5)=load('Measurements\steer\all_data_steer_christodoulos.mat');
r(6)=load('Measurements\steer\all_data_steer_nikos.mat');
r(7)=load('Measurements\steer\all_data_steer_katerina.mat');
r(8)=load('Measurements\steer\all_data_steer_marko.mat');
r(9)=load('Measurements\steer\all_data_steer_jork.mat');
r(10)=load('Measurements\steer\all_data_steer_thomas.mat');
r(11)=load('Measurements\steer\all_data_steer_camilo.mat');
r(12)=load('Measurements\steer\all_data_steer_michael.mat');
r(13)=load('Measurements\steer\all_data_steer_pranav.mat');
r(14)=load('Measurements\steer\all_data_steer_iris.mat');

for i=1:length(r)
  if (i==4)

    r(i).results.fb.data(2)=[];
    r(i).results.fb.black_box(2)=[];
    
    r(i).results.fb.data(4)=[];
    r(i).results.fb.black_box(4)=[];

  elseif(i==5)

    r(i).results.fb.data(4)=[];
    r(i).results.fb.black_box(4)=[];


  elseif(i==6)


    r(i).results.fb.data(1)=[];
    r(i).results.fb.black_box(1)=[];

    r(i).results.fb.data(3)=[];
    r(i).results.fb.black_box(3)=[];
 
  elseif(i==11)
    
    r(i).results.fb.data(3)=[];
    r(i).results.fb.black_box(3)=[];


  end
end



%%

results=getMeanResponseSteer(r);

results.data(1).w=r(1).results.fb.data(2).w(1500:4500);
results.data(1).t=r(1).results.fb.data(2).t(1500:4500).';
data=cmpIRoutput(results);


%%



options = optimoptions('ga', "PopulationSize", 80, ...
  'EliteCount', 4, 'CrossoverFraction', 0.85, ...
  'InitialPopulationRange', [-50; 50],...
  'InitialPopulationMatrix',[-51.7128738871086,21.6551860444900,-51.6774198438909,25.2218831861680,0.632212175201730,0.125000000000000]);


 lb=[-250;-250;-250;-250];
 ub=[250;250;250;250];
 X0=[-17.4842654843009,2.26661177007119,-1.36578045475365,2.72081589849312 ];
X0=[-17.6389693630700,10.4281683734837,-57.4743914301518,33.1838811654843 0 0];

   
 
for i=4:4
 np=data(i);
 dat=data(i);
 np.N=length(data(i).w);
 dat.N=length(data(i).w);

 bike = delftbike(dat.v,whipple); 
 [K, fval, ~, ~] =ga(@(X)statefbError2(X,np,bike,dat),6,[], [], [], [], lb, ub, [],options);
 output =modelSim(K,bike,dat);
 output.K=K;
 final_model(i)= output;
end
%%

for i=4:4
np=data(i);
dat=data(i);
 np.N=length(data(i).w);
 dat.N=length(data(i).w);

output2= modelSim(final_model(i).K,bike,dat) ;

plotSimData(output2,np,dat)
end