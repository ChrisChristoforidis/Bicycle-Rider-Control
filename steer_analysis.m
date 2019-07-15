clear;
addpath(genpath('functions'), genpath('Measurements'),genpath('simulink'));
close all;
%%
r(1)=load('C:\dev\bicycle-rider-control-identification\Measurements\steer\all_data_steer_lars.mat');
r(2)=load('C:\dev\bicycle-rider-control-identification\Measurements\steer\all_data_steer_leo.mat');
r(3)=load('C:\dev\bicycle-rider-control-identification\Measurements\steer\all_data_steer_pier.mat');
r(4)=load('C:\dev\bicycle-rider-control-identification\Measurements\steer\all_data_steer_koen.mat');
r(5)=load('C:\dev\bicycle-rider-control-identification\Measurements\steer\all_data_steer_christodoulos.mat');
r(6)=load('C:\dev\bicycle-rider-control-identification\Measurements\steer\all_data_steer_nikos.mat');
r(7)=load('C:\dev\bicycle-rider-control-identification\Measurements\steer\all_data_steer_katerina.mat');
r(8)=load('C:\dev\bicycle-rider-control-identification\Measurements\steer\all_data_steer_marko.mat');
r(9)=load('C:\dev\bicycle-rider-control-identification\Measurements\steer\all_data_steer_jork.mat');
r(10)=load('C:\dev\bicycle-rider-control-identification\Measurements\steer\all_data_steer_thomas.mat');
r(11)=load('C:\dev\bicycle-rider-control-identification\Measurements\steer\all_data_steer_camilo.mat');
r(12)=load('C:\dev\bicycle-rider-control-identification\Measurements\steer\all_data_steer_michael.mat');


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


    r(i).results.fb.data(2)=[];
    r(i).results.fb.black_box(2)=[];

    r(i).results.fb.data(3)=[];
    r(i).results.fb.black_box(3)=[];
 
  elseif(i==11)
    
    r(i).results.fb.data(3)=[];
    r(i).results.fb.black_box(3)=[];


  end
end



%%

results=getMeanResponseSteer(r);

results.data(1).w=r(1).results.fb.data(1).w(1500:5500);
results.data(1).t=r(1).results.fb.data(1).t(1500:5500).';
data=cmpIRoutput(results);