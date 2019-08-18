
clear;
addpath(genpath('functions'), genpath('Measurements'),genpath('simulink'));
close all;


%% 
r=zeros(20,1);
load('results.mat');

% for i=1:length(r)
%   if (i==1)
%     
%     r(i).results.nofb.data(2)=[];
%     r(i).results.nofb.black_box(2)=[];
% 
%     r(i).results.nofb.data(5)=[];
%     r(i).results.nofb.black_box(5)=[];
% 
%   elseif(i==2)
%     
%     r(i).results.nofb.data(2)=[];
%     r(i).results.nofb.black_box(2)=[];
% 
%     r(i).results.fb.data(5)=[];
%     r(i).results.fb.black_box(5)=[];
% 
%   elseif(i==3)
%     
%     r(i).results.nofb.data(3)=[];
%     r(i).results.nofb.black_box(3)=[];
% 
%     r(i).results.fb.data(2)=[];
%     r(i).results.fb.black_box(2)=[];
% 
%     r(i).results.fb.data(5)=[];
%     r(i).results.fb.black_box(5)=[];
%   elseif(i==5)
%     
%     r(i).results.nofb.data(2)=[];
%     r(i).results.nofb.black_box(2)=[];
%     r(i).results.nofb.data(2)=[];
%     r(i).results.nofb.black_box(2)=[];
%     
%   elseif(i==6)
%     
%     r(i).results.nofb.data(4)=[];
%     r(i).results.nofb.black_box(4)=[];
%     
%   elseif(i==10)
%     
%     r(i).results.nofb.data(4)=[];
%     r(i).results.nofb.black_box(4)=[];
%     
%   elseif(i==11)
%     
%     r(i).results.nofb.data(2)=[];
%     r(i).results.nofb.black_box(2)=[];
%     r(i).results.nofb.data(3)=[];
%     r(i).results.nofb.black_box(3)=[];
%     
%   elseif(i==12)
%     
%     r(i).results.nofb.data(2)=[];
%     r(i).results.nofb.black_box(2)=[];
%     
%     r(i).results.fb.data(3)=[];
%     r(i).results.fb.black_box(3)=[];
%     
%   elseif(i==13)
%     
%     r(i).results.fb.data(4)=[];
%     r(i).results.fb.black_box(4)=[];
%     
%   elseif(i==14)
%     
%     r(i).results.fb.data(4)=[];
%     r(i).results.fb.black_box(4)=[];
%     
%   elseif(i==16)
%     
%     r(i).results.nofb.data(4)=[];
%     r(i).results.nofb.black_box(4)=[];
% 
%   elseif(i==19)
%     
%     r(i).results.fb.data(3)=[];
%     r(i).results.fb.black_box(3)=[];
%     
%   elseif(i==20)
%     
%     r(i).results.fb.data(1)=[];
%     r(i).results.fb.black_box(1)=[];
%     
%   end
% end


%% Roll rate FIR

for kk=1:length(r)
  for k=1:4
    dat=r(kk).results.fb.data(k);
    dat.y=dat.Rates(:,1);
    np=nonparaID(dat);
    r(kk).results.fb.black_box(k).y2=np.y(:,1);
    r(kk).results.fb.black_box(k).h2=np.h(:,1);
    
  end
end

%%a

%%

for ii=1:20
  for k=1:4
    plot(r(kk).results.fb.black_box(k).h2);
    pause()
  end
  
end


for kk=1:4
  data2(:,kk)=K(:,:,kk);
  
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

w=r(12).results.fb.data(3).w;
t=r(12).results.fb.data(3).t.';
n=size(data2,2);
for i=1:n
np.h=[data2(:,i) fb_results.black_box(i).h];
y_1=convSim(w,np);
data(i).y=y_1;
data(i).v=fb_results.data(i).v;
data(i).t=t;
data(i).w=w;
end

for i=1:n
np=nofb_results.black_box(i); 
y_1=convSim(w,np);
data2(i).y=y_1;
data2(i).v=fb_results.data(i).v;
data2(i).t=t;
data2(i).w=w;
end
%%

for k=1:length(r)
 for kk=1:4
    data=r(k).results.fb.data(kk) ;
    [COMx ,COMy,~]=findCOM(data.f,data.PSD,0,5);
    data=r(k).results.nofb.data(kk) ;
    [COMx2 ,COMy2,~]=findCOM(data.f,data.PSD,0,5);
      v.compare(kk).PSD(k,:)=[COMx COMy  COMx2 COMy2];
 end
end


%%
figure(2)
vel=[2.6 3.7 4.5 5.6];
for i =1:4
 subplot(4,1,i)
   plot(v.compare(i).PSD(:,1),v.compare(i).PSD(:,2),'c*');
   hold on
   grid on
   plot(mean(v.compare(i).PSD(:,1)),mean(v.compare(i).PSD(:,2)),'b*','LineWidth',3);
   plot(v.compare(i).PSD(:,3),v.compare(i).PSD(:,4),'*','Color','#F08080');
   plot(mean(v.compare(i).PSD(:,3)),mean(v.compare(i).PSD(:,4)),'r*','LineWidth',3);
   xlim([0 2])
   title(vel(i)+" m/s",'Interpreter','latex');
      ylabel('$S_\delta(f) \;\;  (rad^{2}/Hz)$ ','Interpreter','latex')
end
xlabel('$Frequency   \;\; (Hz)$','Interpreter','latex');
legend('FB off','FB off (mean)','FB on','FB on (mean)','Interpreter','latex','Location','best')
ax=gca;

ax.ZTick=[0 2.6 3.7 4.5 5.6];


%%
string="";
clc
for i = 1:4
b=v.compare(i).values(:,1);
%[h,p,ci,stats] = ttest2(v.compare(i).PSD(:,1),v.compare(i).PSD(:,3));
for kk=100:-1:80
[h,p,ci,stats] = ttest(b,kk,'Tail','right');
if(h==1)
  "for speed level "+i+" VAF "+kk
  break;
end
end

% string = " For forward speed  "+vel(i)+" m/s there was no significant difference in the scores for feedback on (M = "+round(mean(v.compare(i).PSD(:,1)),2)+", SD = "+round(std(v.compare(i).PSD(:,1)),2)+ ...
%   ") and feedback off (M = "+ round(mean(v.compare(i).PSD(:,1)),2)+", SD = "+round(std(v.compare(i).PSD(:,1)),2)+") conditions;t("+stats.df+")="+round(stats.tstat,2)+" ,p="+round(p,4)+"."
%  string =" For forward speed  "+vel(i)+" m/s there was no significant difference  (M = "+round(mean(v.compare(i).delay(:,2)),2)+", SD = "+round(std(v.compare(i).delay(:,2)),2)+ ...
%    ") between signals; t("+stats.df+")="+round(stats.tstat,2)+", p="+round(p,4)+"."
end

%%
for k=1:length(r)
  
  for kk=1:4
    w=r(k).results.fb.data(kk).w;
    np=r(k).results.fb.black_box(kk) ;
%     v_roll=vaf(data.y(:,1),np.y(:,1));
%     v_steer=vaf(data.y(:,2),np.y(:,2));
%     v.fb(kk).values(k,:)=[ v_roll v_steer];
%     COMx=trapz(data(kk).f,data(kk).f.*data(kk).PSD/max(data(kk).PSD))/trapz(data(kk).f,data(kk).PSD/max(data(kk).PSD));
%     y_1=convSim(w,np);
    np2=r(k).results.nofb.black_box(kk) ;
%     data=r(k).results.nofb.data(kk) ;
%     v_roll=vaf(data.y(:,1),np2.y(:,1));
%     v_steer=vaf(data.y(:,2),np2.y(:,2));
%     v.nofb(kk).values(k,:)=[ v_roll v_steer]; 
%     y_2=convSim(w,np2);
%     v_roll=vaf(y_1(:,1),y_2(:,1));
%     v_steer=vaf(y_1(:,2),y_2(:,2));
%     v.compare(kk).values(k,:)=[ v_roll v_steer];
    D = finddelay(np.h,np2.h);
    v.compare(kk).delay(k,:)=D;
    v.compare(kk).ME(k,:)= mean(np.h-np2.h);
    v.compare(kk).values(k,1)= vaf(np2.h(:,1),np.h(:,1));
    v.compare(kk).values(k,2)= vaf(np2.h(:,2),np.h(:,2));

%         dt=0.001;Fs=1000;
%         data_on.f = (0:length(np.y(:,2)) / 2).' / length(np.y(:,2)) / dt;
%         data_off.f = (0:length(np2.y(:,2)) / 2).' / length(np2.y(:,2)) / dt;
%         [data_on.PSD, ~] = periodogram(np.y(:,2), hann(length(np.y(:,2))),length(np.y(:,2)),Fs);
%         [data_off.PSD, ~] = periodogram(np2.y(:,2), hann(length(np2.y(:,2))),length(np2.y(:,2)),Fs);
        
%         clf;figure(2)
%         idx = find(data_on.f < 5);
%         plot(data_on.f(idx),data_on.PSD(idx))  
%         yyaxis right 
%         plot(data_off.f(idx),data_off.PSD(idx),'--') 
%         pause()
%     clf
%     figure(1)
%     subplot(211)
%     plot(y_1(:,2));hold on;plot(y_2(:,2))
%     ylabel("STEER")
%     title("VAF "+ v.compare(kk).values(k,2));
%     subplot(212)
%     plot(y_1(:,1));hold on;plot(y_2(:,1))
%     ylabel("ROLL")
%     legend('fb on','fb off');
%     title("VAF "+ v.compare(kk).values(k,1));
%     pause()


   
  end
  
end
%%
for i =1:4
  z(:,i)=v.fb(i).values(:,1);
  zz(:,i)=v.fb(i).values(:,2);
  zzz(:,i)=v.nofb(i).values(:,1);
  zzzz(:,i)=v.nofb(i).values(:,2);

end

  y=[z ; zzz];
 yy=[zz; zzzz];
figure()
subplot(211)
boxplot(y)
ylabel('Roll Angle VAF (%)')
ax=gca;
ax.XTickLabel={'2.6' '3.7' '4.5' '5.6'};

subplot(212)
boxplot(yy)
ylabel('Steer Angle VAF (%)')
xlabel('Speed Levels')
ax=gca;
ax.XTickLabel={'2.6' '3.7' '4.5' '5.6'};
%%

z=[];
zz=[]
for i =1:4
  z(:,i)=v.compare(i).delay(:,1);
  zz(:,i)=v.compare(i).delay(:,2);
end


% figure()
% subplot(211)
% boxplot(z)
% ylabel('Roll Angle Delay (ms)')
% ax=gca;
% ax.XTickLabel={'2.6' '3.7' '4.5' '5.6'};

figure()
boxplot(zz)
ylabel('$d_\delta  \;\;\;\;$ (ms)','Interpreter','latex')
xlabel('$v \;\;\;\;$ (m/s)','Interpreter','latex')
ax=gca;
grid on
ax.XTickLabel={'2.6' '3.7' '4.5' '5.6'};



%%
z=[];
zz=[];
for i =1:4
  z=[z;v.compare(i).values(:,1)];
  zz=[zz;v.compare(i).values(:,2)];
end
g=[2.6*ones(length(v.compare(1).values(:,1)), 1); 3.7*ones(length(v.compare(2).values(:,1)), 1); ...
  4.5*ones(length(v.compare(3).values(:,1)), 1);5.6*ones(length(v.compare(4).values(:,1)), 1)];
gg=[2.6*ones(length(v.compare(1).values(:,2)), 1); 3.7*ones(length(v.compare(2).values(:,2)), 1); ...
  4.5*ones(length(v.compare(3).values(:,2)), 1);5.6*ones(length(v.compare(4).values(:,2)), 1)];
figure()
% subplot(211)
boxplot(z,g)
ylabel('$\mathit{VAF}_{\phi} \;\;\;\;\;(\%)$','Interpreter','latex')
xlabel('$v \;\;\;\;$ (m/s)','Interpreter','latex')
% ax=gca;
% ax.XTickLabel={'2.6' '3.7' '4.5' '5.6'};
grid on

% subplot(212)
% boxplot(zz,gg)
% ylabel('Steer Angle VAF (%)')
% xlabel('Speed Levels')
% ax=gca;
% ax.XTickLabel={'2.6' '3.7' '4.5' '5.6'};
%%
for jj=1:4
 RMSE(jj,:)= sqrt(mean((data(jj).y-data2(jj).y).^2));
 va(1,jj)=vaf(data(jj).y(:,1),data2(jj).y(:,1));
 va(2,jj)=vaf(data(jj).y(:,2),data2(jj).y(:,2));

end
%%
j=1;
figure()
subplot(313)
plot(data(j).t,data(j).y(:,1),'b--')
ylabel('$\hat{\phi}_{mean}(t)$  (rad)','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')

hold on
grid on
plot(data2(j).t,data2(j).y(:,1),'r')
xlim([data(j).t(1) data(j).t(end)])
subplot(312)
plot(data(j).t,data(j).y(:,2),'b--')
hold on
grid on
plot(data2(j).t,data2(j).y(:,2),'r')
ylabel('$\hat{\delta}_{mean}(t)$  (rad)','Interpreter','latex')
xlim([data(j).t(1) data(j).t(end)])
legend('Feedback On', 'Feedback Off' ,'Interpreter','latex')

subplot(311)
plot(data(j).t,data(j).w)
ylabel('w  (N)','Interpreter','latex')
grid on
xlim([data(j).t(1) data(j).t(end)])
title("Simulated output for the mean rider at speed "+round(data(j).v,2)+" m/s",'Interpreter','latex');
%%
figure(1)
 
 for kk=1:4
 subplot(2,4,kk)
  t=[0:0.001:0.001*(length(fb_results.black_box(kk).h(:,1))-1)].';

 amean=fb_results.black_box(kk).h(:,1)*1e+3;
 astd=fb_results.black_box(kk).std(:,1)*1e+3;
 p1(i,kk)=plot(t,amean,'b--');
 grid on;
 hold on
 if (i==length(r))
 fill([t ;flipud(t)],[amean+astd  ;flipud(amean-astd)],'b', 'FaceAlpha', 0.2,'linestyle','none','HandleVisibility','off');
 end
 amean=nofb_results.black_box(kk).h(:,1)*1e+3;
 astd=nofb_results.black_box(kk).std(:,1)*1e+3;

 p2(i,kk)=plot(t,amean,'r');
 if (i==length(r))
 fill([t ;flipud(t)],[amean+astd  ;flipud(amean-astd)],'r', 'FaceAlpha', 0.2,'linestyle','none','HandleVisibility','off');
 end
 if(kk==1)
     legend('Feedback On','Feedback Off','Interpreter','latex')

 ylabel('$\hat{h}_\delta(\tau)\;\;\;\;$     (rad  $\mathit{mN}^{-1}$)','Interpreter','latex')
 end

  title( round(fb_results.data(kk).v,2)+"m/s" ,'Interpreter','latex')

 subplot(2,4,kk+4)
 amean=fb_results.black_box(kk).h(:,2)*1e+3;
 astd=fb_results.black_box(kk).std(:,2)*1e+3;
 
  p3(i,kk)=plot(t,amean,'b--','HandleVisibility','off');
  hold on
  grid on
 if (i==length(r))
 fill([t ;flipud(t)],[amean+astd  ;flipud(amean-astd)],'b', 'FaceAlpha', 0.2,'linestyle','none','HandleVisibility','off');
 end
 amean=nofb_results.black_box(kk).h(:,2)*1e+3;
 astd=nofb_results.black_box(kk).std(:,2)*1e+3;
 
 p4(i,kk)=plot(t,amean,'r','HandleVisibility','off');
 if(i==length(r))
 fill([t ;flipud(t)],[amean+astd  ;flipud(amean-astd)],'r', 'FaceAlpha', 0.2,'linestyle','none','HandleVisibility','off');
 end
 if (kk==1)
 ylabel('$\hat{h}_\delta(\tau)\;\;\;\;$      (rad  $\mathit{mN}^{-1}$)','Interpreter','latex')
 end
 xlabel('$\tau(s)$','Interpreter','latex')
   if (i==length(r))
   p1(i,kk).LineWidth=1.2;
   p2(i,kk).LineWidth=1.2;
   p3(i,kk).LineWidth=1.2;
   p4(i,kk).LineWidth=1.2;
 end  
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
ub=ones(6,1)*500;
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

%%
sub=3;
options = optimoptions('ga', "PopulationSize", 80, ...
  'CrossoverFraction', 0.85,'FitnessLimit',0.02, ...
  'InitialPopulationRange', [-50; 50] ...
);
lb=-ones(6,1)*500;
ub=ones(6,1)*500;
for i=1:1
 dat=r(sub).results.fb.data(i);
 np=r(sub).results.fb.black_box(i);
 bike=delftbikeHeading(dat.v); 
 [X0, fval, ~, ~] = ga(@(X)statefbError2(X,np,bike, dat),6, [], [], [],[], lb,ub, [],options);
 [K , fval, ~, ~] = fmincon(@(X)statefbError2(X,np,bike, dat),X0,[], [], [],[], lb,ub, [],[]);
 output =modelSim(K,bike,dat);
 output.K=K;
 final_model(i)= output;
 plotSimData(output,np,dat)

end

%%
for i=1:1
output= final_model(i) ;
end