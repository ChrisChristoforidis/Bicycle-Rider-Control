 function dat=cmpIRoutput(r,fb_results,varargin)
%Compares the ouput of the response  produced by the FIR for the 2 experimental conditions.
if(~isempty(varargin))
  nofb_results=varargin{1};
  flag=true;
  leg='FB On';
  
else 
  flag=false;
  leg='Response';
end

if(flag==true)
  [t,w]=generateImpulse(400,0.2);
%   w=r(1).results.fb.data(1).w;
%   t=r(1).results.fb.data(1).t.';
else
  [t,w]=generateImpulse(15,0.2);
   %w=fb_results.data(1).w;
   %t=fb_results.data(1).t;
end

figure('units','normalized','outerposition',[0 0 1 1])
n=length(fb_results.black_box);
for i=1:n
np=fb_results.black_box(i); 
y_1=convSim(w,np);
dat(i).y=y_1;
np.h = fb_results.black_box(i).std+ fb_results.black_box(i).h;
y_1std_plus=convSim(w,np)*180/pi;
np.h = -fb_results.black_box(i).std+ fb_results.black_box(i).h;
y_1std_minus=convSim(w,np)*180/pi;
  
if(flag==true)
np=nofb_results.black_box(i);
y_2=convSim(w,np);
np.h = nofb_results.black_box(i).std+ nofb_results.black_box(i).h;
y_2std_plus=convSim(w,np)*180/pi;
np.h = -nofb_results.black_box(i).std+ nofb_results.black_box(i).h;
y_2std_minus=convSim(w,np)*180/pi;
end

subplot(2,n,i)
plot(t,y_1(:,2)*180/pi,'b','DisplayName',leg,'linewidth',1.5);
hold on
fill([t ;flipud(t)],[y_1std_plus(:,2);flipud(y_1std_minus(:,2))],'b', 'FaceAlpha', 0.2,'linestyle','none','HandleVisibility','off') 
if ( flag==true)
plot(t,y_2(:,2)*180/pi,'r','DisplayName',"FB off",'linewidth',1.5)
fill([t ;flipud(t)],[y_2std_plus(:,2);flipud(y_2std_minus(:,2))],'r', 'FaceAlpha', 0.2,'linestyle','none','HandleVisibility','off') 
end
plot(t,zeros(length(t),1),'k','HandleVisibility','off')
ylim([-max(abs(y_1(:,2)))*1.4*180/pi  max(abs(y_1(:,2)))*1.4*180/pi])
title(num2str(round((fb_results.data(i).v+fb_results.data(i).v)/2*3.6,2))+" km/h")
if(i==1)
legend

ylabel('Steer Angle (deg)');
yyaxis right
ylabel('Force (N)');
end
yyaxis right


grid on 

plot(t,w,'m','DisplayName','Impulse Disturbance')
ylim([-max(abs(w))*1.2 max(abs(w))*1.2]);

subplot(2,n,i+n)
plot(t,y_1(:,1)*180/pi,'DisplayName',leg,'linewidth',1.5);
hold on
grid on
fill([t ;flipud(t)],[y_1std_plus(:,1);flipud(y_1std_minus(:,1))],'b', 'FaceAlpha', 0.2,'linestyle','none','HandleVisibility','off') 
if ( flag==true)
plot(t,y_2(:,1)*180/pi,'r','DisplayName',"FB off",'linewidth',1.5)
fill([t ;flipud(t)],[y_2std_plus(:,1);flipud(y_2std_minus(:,1))],'r', 'FaceAlpha', 0.2,'linestyle','none','HandleVisibility','off') 
end
plot(t,zeros(length(t),1),'k','HandleVisibility','off')
ylim([-max(abs(y_1(:,1)))*1.4*180/pi  max(abs(y_1(:,1)))*1.4*180/pi])
legend('off')
if(i==1)
ylabel('Roll Angle (deg)');
end
yyaxis right
plot(t,w,'m','DisplayName','Impulse Disturbance')
ylim([-max(abs(w))*1.2 max(abs(w))*1.2]);

end

dat(1).w=w;
dat(2).w=w;
dat(3).w=w;
dat(4).w=w;

dat(1).t=t.';
dat(2).t=t.';
dat(3).t=t.';
dat(4).t=t.';

dat(1).v=fb_results.data(1).v;
dat(2).v=fb_results.data(2).v;
dat(3).v=fb_results.data(3).v;
dat(4).v=fb_results.data(4).v;


end