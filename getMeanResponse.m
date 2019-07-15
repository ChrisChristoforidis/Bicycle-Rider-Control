function [fb_results, nofb_results]=getMeanResponse(r,varargin)
if(isempty(varargin))
 animation=false;
else
  animation=true; 
end

  fb_avg_vel=zeros(4,1);
  nofb_avg_vel=zeros(4,1);

for i=1:length(r)
 for jj=1:4
  irf_fb(jj).roll(:,i)=r(i).results.fb.black_box(jj).h(:,1);
  irf_fb(jj).steer(:,i)=r(i).results.fb.black_box(jj).h(:,2);
  irf_nofb(jj).roll(:,i)=r(i).results.nofb.black_box(jj).h(:,1);
  irf_nofb(jj).steer(:,i) = r(i).results.nofb.black_box(jj).h(:,2);
  nofb_avg_vel(jj) = nofb_avg_vel(jj)+r(i).results.nofb.data(jj).v;
  fb_avg_vel(jj) = fb_avg_vel(jj)+r(i).results.fb.data(jj).v;
 end
 for k=1:4
 fb_results.black_box(k).h=[mean(irf_fb(k).roll,2),mean(irf_fb(k).steer,2)];
 fb_results.black_box(k).std=[std(irf_fb(k).roll,0,2),std(irf_fb(k).steer,0,2)];

 nofb_results.black_box(k).h=[mean(irf_nofb(k).roll,2),mean(irf_nofb(k).steer,2)];
 nofb_results.black_box(k).std=[std(irf_nofb(k).roll,0,2),std(irf_nofb(k).steer,0,2)];

 fb_results.data(k).v=nofb_avg_vel(k)/i;
 nofb_results.data(k).v=fb_avg_vel(k)/i;
 end
 
 if (animation==true)
 figure(1)
 
 for kk=1:4
 subplot(2,4,kk)
  amean=fb_results.black_box(kk).h(:,1);
 astd=fb_results.black_box(kk).std(:,1);
 p1(i,kk)=plot(amean,'b');
 t=[1:length(fb_results.black_box(kk).h(:,1))].';
 hold on
 if (i==length(r))
 fill([t ;flipud(t)],[amean+astd  ;flipud(amean-astd)],'b', 'FaceAlpha', 0.2,'linestyle','none');
 end
 amean=nofb_results.black_box(kk).h(:,1);
 astd=nofb_results.black_box(kk).std(:,1);

 p2(i,kk)=plot(amean,'r');
 if (i==length(r))
 fill([t ;flipud(t)],[amean+astd  ;flipud(amean-astd)],'r', 'FaceAlpha', 0.2,'linestyle','none');
 end
 ylabel('Roll Angle IRF (rad/N)')
 subplot(2,4,kk+4)
 amean=fb_results.black_box(kk).h(:,2);
 astd=fb_results.black_box(kk).std(:,2);
 
  p3(i,kk)=plot(amean,'b');
  hold on
 if (i==length(r))
 fill([t ;flipud(t)],[amean+astd  ;flipud(amean-astd)],'b', 'FaceAlpha', 0.2,'linestyle','none');
 end
 amean=nofb_results.black_box(kk).h(:,2);
 astd=nofb_results.black_box(kk).std(:,2);
 
 p4(i,kk)=plot(amean,'r');
 if(i==length(r))
 fill([t ;flipud(t)],[amean+astd  ;flipud(amean-astd)],'r', 'FaceAlpha', 0.2,'linestyle','none');
 end
 ylabel('Steer Angle IRF (rad/N)')
for k=1:6
 if (i==length(r))
   p1(i,kk).LineWidth=2;
   p2(i,kk).LineWidth=2;
   p3(i,kk).LineWidth=2;
   p4(i,kk).LineWidth=2;
 end  
end
 for k=1:6
  if(i>k)
   p1(i-k,kk).Color(4) = abs(0.6-k*0.1);
   p2(i-k,kk).Color(4) = abs(0.6-k*0.1);
   p3(i-k,kk).Color(4) = abs(0.6-k*0.1);
   p4(i-k,kk).Color(4) = abs(0.6-k*0.1);
  end
 end
 
end

 pause()
end
end

end
