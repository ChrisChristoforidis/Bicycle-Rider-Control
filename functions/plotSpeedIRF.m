
function plotSpeedIRF(fb_np,fb_runs,Name)

figure()
subplot(211)
for i =1:length(fb_np)
plot(fb_np(i).tau(fb_np(2).tau>=0),fb_np(i).h(:,1));
hold on
end
ylabel('Roll Angle IRF')
if fb_runs(1).FB==true
title("FEEDBACK ON Subject: "+ Name);
else 
title("FEEDBACK OFF Subject: "+ Name);
end
  for i=1:length(fb_np)
    legnd{i}=num2str(fb_runs(i).v);
  end
  legend(legnd)
subplot(212)
for i=1:length(fb_np)
plot(fb_np(i).tau(fb_np(2).tau>=0),fb_np(i).h(:,2));
hold on
end
ylabel('Steer Angle IRF')
legend(legnd)
end