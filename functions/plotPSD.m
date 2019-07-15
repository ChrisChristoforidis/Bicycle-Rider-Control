function plotPSD(data_on, data_off,Name)

  figure()

for i=1:length(data_on)
  idx = find(data_on(i).f < 5);
  subplot(length(data_on),1,i)
  plot(data_on(i).f(idx),data_on(i).PSD(idx))  
  yyaxis right 
  plot(data_off(i).f(idx),data_off(i).PSD(idx),'--')  

ylabel('PSD [rad^{2}/Hz]')
legend("FB ON at "+ num2str(data_on(i).v)+" m/s","FB OFF at "+ num2str(data_off(i).v)+" m/s")
end
sgtitle("Subject: "+ Name);

end