load('results_steer.mat');



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

%%
for k=1:length(results)
  for i=1:length(results(k).fb.data)
    dat=results(k).fb.data(i);
    dat.Rates(:,1)=dat.Rates(:,1)-median(dat.Rates(:,1));
    dat.y=detrend(cumtrapz(dat.Rates(:,1))*0.001,5);
    np = nonparaID(dat);  
    results(k).fb.black_box(i).h(:,1)=np.h(:,1);
    results(k).fb.black_box(i).y(:,1)=np.y(:,1);

  end
  
  
end