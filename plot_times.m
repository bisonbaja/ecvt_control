results_wem=[];
results_times=[];
results_v = [];
wet_values = [2200:200:3800];
Rfd_values = [6:0.1:7];
for Rfd = Rfd_values
  cvtmath_stepwise
  results_wem = [results_wem ; wem];
  results_times=[results_times acceltime];
  results_v = [results_v ; v];
end

function multiplot(x, data)
  hold on
  for i=[1:size(data,1)]
    plot(x, data(i,:))
  endfor
  hold off
end

figure
title("Engine RPM, ideal vs real")
xlabel("Time (s)")
ylabel("RPM")
line([0 5], [3800 3800], "linestyle", "-", "color", "r")
%line([0 8], [wet*60/(2*pi) wet*60/(2*pi)], "linestyle", "-", "color", "k")
grid minor on

multiplot(t, results_wem*(60/(2*pi));

figure
multiplot(t, results_v);

figure
plot(Rfd_values, results_times);
title("Accel time vs Rfd")
grid minor on

