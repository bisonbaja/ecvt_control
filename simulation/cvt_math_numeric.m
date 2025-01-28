m=600/32.2 ;     % 600 lb to slugs
Rfd=6.418  ;     % final drive ratio
rt=10/12   ;     % inches
dt=0.001;
t=[0:dt:8];
showplots = 1;
Rdot_max = 1;
Edata_rpm =   [3800 3600 3400 3200 3000 2800 2600 2400] % RPM
Edata_torque= [12.7 13.5 14.5 15.4 16.6 17.4 18.1 18.5] % ft*lbf
Edata_power=  [9.2 9.3 9.4 9.4 9.5 9.3 9.0 8.5]        % hp
Edata_power_real = Edata_power*0.67;
Edata_power_real_lbfts = Edata_power_real*550; % Corrected power in lbf*ft/s
Edata_rads = Edata_rpm*2*pi/60; % Sampled RPM values but in rad/s
Edata_torque_real = Edata_power_real_lbfts./Edata_rads; % Corrected torque in lbf*ft

if (exist("done"))
  H=interp1(Edata_rads, Edata_power_real_lbfts, wem, "cubic", "extrap");
  a=sqrt(H/(2*m)) .* sqrt(1./t);
  a=[a(2) a(2:end)];  % throw out position one because of discontinuity
  v=cumsum(a)*dt;
  s=cumsum(v)*dt;
else
  H=ones(1,length(t))*3500     ;     % 6.7hp to lb*ft/s
  v=sqrt(2*H/m) .* sqrt(t);
  a=sqrt(H/(2*m)) .* sqrt(1./t);
  s=(2/3)*sqrt(2*H/m).*t.^(3/2);
end

wf=v/rt;
wfdot=a/rt;
ws=wf*Rfd;
wsdot=wfdot*Rfd;
we=ones(1,length(ws))*314.2;
R=we./ws;
Rdot=-(wsdot.*we.*ws.^(-2));

Rt_corr=max( min(R,3.9), 0.9);
Rm = zeros(size(Rt_corr));
Rm(1)=Rt_corr(1);

for i = 2:length(Rm)
  Rm(i)=Rm(i-1)+max(min((Rt_corr(i)-Rm(i-1)),Rdot_max*dt),-Rdot_max*dt);
end


wem = ws.*Rm;
wem_RPM= wem*60/(2*pi);
we_RPM=we*60/(2*pi);

if (showplots)

figure
hold on
plot(t,R)
plot(t,Rm)
%legend("R", "R_m")
title("R_t and R_m")
ylim([0 8]);
xlabel("Time (s)")
ylabel("rev/rev (ratio)")
line([0 8],[3.9 3.9], "linestyle", "--", "color", "k")
line([0 8],[0.9 0.9], "linestyle", "--", "color", "k")
grid minor on
hold off

figure
hold on
plot(t,we_RPM);
plot(t,wem_RPM);
%legend("eRPM Target", "eRPM actual")
title("Engine RPM, ideal vs real")
xlabel("Time (s)")
ylabel("RPM")
line([0 8], [3800 3800], "linestyle", "-", "color", "r")
grid minor on

hold off

power=interp1(Edata_rpm, Edata_power_real, wem_RPM, "linear", "extrap");
figure
plot(t, power)
title("Engine Power")
xlabel("Time (s)")
ylabel("Power (hp), assuming eff_P=67%")
grid minor on

figure
plot(t,s);
line([0 8], [100 100], "linestyle", "--", "color", "k");
title("Position vs Time");
end
acceltime = 0;
for i = [1:length(s)]
  if s(i) > 100
    acceltime = t(i)
    break
  endif
end

if exist("done")
  done += 1;
  timelist = [timelist acceltime]
else
  done = 1;
  timelist = [acceltime]
end

