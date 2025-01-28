if !exist("m")
  m=600/32.2 ;     % 600 lb to slugs
endif

if !exist("Rfd")
  Rfd=6.418  ;     % final drive ratio
endif
rt=10/12   ;     % inches
dt=0.002;
tmax = 5;
t=[0:dt:tmax];
if !exist("showplots")
  showplots = 0;
endif

servo_tdata = [0 833.24/16];
servo_sdata = [1/(0.14*6) 0];
ls_torque = 30.85; % in-lbs

servo_rpm_max = interp1(servo_tdata, servo_sdata, ls_torque, "linear", "extrap")
ls_lead = 1;
Pdot_max = servo_rpm_max*ls_lead; % in/s
Rdot_max = 4*Pdot_max
we_high_rpm = 3800;
we_high = we_high_rpm*2*pi/60;
Edata_rpm =   [3800 3600 3400 3200 3000 2800 2600 2400] ;% RPM
Edata_torque= [12.7 13.5 14.5 15.4 16.6 17.4 18.1 18.5] ;% ft*lbf
Edata_power=  [9.2 9.3 9.4 9.4 9.5 9.3 9.0 8.5]    ;    % hp
Edata_power_real = Edata_power*0.67;
Edata_power_real_lbfts = Edata_power_real*550; % Corrected power in lbf*ft/s
Edata_rads = Edata_rpm*2*pi/60; % Sampled RPM values but in rad/s
Edata_torque_real = Edata_power_real_lbfts./Edata_rads; % Corrected torque in lbf*ft

a=[0];
v=[1];
s=[0];
wem=[2800*2*pi/60];
if !exist("wet")
  wet=3000*2*pi/60;
endif

Rm=[3.9];
Rt=[3.9];
wf=[0];
fail=0;

for i = [2:length(t)]
  H(i)=interp1(Edata_rads, Edata_power_real_lbfts, wem(i-1), "linear", "extrap");
  a(i)=H(i)/(m*v(i-1));

##  if (t(i)>5 && t(i)<6)
##    a(i)=-32.2*0.8;
##  endif

  v(i)=v(i-1)+a(i)*dt;
  s(i)=s(i-1)+v(i)*dt;

  wf(i)=v(i)/rt;
  ws(i)=wf(i)*Rfd;
  Rt(i)=max(min((wet/ws(i)),3.9), 0.9);
  Rm(i)=Rm(i-1)+max(min((Rt(i)-Rm(i-1)),Rdot_max*dt),-Rdot_max*dt);
  wem(i)=ws(i)*Rm(i);
  if (wem(i)>we_high)
    wem(i)=we_high;
    ws(i)=wem(i)/Rm(i);
    wf(i)=ws(i)/Rfd;
    v(i)=wf(i)*rt;
    a(i)=(v(i)-v(i-1))/dt;
    H(i)=m*a(i)*v(i);
    fail(i)=1;
  else
    fail(i)=0;
  endif
end

for i = [1:length(s)]
  if s(i) > 100
    acceltime = t(i)
    break
  endif
end

close all
if showplots
  figure
  hold on
  plot(t,wem*60/(2*pi));
  title("Engine RPM, ideal vs real")
  xlabel("Time (s)")
  ylabel("RPM")
  line([0 8], [3800 3800], "linestyle", "-", "color", "r")
  line([0 8], [wet*60/(2*pi) wet*60/(2*pi)], "linestyle", "-", "color", "k")
  grid minor on
  hold off

  figure
  plot(t, a)
  ylim([0 40])
  title("Acceleration")

  figure
  plot(t, H/550)
  title("Engine Power")
  xlabel("Time (s)")
  ylabel("Power delivered (hp)")
  grid minor on
endif
