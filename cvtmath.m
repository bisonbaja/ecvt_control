clear
H=3685     ;     % 6.7hp to lb*ft/s
m=600/32.2 ;     % 600 lb to slugs
Rfd=6.418  ;     % final drive ratio
rt=10/12   ;     % inches
dt=0.01;
t=[0:dt:8];
Rdot_max = 2;

Edata_rpm =   [3800 3600 3400 3200 3000 2800 2600 2400] % RPM
Edata_torque= [12.7 13.5 14.5 15.4 16.6 17.4 18.1 18.5] % ft*lbf
Edata_power=  [9.2 9.3 9.4 9.4 9.5 9.3 9.0 8.5]        % hp
Edata_power_real = Edata_power*0.67;

v=sqrt(2*H/m) * sqrt(t);
a=sqrt(H/(2*m)) * sqrt(1./t);
s=(2/3)*sqrt(2*H/m)*t.^(3/2);

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

figure 1

set(gca, "fontsize", 16)
plot(Rm,Rt_corr);
title('Correlation of R_M with R_T')
xlabel('R_M (real)')
ylabel('R_T')
grid minor on


figure 2
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



figure 3
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

power=interp1(Edata_rpm, Edata_power_real, wem_RPM);
figure 4
plot(t, power)
title("Engine Power")
xlabel("Time (s)")
ylabel("Power (hp), assuming eff_P=67%")
grid minor on

