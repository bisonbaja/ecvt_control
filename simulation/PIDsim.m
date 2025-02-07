clear;
close all;
delta_t = 0.0001;
t_final = 0.5;
Ka = 5.5;
Kp = -1;
Ki = 0;
Kd = 0;
t_vals = [0:delta_t:t_final];
R_target = 1;

function r_0 = r_sol(Rvar, c, L)
  [r_0, fval, info] = fsolve( @(rvar) (pi.*rvar.*(Rvar+1) + 2.*sqrt(c.^2 + ( (rvar.^2).*( (Rvar-1).^2) ) ) - L), [2]);
endfunction

function R_0 = R_sol(rvar, c, L)
  [R_0, fval, info] = fsolve( @(Rvar) (pi.*rvar.*(Rvar+1) + 2.*sqrt(c.^2 + ( (rvar.^2).*( (Rvar-1).^2) ) ) - L), [3]);
endfunction

Rfunc = @(r) R_sol(r, 10, 36);
rfunc = @(R) r_sol(R, 10, 36);

r_vals = zeros(1,length(t_vals));
r_vals(1) = 1.125;
R_vals = zeros(1,length(t_vals));
R_vals(1) = Rfunc(r_vals(1));
e_vals = zeros(1,length(t_vals));
e_vals(1) = R_target - R_vals(1);
de_vals = zeros(1, length(t_vals));
de_vals(1) = 0;
Ie_vals = zeros(1, length(t_vals));
Ie_vals(1) = 0;

for (i = [2:length(t_vals)])
  r_vals(i) = r_vals(i-1) + e_vals(i-1).*Kp + de_vals(i-1).*Kd + Ie_vals(i-1).*Ki;
  R_vals(i) = Rfunc(r_vals(i));
  e_vals(i) = R_target - R_vals(i);
  de_vals(i) = ( e_vals(i) - e_vals(i-1) )./delta_t;
  Ie_vals(i) = Ie_vals(i-1) + e_vals(i).*delta_t;
end

figure
hold on;
plot(t_vals, R_vals);
line([0 t_final], [R_target R_target], "linestyle", "-", "color", "r");
grid minor;
hold off;

figure
hold on;
plot(t_vals, r_vals);
line([0 t_final], [rfunc(R_target) rfunc(R_target)], "linestyle", "-", "color", "r");
grid minor;
hold off;


