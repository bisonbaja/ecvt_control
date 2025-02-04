clear;
global L = 36; % belt length
global c = 10; % center-to-center of pulleys
belt_angle = 30;
R = [0.9:0.1:3.9];

function r_0 = r_sol(Rvar, c, L)
  [r_0, fval, info] = fsolve( @(rvar) (pi.*rvar.*(Rvar+1) + 2.*sqrt(c.^2 + ( (rvar.^2).*( (Rvar-1).^2) ) ) - L), [3]);
endfunction
r = zeros(1, length(R));
for i = 1:length(R)
  r(i) = r_sol(R(i), c, L);
end

r_eng = 0.5;
P=2*(r - r_eng)*tan( (belt_angle/2) * pi/180);
plot(R, P);
