function generate_vehicle_reference(save_file)
  
  import casadi.*
  fprintf('Generating vehicle reference...');

  %  Casadi variables: theta, time
  z = MX.sym('z',1,1);
  tk = MX.sym('tk',1,1);

  % Define path from [14] T. Faulwasser, B. Kern, and R. Findeisen,
  % "Model Predictive Path-Following for Constrained Nonlinear Systems"
  a = 6; beta = 5; gm = 20; wm = 0.35;
  p1 = Function('p1',{z},{z});
  dp1 = Function('dp1',{z},{jacobian(p1(z),z)});
  ddp1 = Function('ddp1',{z},{jacobian(dp1(z),z)});
  p2 = Function('p2',{z},{-a*log(gm/(beta-z))*sin(wm*z)});
  dp2 = Function('dp2',{z},{jacobian(p2(z),z)});
  ddp2 = Function('ddp2',{z},{jacobian(dp2(z),z)});

  %  Numerically integrate the path from theta = [-30, 0]
  df = @(x) (-a*wm.*cos(wm*x).*log(gm./(beta-x))-a.*sin(wm*x)./(beta-x));
  fun = @(x) sqrt(1+ df(x).^2);
  q = integral(fun,-30,0);

  % Constant velocity for 7 sec, then brake remaining distance
  vRef = 5;
  s0 = vRef*7;
  s1 = (q-s0);
  a = -vRef^2/(2*s1);
  t0 = -vRef/a;
  
  % Resulting velocity profile
  v = [5 5 0 0];
  t = [0 7 7+t0 10];
  vR = interpolant('vR','linear',{t},v);

  % Define a RK4 integrator for \dot theta 
  f = casadi.Function('f',{z,tk},{vR(tk) / norm([dp1(z);dp2(z)]),1});
  ts = 0.001; n_int_steps = 40;
  dt = ts/n_int_steps;
  [k1,k1t] = f(z,tk);
  [k2,k2t] = f(z+0.5*dt*k1,tk);
  [k3,k3t] = f(z+0.5*dt*k2,tk);
  [k4,k4t] = f(z+dt*k3,tk);
  rk4_step = casadi.Function('rk4_step',{z,tk},...
                            {z + (1./6)*dt*(k1 + 2*k2 + 2*k3 + k4),...
                            tk + (1./6)*dt*(k1t + 2*k2t + 2*k3t + k4t)});
  x_ = z; t_ = tk;
  for kj = 1:n_int_steps
    [x_,t_] = rk4_step(x_,t_);
  end
  rk4 = casadi.Function('rk4',{z,tk},{x_});

  % Initial conditions
  theta_k = -30;
  theta  = theta_k;
  dt = full(vRef / norm([dp1(theta_k);dp2(theta_k)]));
  dtheta = dt;

  % Generate a trajectory for 10 sec
  kIter = 10/ts;
  for kj = 1:kIter
    tt = (kj-1)*ts;
    theta_k = min(full(rk4(theta_k,tt)),0);%th + ts*dt;
    theta = [theta theta_k];
    tt = kj*ts;
    dtheta = [dtheta full(vR(tt) / norm([dp1(theta_k);dp2(theta_k)]))];
  end

  T_traj = 0:ts:kIter*ts;
  theta_ = theta;
  dtheta_ = dtheta;

  n_sample = 10; % every 10th sample
  dtheta = casadi.interpolant('dtheta','bspline',...
                        {T_traj(1:n_sample:end)},dtheta_(1:n_sample:end));
  theta = casadi.interpolant('theta','bspline',...
                        {T_traj(1:n_sample:end)},theta_(1:n_sample:end));

  save(save_file,'dtheta','theta','p1','dp1','ddp1','p2','dp2','ddp2');

  fprintf('done\n');

end
