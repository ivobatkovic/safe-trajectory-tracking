function generate_robotic_arm_reference(save_file)
%%
  import casadi.*
  fprintf('Generating robotic arm reference...');

  %  Casadi variables: theta, time
  z = MX.sym('z',1,1);
  tk = MX.sym('tk',1,1);
  
  % Define path from [17] T. Faulwasser, and R. Findeisen, "Nonlinear 
  % Model Predictive Control for Constrained Output Path Following"
  p1 = Function('p1',{z},{z-pi/3});
  dp1 = Function('dp1',{z},{jacobian(p1(z),z)});
  ddp1 = Function('ddp1',{z},{jacobian(dp1(z),z)});
  p2 = Function('p2',{z},{5*sin(0.6*(z-pi/3))});
  dp2 = Function('dp2',{z},{jacobian(p2(z),z)});
  ddp2 = Function('ddp2',{z},{jacobian(dp2(z),z)});

  %  Numerically integrate the path from theta = [-5.3, 0]
  dp1_ = @(x) 1;
  dp2_ = @(x) 3*cos(0.6*(x-pi/3));
  df = @(x) sqrt(dp1_(x).^2+dp2_(x).^2);
  q = integral(df,-5.3,0);

  % Constant velocity for 5 sec, then brake remaining distance
  vRef = 1;
  t0 = 5;
  s0 = vRef*t0;
  a = -vRef^2/(2*(q-s0));
  t1 = -vRef/a;
  
  % Resulting velocity profile
  v = [vRef vRef 0 0];
  t = [0 t0 t0+t1 t0+t1+10];
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
  theta_k = -5.3;
  theta  = theta_k;
  dt = full(vRef / norm([dp1(theta_k);dp2(theta_k)]));
  dtheta = dt;
  
  % Generate trajectory
  kIter = t(end)/ts;
  for kj = 1:kIter
    tt = (kj-1)*ts;
    theta_k = min(full(rk4(theta_k,tt)),0);
    theta = [theta theta_k];
    tt = kj*ts;
    dtheta = [dtheta full(vR(tt) / norm([dp1(theta_k);dp2(theta_k)]))];
  end
  
  T_traj = 0:ts:kIter*ts-ts;
  theta_ = theta(1:end-1);
  dtheta_ = dtheta(1:end-1);
  ddtheta_ = diff(dtheta)/ts;
  
  n_sample = 10; % every 10th sample
  theta = casadi.interpolant('theta','bspline',...
            {T_traj(1:n_sample:end)},theta_(1:n_sample:end));
  dtheta = casadi.interpolant('dtheta','bspline',...
            {T_traj(1:n_sample:end)},dtheta_(1:n_sample:end));
  ddtheta = casadi.interpolant('dtheta','bspline',...
            {T_traj(1:n_sample:end)},ddtheta_(1:n_sample:end));
  
  ddp = [];
  for kj = 1 : n_sample : kIter-1
    d2p = full(norm([ddp1(theta_(kj));ddp2(theta_(kj))]*dtheta_(kj)^2+...
                    [dp1(theta_(kj));dp2(theta_(kj))]*ddtheta_(kj),2));
    ddp =[ddp d2p];
  end
  ddp = max(abs(ddp))+0.1;
  dp = vRef;
  
  save(save_file,'p1','dp1','ddp1','p2','dp2','ddp2',...
                 'theta','dtheta','ddtheta','dp','ddp');
  fprintf('done\n');