function rk4 = casadi_rk4(x,u,rhs,ts,n_int_steps,name)

  f = casadi.Function('f',{x,u},{rhs});
  
  % Define an integrator
  dt = ts/n_int_steps;
  [k1] = f(x,u);
  [k2] = f(x+0.5*dt*k1,u);
  [k3] = f(x+0.5*dt*k2,u);
  [k4] = f(x+dt*k3,u);
  rk4_step = casadi.Function('rk4_step',{x,u},...
                                {x + (1./6)*dt*(k1 + 2*k2 + 2*k3 + k4)});
  
  x_ = x;
  for k = 1:n_int_steps
    x_ = rk4_step(x_,u);
  end
  rk4 = casadi.Function(name,{x,u},{x_});
    
end