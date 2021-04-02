function [tau, flag] = trajectory_projection(xInit,theta,p1,p2)
  
  import casadi.*
  
  [theta_, flag] = path_projection(xInit,theta(xInit(end)),p1,p2);
  if flag
    tau = 0; return
  end
 
  opti = casadi.Opti();
  x = opti.variable();
  f = (theta_-theta(x))^2;
  
  opti.minimize(f);
  opti.subject_to(x >= 0);
  opti.subject_to(x <= 25);
  opts.ipopt.print_level = 0;
  opts.ipopt.suppress_all_output = 'yes';
  opts.print_time = 0;

  opti.solver('ipopt',opts);
  sol = opti.solve();
  
  tau = sol.value(x);
  flag = 1;
  if opti.return_status == 'Solve_Succeeded'
    flag = 0;
  end
  
end