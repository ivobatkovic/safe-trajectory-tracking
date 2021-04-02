function [theta, flag] = path_projection(xInit, theta_0, p1, p2)

  import casadi.*
  
  opti = casadi.Opti();
  x = opti.variable();
  
  dx = xInit(1:2) - [p1(x);p2(x)];
  f = dx'*dx;
  
  opti.minimize(f);
  opti.subject_to(x <= 0);
  opti.subject_to(x >= -30);
  
  opts.ipopt.print_level = 0;
  opts.ipopt.suppress_all_output = 'yes';
  opts.print_time = 0;
  
  opti.solver('ipopt',opts);
  opti.set_initial(x,theta_0);
  sol = opti.solve();
  
  theta = sol.value(x);
  flag = 1;
  if opti.return_status == 'Solve_Succeeded'
    flag = 0;
  end

end