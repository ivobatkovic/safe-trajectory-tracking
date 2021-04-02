function vehicle_mpfc(xInit,save_file,sim_time)

  import casadi.*

  % Horizon and shooting intervals
  N = 20;
  T = 1;
  ts = T/N;

  % Closed loop iterations
  kIter = sim_time/ts;

  % Cost tuning and terminal cost
  Q = 8*diag([1e4 1e5 1e5 1/16]);
  R = diag([10 10 1]);
  Qn = 1740/2 * diag([0 0 0 1]);

  % Define states and controls
  xLabels = {'x','y','psi','theta'};
  uLabels = {'u1','u2','v'};

  % Indexing functions for states
  nx = numel(xLabels);
  nu = numel(uLabels);
  for k = 1:nx
      xI.(char(xLabels{k})) = k;
  end
  for k = 1:nu
      uI.(char(uLabels{k})) = k;
  end
  xuI.x = xI;
  xuI.u = uI;

  % States and controls, and auxiliary state
  x  = SX.sym('x',[numel(xLabels),1]);
  u = SX.sym('u',[numel(uLabels),1]);

  % Load vehicle reference
  reference_file = gen_path({'data','vehicle','vehicle_reference.mat'});
  if ~exist(reference_file,'file')
    generate_vehicle_reference(reference_file);
  end
  load(reference_file);

                 
  % Define the right hand side
  lm = -1e-3;
  rhs = [ u(uI.u1)*cos(x(xI.psi));
          u(uI.u1)*sin(x(xI.psi));
          u(uI.u1)*tan(u(uI.u2));
          -lm*x(xI.theta)+u(uI.v);
          ];

  % Define integrator
  n_int_steps = 10;
  rk4  = casadi_rk4(x,u,rhs,ts,n_int_steps,'rk4');

  % Create OCP variables
  V  = MX.sym('V',  N*nu + (N+1)*nx );
  Vr = MX.sym('Vr', N*nu + (N+1)*nx );

  % Indexing of variables
  [iV, iif] = state_indexing(xLabels,uLabels,N,xuI);

  % Define cost and constraints
  cost = 0;
  g = [];
  iG_.dyn = {}; iG_.xf = {};
  offset = 0;

  % Loop over cost for 1 : N
  for k = 1:N
  
    % Integrator constraint
    [X_,] = rk4(V(iV('x',k)),V(iV('u',k)));
    g = [g; V(iV('x',k+1)) - X_];
    iG_.dyn{k} = (1:nx).' + offset;
    offset = offset + nx;
     
    % States at time k
    xk = V(iV('x',k));
    thetak = xk(end);
    
    % Control inputs at time k
    uk = V(iV('u',k));
    vk = uk(end);
    
    % Reference at time k
    dthetak = -lm*thetak+vk;
    xkr = [p1(thetak);
           p2(thetak);
           atan2(dp2(thetak),dp1(thetak));
           0];
    ukr = [dthetak * sqrt(dp1(thetak)^2+dp2(thetak)^2);
           atan2(ddp2(thetak)*(1+dp2(thetak)^2)^(-3/2),1);
           0];
    
    dx = xk-xkr;
    du = uk-ukr;
    cost = cost + dx'*Q*dx + du'*R*du;
   
  end
  % Terminal states
  xN = V(iV('x',N+1)); thN = xN(end);

  % Terminal reference
  xrN = [p1(thN); p2(thN); atan2(dp2(thN),dp1(thN));0];

  % Terminal cost
  cost = cost + (xN-xrN)'*Qn*(xN-xrN);

  % Enforce terminal point
  g = [g; xN-xrN];
  iG_.xf{1} = (1:3)' + offset;
  offset = 3 + offset;

  % Set up nlp
  nlp = struct( 'x', V, 'p', Vr, 'f', cost, 'g', g);
  opts = struct('ipopt',struct());
  opts.ipopt.print_level = 0;
  opts.ipopt.suppress_all_output = 'yes';
  opts.print_time = 0;
  solver = nlpsol( 'solver', 'ipopt', nlp, opts );

  % Set up constraints indexing
  iG =  constraint_indexing(iG_,iif);

  % Set  up constraints
  ng = size(g);
  lbv = -inf*ones(size(V));
  ubv =  inf*ones(size(V));
  lbg = -inf*ones(ng);
  ubg =  inf*ones(ng);

  % System dynamics constraints
  lbg(iG('dyn')) = zeros( size(iG('dyn')) );
  ubg(iG('dyn')) = zeros( size(iG('dyn')) );

  % Terminal point constraint
  lbg(iG('xf')) = zeros( size(iG('xf')) );
  ubg(iG('xf')) = zeros( size(iG('xf')) );

  % State and control constraints
  lbv(iV('u',1:N,'u1')) = 0*ones(size(iV('u',1:N,'u1')));
  ubv(iV('u',1:N,'u1')) =  6*ones(size(iV('u',1:N,'u1')));

  lbv(iV('u',1:N,'u2')) = -.63*ones(size(iV('u',1:N,'u2')));
  ubv(iV('u',1:N,'u2')) = .63*ones(size(iV('u',1:N,'u2')));

  lbv(iV('u',1:N,'v')) = 0*ones(size(iV('u',1:N,'v')));
  ubv(iV('u',1:N,'v')) =  6*ones(size(iV('u',1:N,'v')));

  % Initialization for solver
  Vref = zeros(size(V)); 
  x0 = zeros(size(V));

  % Project position to find corresponding tau
  fprintf('Projecting theta on the path...');
  [theta_0, flag] = path_projection(xInit,xInit(end),p1,p2);
  if flag
    fprintf('failed\n');
    return;
  end
  xInit(end) = theta_0;
  fprintf('done\n');
  
  % Apply initial state
  x0(iV('x')) = repmat(xInit,N+1,1);
  x_0 = xInit;

  % Logging variables
  X = x_0; U = [];
  X_open = []; U_open = [];

  % Closed loop simulation
  fprintf('Processing: '); percent = [];
  t_elapsed = 0;

  time_array = zeros(1,kIter);
  for k = 1 : kIter
    tic;

    % Update  initial state
    lbv(iV('x',1)) = x_0;
    ubv(iV('x',1)) = x_0;

    % Solve problem
    sol = solver('x0',x0,'lbx',lbv,'ubx',ubv,'lbg',lbg,'ubg',ubg,'p',Vref);
    if isequal(solver.stats.return_status,'Infeasible_Problem_Detected')
      disp('Infeasible');
      break;
    end
    v_opt = full(sol.x); 
    x0 = v_opt;

    % Update system with control input
    u_0 = full(v_opt(iV('u',1)));
    x_0 = full(rk4(x_0,u_0));

    % Fill logging vector
    X = [X x_0];
    U = [U u_0];

    X_open = [X_open; v_opt(iV('x',1:N+1,'x'))';
                      v_opt(iV('x',1:N+1,'y'))';
                      v_opt(iV('x',1:N+1,'psi'))'
                      v_opt(iV('x',1:N+1,'theta'))'];
    U_open = [U_open; v_opt(iV('u',1:N,'u1'))';
                      v_opt(iV('u',1:N,'u2'))';
                      v_opt(iV('u',1:N,'v'))';];

    % Timing and printing
    t1 = toc;
    time_array(k) = t1;
    t_elapsed = t_elapsed + t1;
    eta = (kIter-k) * t1;
    fprintf(repmat('\b',1,length(percent)-1));
    percent = ...
      sprintf('%.2f%%%% - Est. time left: %.2fs - Time elapsed: %.2fs',...
      (100*k/kIter),eta,t_elapsed);
    fprintf(percent); drawnow;
  end
  fprintf('\n');

  % Process data into desired format
  ref=[]; refu = [];
  % Open loop
  if kIter == 1
    t = v_opt(iV('x',1:N+1,'theta'))';
    for k = 1 : size(t,2)
      ref = [ref [full(p1(t(k)));
                  full(p2(t(k)));
                  full(atan2(dp2(t(k)),1))]];
    end

    X = [v_opt(iV('x',1:N+1,'x')) v_opt(iV('x',1:N+1,'y')) ...
         v_opt(iV('x',1:N+1,'psi')) v_opt(iV('x',1:N+1,'theta'))]';
    U = [v_opt(iV('u',1:N,'u1')) v_opt(iV('u',1:N,'u2')) ...
         v_opt(iV('u',1:N,'v'))]';

    % Timing for state and controls in open loop
    tx = 0:ts:N*ts;
    tu = 0:ts:N*ts-ts;

    % Closed loop
    else
      t = X(end,:);
      for k = 1 : size(t,2)
        ref = [ref [full(p1(t(k)));
                   full(p2(t(k)));
                   full(atan2(dp2(t(k)),1))]];
      end

      % Timing for state and controls in closed loop
      tx = 0:ts:kIter*ts;
      tu = 0:ts:kIter*ts-ts;

  end
    
  save(save_file,...
    'X','U','ref','refu','tx','tu','Q','R','xInit','time_array');
end