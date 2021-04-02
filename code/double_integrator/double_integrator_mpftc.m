function double_integrator_mpftc(xInit, save_file, sim_time)

  import casadi.*
  
  % Horizon and shooting intervals
  N = 100;
  T = 2;
  ts = T/N;

  % Closed loop iterations
  kIter = sim_time/ts;

  % Cost tuning
  Q = diag([10 10]);
  R = diag([1 1]);

  % Compute terminal cost
  QN = eye(2); RN = 10*R(1);
  Ac = [0 1; 0 0]; Bc = [0; 1];
  sysc = ss(Ac,Bc,[],[]);
  sysd = c2d(sysc,ts,'zoh');
  K = lqrd(sysc.A,sysc.B,QN,RN,ts);
  P = dlyap((sysd.A-sysd.B*K).',Q+K'*R(1)*K);

  % Define states and controls
  xLabels = {'p','v','tau'};
  uLabels = {'a','tdot','s'};

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

  % States and controls
  x = SX.sym('x',[numel(xLabels),1]);
  u = SX.sym('u',[numel(uLabels),1]);
  t = SX.sym('t',1,1);
                         
  % Define the right hand side
  rhs = [ x(xI.v);
          u(uI.a);
          1+u(uI.tdot);
          ];

  % Constant trajectory
  vRef = 4;
  r_t = Function('r_t',{t},{[vRef*t;vRef]});

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
  iG_.dyn = {}; iG_.xf = {}; iG_.stop = {};
  offset = 0;

  % Loop over cost for 1 : M
  for k = 1:N
  
    % Integrator constraint
    [X_,] = rk4(V(iV('x',k)),V(iV('u',k)));
    g = [g; V(iV('x',k+1)) - X_];
    iG_.dyn{k} = (1:nx).' + offset;
    offset = offset + nx;
    
    % Obstacle constraint
    g = [g; V(iV('x',k+1,'p'))-Vr(iV('x',k+1,'p'))-V(iV('u',k,'s'))];
    iG_.obs{k} = 1 + offset;
    offset = offset + 1;
     
    % States at time k
    xk = V(iV('x',k));
    tauk = V(iV('x',k,'tau'));
    
    % Control inputs at time k
    uk = V(iV('u',k));
    
    % Reference at time k
    xkr = r_t(tauk);
    
    % Cost
    dx = xk(1:2) - xkr;
    du = uk(1:2);
    cost = cost + dx'*Q*dx + du'*R*du +10000*uk(3);
  end

  % Terminal state and cost
  xN = V(iV('x',N+1));

  % Terminal reference
  xrN = r_t(V(iV('x',N+1,'tau')));

  % Terminal cost
  cost = cost + (xN(1:2)-xrN)'*P*(xN(1:2)-xrN);
  
  % Terminal set
  g = [g; -K*(xN(1:2)-r_t(V(iV('x',N+1,'tau'))))];
  iG_.xf{1} = 1 + offset;
  offset = 1 + offset;

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

  % Fix dynamics
  lbg(iG('dyn')) = zeros( size(iG('dyn')) );
  ubg(iG('dyn')) = zeros( size(iG('dyn')) );

  % Terminal set
  a_min = -1; a_max = 5;
  lbg(iG('xf')) = a_min*ones(size(iG('xf')) );
  ubg(iG('xf')) =  a_max*ones(size(iG('xf')) );

  % Obstacle
  ubg(iG('obs')) = zeros( size(iG('obs')) );

  % State and control constraints
  lbv(iV('x',1:N,'v')) = zeros(size(iV('x',1:N,'v')));

  lbv(iV('u',1:N,'a')) = a_min*ones(size(iV('u',1:N,'a')));
  ubv(iV('u',1:N,'a')) = a_max*ones(size(iV('u',1:N,'a')));

  lbv(iV('u',1:N,'s')) =  zeros(size(iV('u',1:N,'s')));

  % Initialization for solver
  Vref = zeros(size(V)); 
  x0 = zeros(size(V));

  % Apply initial state
  x0(iV('x')) = repmat(xInit,N+1,1);
  x_0 = xInit;

  % Logging variables
  X = x_0; XP_open = []; XV_open = []; UV_open = [];
  U = [];
  
  % Obstacle position and when it disappears
  x_obs = 20 ; t0 = 15;

  % Closed loop simulation
  fprintf('Processing:\n'); percent = [];
  t_elapsed = 0;
  time_array = zeros(1,kIter);
  for k = 1 : kIter
    tic;

    if k*ts <= t0
      Vref(iV('x',1:N+1,'p')) = x_obs;
    else
      Vref(iV('x',1:N+1,'p')) = x_obs+100;
    end
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

    % Get control input and integrate the closed loop system
    u_0 = full(v_opt(iV('u',1)));
    x_0 = full(rk4(x_0,u_0));

    % Fill logging vector
    X = [X x_0];
    XP_open = [XP_open; v_opt(iV('x',1:N+1,'p'))'];
    XV_open = [XV_open; v_opt(iV('x',1:N+1,'v'))'];
    UV_open = [UV_open; v_opt(iV('u',1:N,'a'))'];
    U = [U u_0];

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
  ref=[];
  % Open loop
  if kIter == 1
    t = v_opt(iV('x',1:N+1,'tau'))';
    for k = 1 : size(t,2)
      ref = [ref full(r_t(t(k)))];
    end

    X = [v_opt(iV('x',1:N+1,'p')) ...
         v_opt(iV('x',1:N+1,'v')) ...
         v_opt(iV('x',1:N+1,'tau'))]';
    U = [v_opt(iV('u',1:N,'a'))]';

    % Timing for state and controls in open loop
    tx = 0:ts:N*ts;
    tu = 0:ts:N*ts-ts;
  % Closed loop
  else
    t = X(3,:);
    for k = 1 : size(t,2)
      ref = [ref full(r_t(t(k)))];
    end
    
    
    % Timing for state and controls in closed loop
    tx = 0:ts:kIter*ts;
    tu = 0:ts:kIter*ts-ts;

  end
  save(save_file,'X','U','ref','tx','tu','Q','R','time_array');
end