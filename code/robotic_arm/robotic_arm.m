function robotic_arm(xInit, save_file, sim_time)

  import casadi.*
 
  % Fix the seed for reproducibility
  rng(2021);

  % Horizon and shooting intervals
  M = 50;
  N = M/2;
  T = 1.5;
  ts = T/M;
  
  % Closed loop iterations
  kIter = round(sim_time/ts);

  % Cost tuning
  Q = diag([1e5 1e5 10 10]);
  R = diag([1e-3 1e-3 10]);
  
  % Load robotic manipulator reference
  reference_file = gen_path(...
      {'data','robotic_arm','robotic_arm_reference.mat'});
  if ~exist(reference_file,'file')
    generate_robotic_arm_reference(reference_file);
  end
  load(reference_file);

  % Compute terminal cost and set
  [P_cost, gamma, flag] = compute_terminal_conditions(Q,R(1:2,1:2),ts);
  if flag
    fprintf('exiting...\n');
    return;
  end
  
  % Define states and controls
  xLabels = {'q1','q2','q1d','q2d','tau'};
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

  % States and controls
  x  = SX.sym('x',[numel(xLabels),1]);
  u = SX.sym('u',[numel(uLabels),1]);

  % q - \dot{q}
  q1 = x(xI.q1); q2 = x(xI.q2);
  x1 = [q1; q2];
  q1d = x(xI.q1d); q2d = x(xI.q2d);
  x2 = [q1d; q2d];

  % tau
  tau = x(xI.tau);

  % u1 - u2 - v
  u1 = u(uI.u1); u2 = u(uI.u2);
  v = u(uI.v);

  % Dynamics parameters 
  b1=200;   b2=50;  
  b3=23.5;  b4=25;  
  b5=122.5; c1=-25;
  g1=784.8; g2=245.3; 

  % Functions
  B = Function('B',{x1},{[b1+b2*cos(q2) b3+b4*cos(q2);
                          b3+b4*cos(q2) b5]});
  invB = Function('B',{x1},...
              {1/((b1+b2*cos(q2))*b5-(b3+b4*cos(q2))*(b3+b4*cos(q2)))*...
               [b5 -b3-b4*cos(q2); -b3-b4*cos(q2) b1+b2*cos(q2)]});                             

  C = Function('C',{x1,x2},{-c1*sin(q2)*[q1d q1d+q2d;
                                        -q1d 0]});
  grav = Function('g',{x1},{[g1*cos(q1) + g2*cos(q1+q2);
                             g2*cos(q1+q2)]});
                           
  % Define the right hand side
  rhs = [ x2;
          invB(x1) * ( [u1;u2]-C(x1,x2)*x2-grav(x1) );
          1 + v;
          ];

  % Define RK4 integrator
  n_int_steps = 10;
  rk4 = casadi_rk4(x,u,rhs,ts,n_int_steps,'rk4');

  % Create OCP variables
  V  = MX.sym('V',  M*nu + (M+1)*nx );
  Vr = MX.sym('Vr', M*nu + (M+1)*nx );

  % Indexing of variables
  [iV, iif] = state_indexing(xLabels,uLabels,M,xuI);

  % Define cost and constraints
  cost = 0;
  g = [];
  iG_.dyn = {};
  iG_.xf = {};
  iG_.stop = {};
  iG_.obs = {};
  offset = 0;

  % Loop over cost for 1 : M
  for k = 1:M

      % Integrator constraint
      [X_,] = rk4(V(iV('x',k)),V(iV('u',k)));
      g = [g; V(iV('x',k+1)) - X_];
      iG_.dyn{k} = (1:nx).' + offset;
      offset = offset + nx;

      % States at time k
      x1_k = [V(iV('x',k,'q1')); V(iV('x',k,'q2'))];
      x2_k = [V(iV('x',k,'q1d')); V(iV('x',k,'q2d'))];
      tau_k = V(iV('x',k,'tau'));
      
      % Controls at time
      u_k = V(iV('u',k));

      % Reference at time k
      th   = theta(tau_k); dth = dtheta(tau_k); ddth = ddtheta(tau_k);
      x1r  = [p1(th);p2(th)]; 
      x2r  = [dp1(th);dp2(th)]*dth;
      dx2r = [ddp1(th);ddp2(th)]*dth^2 + [dp1(th);dp2(th)]*ddth;
      ur = [B(x1_k)*dx2r + C(x1_k,x2_k)*x2_k + grav(x1_k); 0];
      
      % Uncertainty constraint - collision avoidance
      obstacle = Vr(iV('x',k));
      obs_x = obstacle(1:2);
      obs_radius = obstacle(3);
      g = [g; obs_radius - (x1_k(1:2)-obs_x)'*(x1_k(1:2)-obs_x)];
      iG_.obs{k} = 1 + offset;
      offset = offset + 1;
      
      % Cost function
      if k <= N 
        dx = [x1_k-x1r; x2_k-x2r];
        du = u_k-ur;
        cost = cost + dx' * Q * dx + du' * R * du;
      end
  end

  % Terminal states
  xN = V(iV('x',N+1)); tauN = xN(end); 
  
  % Terminal reference
  th = theta(tauN); dth = dtheta(tauN);
  xrN = [p1(th);p2(th); dp1(th)*dth;dp2(th)*dth];
  
  % Terminal cost
  dx = xN(1:4)-xrN;
  cost = cost + dx'*P_cost*dx;

  % Terminal set constraint (20g) in the paper
  for k = N+1:M+1
    xk = V(iV('x',k)); tauk = xk(end);
    th = theta(tauk); dth = dtheta(tauk);
    xrk = [p1(th);p2(th); dp1(th)*dth;dp2(th)*dth];
    dx = xk(1:4)-xrk;
    g = [g; dx'*P_cost*dx];
    iG_.xf{end+1} = 1 + offset;
    offset = 1 + offset;
  end

  % Terminal set constraint (20h) in the paper
  xM = V(iV('x',M+1)); tauM = xM(end);
  g = [g; xM(3:4)];
  iG_.stop{1} = (1:2)' + offset;
  offset = 2 + offset;

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

  % Dynamic constraints
  lbg(iG('dyn')) = zeros( size(iG('dyn')) );
  ubg(iG('dyn')) = zeros( size(iG('dyn')) );

  % Obstacle constraint
  ubg(iG('obs')) = 0*ones( size(iG('obs')) );

  % Terminal set
  ubg(iG('xf')) = gamma*ones( size(iG('xf')) );

  % Stopping constraint
  lbg(iG('stop')) = -zeros( size(iG('stop')) );
  ubg(iG('stop')) = zeros( size(iG('stop')) );

  % State and control constraints
  lbv(iV('x',1:M,'q1d')) = -3*pi/2*ones(size(iV('x',1:M,'q1d')));
  ubv(iV('x',1:M,'q1d')) =  3*pi/2*ones(size(iV('x',1:M,'q1d')));

  lbv(iV('x',1:M,'q2d')) = -3*pi/2*ones(size(iV('x',1:M,'q2d')));
  ubv(iV('x',1:M,'q2d')) =  3*pi/2*ones(size(iV('x',1:M,'q2d')));

  lbv(iV('x',1:M,'tau')) = 0*ones(size(iV('x',1:M,'tau')));
  ubv(iV('x',1:M,'tau')) =  inf*ones(size(iV('x',1:M,'tau')));

  lbv(iV('u',1:M,'u1')) = -4000*ones(size(iV('u',1:M,'u1')));
  ubv(iV('u',1:M,'u1')) =  4000*ones(size(iV('u',1:M,'u1')));

  lbv(iV('u',1:M,'u2')) = -4000*ones(size(iV('u',1:M,'u2')));
  ubv(iV('u',1:M,'u2')) =  4000*ones(size(iV('u',1:M,'u2')));

  lbv(iV('u',1:M,'v')) = -50*ones(size(iV('u',1:M,'v')));
  ubv(iV('u',1:M,'v')) =  50*ones(size(iV('u',1:M,'v')));

  % Initialization for solver
  Vref = zeros(size(V)); 
  x0 = zeros(size(V));

  % Project position to find corresponding tau
  fprintf('Projecting tau on the trajectory...');
  [tau_0, flag] = trajectory_projection(xInit,theta,p1,p2);
  if flag
    fprintf('failed\n');
  end
  xInit(end) = tau_0;
  fprintf('done\n');
  
  x_0 = xInit;
  x0(iV('x',1:N+1)) = repmat(x_0,N+1,1);

  % Apply initial prediction
  xobs = [-6;-2]; heading = pi/4; velocity = .3; r_0 = 0.03;
  Vref(iV('x',1:M+1)) = ...
    constant_velocity_prediction(xobs,r_0,velocity,heading,ts,M);

  % Logging variables
  X = x_0; U = []; X_obs = xobs;
  X_open = []; U_open = []; X_obs_open = [];

  % Closed loop simulation
  fprintf('Processing: '); percent = [];
  t_elapsed = 0;

  time_array = zeros(1,round(kIter));
  for k = 1 : kIter
    tic;

    % Update  initial state
    lbv(iV('x',1)) = x_0;
    ubv(iV('x',1)) = x_0;

    % Pass obstacle predictions 
    Vref(iV('x',1:M+1)) = ...
      constant_velocity_prediction(xobs,r_0,velocity,heading,ts,M);
    
    % Solve problem
    sol = solver('x0',x0,'lbx',lbv,'ubx',ubv,'lbg',lbg,'ubg',ubg,'p',Vref);
    if isequal(solver.stats.return_status,'Infeasible_Problem_Detected')
      disp('Infeasible');
      break;
    end
    v_opt = full(sol.x); 

    x0 = v_opt;
    x0(iV('x',1:N)) = v_opt(iV('x',2:N+1));
    x0(iV('u',1:N-1)) = v_opt(iV('u',2:N));

    % Update system with control input
    u_0 = full(v_opt(iV('u',1)));
    x_0 = full(rk4(x_0,u_0));

    % Update obstacle state
    xobs = [Vref(iV('x',2,'q1'));...
            Vref(iV('x',2,'q2'))]+[r_0/2*(2*rand-1);r_0/2*(2*rand-1)];
    
    % Fill logging vector
    X = [X x_0];
    U = [U u_0];
    X_obs = [X_obs xobs];

    X_open = [X_open; v_opt(iV('x',1:M+1,'q1'))';
                      v_opt(iV('x',1:M+1,'q2'))';
                      v_opt(iV('x',1:M+1,'q1d'))'
                      v_opt(iV('x',1:M+1,'q2d'))'
                      v_opt(iV('x',1:M+1,'tau'))'];
    U_open = [U_open; v_opt(iV('u',1:M,'u1'))';
                      v_opt(iV('u',1:M,'u2'))';
                      v_opt(iV('u',1:M,'v'))';];
    X_obs_open = [X_obs_open; Vref(iV('x',1:M+1,'q1'))';
                              Vref(iV('x',1:M+1,'q2'))';
                              Vref(iV('x',1:M+1,'q1d'))'];

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
  %%
  % Process data into desired format
  tx = 0; tu = 0;
  ref=[]; refu = [];
  % Open loop
  if kIter == 1
    t = v_opt(iV('x',1:M+1,'tau'))';
    for k = 1 : size(t,2)
      x1_k = full([v_opt(iV('x',k,'q1')); v_opt(iV('x',k,'q2'))]);
      x2_k = full([v_opt(iV('x',k,'q1d')); v_opt(iV('x',k,'q2d'))]);
      th = theta(t(k)); dth = dtheta(t(k)); ddth = ddtheta(t(k));
      ref = [ref full([p1(th);p2(th);dp1(th)*dth;dp2(th)*dth])];

      dx2r = [ddp1(th);ddp2(th)]*dth^2 + [dp1(th);dp2(th)]*ddth;
      ur = full([B(x1_k)*dx2r + C(x1_k,x2_k)*x2_k + grav(x1_k); 0]);
      refu = [refu ur];
    end
    refu(:,end) = [];

    X = [v_opt(iV('x',1:M+1,'q1')) v_opt(iV('x',1:M+1,'q2')) ...
         v_opt(iV('x',1:M+1,'q1d')) v_opt(iV('x',1:M+1,'q2d')) ...
         v_opt(iV('x',1:M+1,'tau'))]';
    U = [v_opt(iV('u',1:M,'u1')) v_opt(iV('u',1:M,'u2')) ...
         v_opt(iV('u',1:M,'v'))]';
    X_obs = [Vref(iV('x',1:M+1,'q1')) ...
             Vref(iV('x',1:M+1,'q2')) Vref(iV('x',1:M+1,'q1d'))]';

    % Timing for state and controls in open loop
    tx = 0:ts:M*ts;
    tu = 0:ts:M*ts-ts;

  % Closed loop
  else
    t = X(5,:);
    for k = 1 : size(t,2)
      x1_k = X(1:2,k); x2_k = X(3:4,k);
      th = theta(t(k)); dth = dtheta(t(k)); ddth = ddtheta(t(k));
      ref = [ref full([p1(th);p2(th);dp1(th)*dth;dp2(th)*dth])];

      dx2r = [ddp1(th);ddp2(th)]*dth^2 + [dp1(th);dp2(th)]*ddth;
      ur = full([B(x1_k)*dx2r + C(x1_k,x2_k)*x2_k + grav(x1_k); 0]);
      refu = [refu ur];
    end
    refu(:,end) = [];
    % Timing for state and controls in closed loop
    tx = 0:ts:kIter*ts;
    tu = 0:ts:kIter*ts-ts;

  end
  
  save(save_file, 'X', 'U', 'ref', 'refu', 'tx', 'tu', 'X_obs', ...
                  'X_open','U_open','X_obs_open','time_array');
