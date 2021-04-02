function [P, gamma, flag] = compute_terminal_conditions(Q,R,ts)

  fprintf('Computing terminal cost and terminal set...');
  
  import casadi.*
  
  % Define states and controls
  xLabels = {'q1','q2','q1d','q2d','tau'};
  uLabels = {'u1','u2','v'};

  nx = numel(xLabels);
  nu = numel(uLabels);
  for k = 1:nx
      xI.(char(xLabels{k})) = k;
  end
  for k = 1:nu
      uI.(char(uLabels{k})) = k;
  end

  % States and controls
  x  = SX.sym('x',[numel(xLabels),1]);
  u = SX.sym('u',[numel(uLabels),1]);

  % q - \dot{q}
  q1 = x(xI.q1); q2 = x(xI.q2); x1 = [q1; q2];
  q1d = x(xI.q1d); q2d = x(xI.q2d); x2 = [q1d; q2d];
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
  C = Function('C',{x1,x2},{-c1*sin(q2)*[q1d q1d+q2d;
                                        -q1d 0]});
  grav = Function('g',{x1},{[g1*cos(q1) + g2*cos(q1+q2);
                          g2*cos(q1+q2)]});
                      
  % Upper bounds                          
  x01 = [0;0];
  Bb = norm(full(B(x01)),2);
  x01 = [0;pi/2]; x02 = [-3*pi/2;-3*pi/2];
  Cb = norm(full(C(x01,x02)),2);
  gb = norm(full(grav([0 0])),2);
  
  % From our defined timing law
  theta = load(gen_path(...
    {'data','robotic_arm','robotic_arm_reference.mat'}));

  % Path bounds
  dotp = theta.dp;
  ddotp = theta.ddp;

  % State and control bounds
  ub = 4000;
  dotqb = 3*pi/2;
  
  % Define dynamics
  Ac = [0 0 1 0; 
        0 0 0 1;
        0 0 0 0;
        0 0 0 0];
  Bc = [0 0; 0 0; 1 0; 0 1];
  sysc = ss(Ac,Bc,[],[]);
  sysd = c2d(sysc,ts,'zoh');
  
  % Cost tuning
  QN = diag([1 1 1 1]);
  RN = diag([10 10]);

  % Compute costs
  Kc = lqrd(sysc.A,sysc.B,QN,RN,ts);
  Ad = sysd.A; Bd = sysd.B;

  % Terminal cost
  P = dlyap((Ad-Bd*Kc).',Q+Kc'*R*Kc);
  
  
  % Compute terminal set by solving LMI (40)
  gamma = sdpvar(1,1);
  lambda1 = sdpvar(1,1);
  lambda2 = sdpvar(1,1);

  d1 = (ub - Cb*dotqb-gb-Bb*ddotp)/(Bb*norm(Kc,2));
  d2 = (dotqb-dotp);

  constr = [];
  constr = [constr; ...
                    blkdiag(P,-gamma) - ...
                    lambda1*blkdiag(eye(4),-d1) >= 0 ];
  constr = [constr; lambda1 >=0 ];
  constr = [constr; ...
                    blkdiag(P,-gamma) - ...
                    lambda2*blkdiag(blkdiag(zeros(2),eye(2)),-d2) >= 0];
  constr = [constr; lambda2 >=0];

  % Solve LMI
  flag = 0;
  sol = optimize(constr,-gamma,sdpsettings('solver','sdpt3','verbose',0));
  if ~contains(sol.info,'Successfully solved')
    flag = 1;
    gamma = 0;
    fprintf('failed to solve LMI\n');
    return
  end
  
  gamma = value(gamma);
  fprintf('done\n');
end
