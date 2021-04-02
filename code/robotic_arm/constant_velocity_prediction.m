function x = constant_velocity_prediction(x_0,r_0,velocity,heading,ts,N)

  % Initial state - pad with zeros to fit the dimensions for IPOPT
  x = [x_0;r_0;0;0];

  % Linearly increase the radius uncertainty
  dr = r_0;

  % Iteration variables
  x_iter = x_0;
  r_iter = r_0;
  for k = 1 : N
    x_iter = x_iter + ts*velocity*[cos(heading); sin(heading)];
    r_iter = r_iter + dr;
    x = [x; [x_iter;r_iter;0;0]];
  end
end



