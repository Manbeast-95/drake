function [J,PI,xbins]=runValueIteration

plant = PendulumPlant;
options.dt = 1e-2;
options.gamma = .999;
options.wrap_flag = [true;false];
%cost = @mintime; ulimit = 1;
cost = @lqrcost; ulimit = 10; % = 5 converges faster (but pumps less)
xbins = {linspace(0,2*pi,101),linspace(-18,18,101)};
ubins = linspace(-ulimit,ulimit,51);
mdp = MarkovDecisionProcess.discretizeSystem(plant,cost,xbins,ubins,options);

function drawfun(J,PI)
  fig = sfigure(2); 
  % figure size is hard-coded simply so that it is aesthetically pleasing.
  % parameters for position set through trial-and-error.
  set(fig, 'units', 'normalized', 'position', [.4 .1 .2 .75]);
  clf;
  n1=length(xbins{1});
  n2=length(xbins{2});
  subplot(2,1,1);mesh(xbins{1},xbins{2},reshape(ubins(PI),n1,n2)');
  axis xy;  axis tight;
  xlabel('theta');
  ylabel('thetadot');
  title('u(x)');
  subplot(2,1,2);mesh(xbins{1},xbins{2},reshape(J,n1,n2)');
  axis xy;  axis tight;
  xlabel('theta');
  ylabel('thetadot');
  title('J(x)');
  drawnow;
end

[J,PI] = valueIteration(mdp,0.001,@drawfun);

sys = feedback(plant,PI);
v = PendulumVisualizer();
for i=1:5
  xtraj = simulate(sys,[0,10],0.2*randn(2,1));
  v.playback(xtraj);
end

end

function g = lqrcost(sys,x,u)
  xd = [pi;0];
  g = (x-xd)'*(x-xd) + 10*u^2;
end

function g = mintime(sys,x,u)
  xd = [pi;0];
  if (x-xd)'*(x-xd) < .05;
    g = 0;
  else
    g = 1;
  end
end
