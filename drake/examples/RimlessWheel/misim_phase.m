function misim_phase

options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain();
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('RimlessWheel.urdf',.01,options);
warning(w);

theta0 = 0.1;
figure(1); clf;  hold on;
for thetadot0 = 0:0.075:1
  x0 = [sin(theta0);cos(theta0);theta0;thetadot0;0;thetadot0];
%  xtraj = misim(getManipulator(p),x0,.02,300);
%  h=fnplt(xtraj,[3,6]);  set(h,'LineStyle','-','MarkerSize',10)
  xtrajlin = milinsim(getManipulator(p),[sin(pi/8);cos(pi/8);pi/8],x0,.02,300);
%  [u,xtrajlin] = milinmpc(getManipulator(p),[sin(pi/8);cos(pi/8);pi/8],x0,.02,20,eye(getNumStates(p)),[]);
  h=fnplt(xtrajlin,[3,6]); set(h,'LineStyle','-','MarkerSize',10,'Color','r');
  drawnow; 
end

axis([theta0 pi/4-theta0 -1 1.5])
xlabel('$\theta$','interpreter','latex')
ylabel('$\dot\theta$','interpreter','latex')

%v = p.constructVisualizer();
%v.axis = [-2.5 2.5 -.1 3];
%v.playback(xtraj);
