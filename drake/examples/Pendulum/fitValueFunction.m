%function fitValueFunction

load pendulum_value_iteration.mat;

num_x = 2;
Sgrid = cell(1,num_x);
[Sgrid{:}] = ndgrid(xbins{:});
% Sgrid{j}(i_1,i_2,...,i_{num_x}) = xbins{j}(i_j)
S = cellfun(@(a)reshape(a,1,[]),Sgrid,'UniformOutput',false);
S = vertcat(S{:});
clear Sgrid;

basis_generators = { @(x)sin(x(1,:)), @(x)cos(x(1,:)), @(x)x(2,:) };
degrees = { 0:3, 0:3, 0:2 };
degree_grid = cell(1,length(degrees));
[degree_grid{:}] = ndgrid(degrees{:});
degree = cellfun(@(a)reshape(a,1,[]),degree_grid,'UniformOutput',false);
degree = vertcat(degree{:});
clear degree_grid;

phi = ones(size(degree,2),size(S,2));
for i=1:size(degree,2)
  for j=1:length(basis_generators)
    phi(i,:) = phi(i,:).*(basis_generators{j}(S).^degree(j,i));
  end
end

V1_indices = (J<30)';
V2_indices = ~V1_indices;
Jhat = J;

%% EM-style algorithm to fit two Lyapunov functions

while (1)
  % least squares fit to data J(V1_indices) ~= alpha1'*phi(:,V1_indices)
  alpha1 = (phi(:,V1_indices)')\J(V1_indices);
  alpha2 = (phi(:,V2_indices)')\J(V2_indices);
  
  V1 = alpha1'*phi(:,V1_indices);
  V2 = alpha2'*phi(:,V2_indices);
  
  clf; hold on;
  plot3(S(1,V1_indices),S(2,V1_indices),J(V1_indices),'r.');
  plot3(S(1,V2_indices),S(2,V2_indices),J(V2_indices),'b.');
  
  Jhat(V1_indices) = V1;
  Jhat(V2_indices) = V2;
  mesh(xbins{1},xbins{2},reshape(Jhat,length(xbins{1}),length(xbins{2}))')
%  plot3(S(1,V1_indices),S(2,V1_indices),V1,'g.');
%  plot3(S(1,V2_indices),S(2,V2_indices),V2,'m.');
  axis tight; drawnow;
  
%  V1_indices_next = (J'-alpha1'*phi).^2 < (J'-alpha2'*phi).^2;
  V1_indices_next = alpha1'*phi < alpha2'*phi;
  
  if all(V1_indices_next == V1_indices)
    break;
  end
  V1_indices = V1_indices_next;
  V2_indices = ~V1_indices;
end


