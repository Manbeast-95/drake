classdef RigidBodyElement
  % handles lazy binding of urdf parameters to rigid body element values
  
  properties
    param_bindings=struct();   % structure containing msspoly parameterized representations of some properties
  end
  
  methods
    function body=bindParams(body,model,pval)
      fr = getParamFrame(model);
      pn = properties(body);
      for i=1:length(pn)
        if isa(body.(pn{i}),'msspoly')
          body.param_bindings.(pn{i}) = body.(pn{i});
          body.(pn{i}) = double(subs(body.(pn{i}),fr.poly,pval));
        end
      end
    end
    
    function body=updateParams(body,poly,pval)
      % this is only intended to be called from the parent manipulator
      % class. (maybe I should move it up to there?)
      fn = fieldnames(body.param_bindings);
      for i=1:length(fn)
        body.(fn{i}) = double(subs(body.param_bindings.(fn{i}),poly,pval));
      end
    end
  end
  
  methods    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      % intentionally do nothing. overload if necessary
    end
    
    function obj = updateForRemovedLink(obj,model,body_ind)
      % intentionally do nothing. overload if necessary
    end
    
    function obj = updateBodyCoordinates(obj,body_ind,T_old_body_to_new_body)
      % intentionally do nothing. overload if necessary
    end
  end
end