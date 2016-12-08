#pragma once 

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace systems {

enum {
  kThrowException = 1,
  kWarnOnce = 2
  // Set output high?  ...
  } RequirementFailureAction;

class RequirementsMonitor : public LeafSystem<double> {
public:
  RequirementsMonitor() {
    // make single decision variable for time
  }

  // Get the decision variable associated with simulation clock time.
  solvers::DecisionVariableVectorX time();
  
  // Create a new input port for the system.  Returns a vector of decision variables.
  DecisionVariableVectorX AddVectorInput(size);
  
  // Add a new requirement to the system.
  void AddConstraint(std::shared_ptr<Constraint> con,
                     const VariableListRef& vars, RequirementFailureAction action);

  std::unique_ptr<RequirementsMonitor> Build();
  
  DoPublish(const Context<double>& context) {
    time() = context.get_time();
    
    // Reads all of the inputs.
    for (const auto& input : input_ports_) {
      input.WriteThrough(this->EvalVectorInput(context,input_.get())->CopyToVector(),variables_)
    }
    
    // Evaluates all of the constraints.
  }

private:
  DecisionVariableVectorX variables_;
  std::vector<DecisionVariableConstraintBinding<Constraint>> constraints_;
  std::vector<DecisionVariableBinding<SystemPortDescriptor<double>> input_ports;
  
  // TODO: add a list of system ports (and their bindings to decision variables)
}

}  // namespace systems
}  // namespace drake

