#pragma once 

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/constraint.h"


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
  DecisionVariable time();
  
  // Create a new input port for the system.  Returns a vector of decision variables.
  DecisionVariableVector AddVectorInput(size);
  
  // Add a new requirement to the system.
  void AddConstraint(std::shared_ptr<Constraint> con,
                     const VariableListRef& vars, RequirementFailureAction action);

  std::unique_ptr<RequirementsMonitor> Build();

private:
  DecisionVariableVectorX variables_;
  std::vector<Binding<Constraint>> constraints_;  // Note: currently Binding is a private class INSIDE MathematicalProgram.  Can we extract it?
  
  // TODO: add a list of system ports (and their bindings to decision variables)
}

