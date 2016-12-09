#pragma once

#include "drake/solvers/constraint.h"
#include "drake/solvers/decision_variable.h"

namespace drake {
namespace systems {

// TODO(russt):
// Support a handful of different results of success/failure on a constraint:
//  - throw an exception (default)?
//  - warn once
//  - set a flag and end the simulation (note: useful on success or failure)
//  - add outputs (e.g., one per constraint), which latch to one?

class RequirementsMonitor : public LeafSystem<double> {
 public:
  RequirementsMonitor() {
    // make single decision variable for time
    time_.resize(1);
    values_.resize(1);
    time_(0) = DecisionVariableScalar(
        DecisionVariableScalar::VarType::CONTINUOUS, "time", values_.data(), 0);
  }

  // Get the decision variable associated with simulation clock time.
  solvers::DecisionVariableVectorX time();

  // Creates a new input port for the system.  Returns a vector of decision
  // variables.
  solvers::DecisionVariableVectorX AddVectorInput(
      int size, const std::string& name = "") {
    int num_values = values.size();
    values_.resize(values_.size() + size);

    std::string port_name = name;
    if (port_name.empty()) port_name = "u" + std::toString(input_.size());

    DecisionVariableVectorX in(size);
    for (int i = 0; i < size; i++) {
      in(i) =
          DecisionVariableScalar(DecisionVariableScalar::VarType::CONTINUOUS,
                                 port_name + "[" + std::toString(i) + "]",
                                 &(values_[num_values + i]), 0);
    }
    input_.push_back(in);
  }

  // Add a new requirement to the system.
  void AddConstraint(std::shared_ptr<Constraint> con,
                     const VariableListRef& vars) {
    constraint_size_
  }

  DoPublish(const Context<double>& context) {
    // Set the input time.
    time_[0] = context.get_time();

    // Read all of the inputs.
    for (int i = 0; i < get_num_input_ports(); i++) {
      const auto& input_data = this->EvalVectorInput(context, i);
      // TODO(russt): Wrap this in a setter method for DecisionVariableVectors.
      for (int j = 0; j < input_[i].size(); j++) {
        input_[j] = input_data_[j];
      }
    }

    // Evaluate all of the constraints.
    Eigen::VectorXd constraint_value;

    // TODO(russt): pre-allocate constraint_values_ to the largest input port
    // size, and pass in .head() to the eval method.  Constraints will need to
    // be updated to support Eigen::Ref for the second argument.  Probably all
    // solvers should do this (and will benefit from it.  (Need to confirm that
    // Eval can still call resize, or mandate that they cannot).

    for (const auto& binding : constraints_) {
      solvers::Constraint* constraint = binding.get().get();
      constraint->Eval(binding.VariableListToVectorXd(), constraint_value);
      if ((constraint_value.array() < constraint->lower_bound.array()).any() ||
          (constraint_value.array() > constraint->upper_bound.array()).any()) {
        // TODO(russt): Display as much useful information as possible here
        // (constraint name, number, description, ..)
        DRAKE_ABORT("Constraint Violated: Terminating simulation");
      }
    }
  }

 private:
  std::vector<double> values_;
  DecisionVariableVectorX
      time_;  // TODO(russt): could this be DecisionVariableVector<1>?
  std::vector<DecisionVariableVectorX> input_;
  std::vector<DecisionVariableConstraintBinding<Constraint>> constraints_;
}

}  // namespace systems
}  // namespace drake
