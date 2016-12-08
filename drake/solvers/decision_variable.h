#pragma once

#include <cstddef>
#include <list>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/number_traits.h"

namespace drake {
namespace solvers {
/**
 * This class stores the type, name, value, and index of a
 * decision variable in an optimization program.
 * The DecisionVariableScalar created by MathematicalProgram should not outlive
 * its creator MathematicalProgram object.
 */
class DecisionVariableScalar {
 public:
  enum class VarType { CONTINUOUS, INTEGER, BINARY };

  /**
   * Constructs a decision variable.
   *
   * IF YOU ARE CALLING THIS CONSTRUCTOR DIRECTLY, YOU ARE PROBABLY DOING
   * SOMETHING WRONG.
   *
   * The intended usage of this function is via MathematicalProgram, e.g.:
   * @code{.cc}
   * // Creates an optimization program object with no decision variables.
   * MathematicalProgram prog;
   *
   * // Add a 2 x 1 vector containing two decision variables to the optimization
   * // program.
   * // This calls the private constructor
   * // DecisionVariableScalar(VarType type, const std::string &name, double*
   * // value, size_t index)
   * DecisionVariableVector<2> x1 = prog.AddContinuousVariables<2>();
   *
   * // Add a 2 x 1 vector containing two decision variables to the optimization
   * // program.
   * // This calls the private constructor
   * // DecisionVariableScalar(VarType type, const std::string &name, double*
   * // value, size_t index)
   * DecisionVariableVector<2> x2 = prog.AddContinuousVariables<2>();
   *
   * // This calls the default constructor DecisionVariableScalar(),
   * // X is not related to the optimization program prog yet.
   * DecisionVariableMatrix<2, 2> X;
   *
   * // Now X contains the decision variables from the optimization program
   * // object prog.
   * // The first column of X is x1, the second column of X is x2.
   * X << x1, x2;
   * @endcode
   *
   * @param type Supports CONTINUOUS, INTEGER or BINARY.
   * @param name The name of the variable.
   * @param index The index of the variable in the optimization program.
   */
  DecisionVariableScalar(VarType type, const std::string& name, double* value,
                         size_t index)
      : type_(type), name_(name), value_(value), index_(index) {}

  /**
   * This constructor creates a dummy placeholder, the value_ pointer
   * is initialized to nullptr. 
   */
  DecisionVariableScalar()
      : type_(VarType::CONTINUOUS), name_(""), value_(nullptr), index_(0) {}

  void set_value(double new_value) { *value_ = new_value; }

  /**
   * @return The type of the variable.
   */
  VarType type() const { return type_; }

  /**
   * @return The name of the variable.
   */
  std::string name() const { return name_; }

  /**
   * @return The value of the variable. This method is only meaningful after
   * calling Solve() in MathematicalProgram.
   */
  double value() const {
    // TODO(hongkai.dai): check if Solve() has been called.
    return *value_;
  }

  /**
   * @return The index of the variable in the optimization program.
   */
  size_t index() const { return index_; }

  /**
   * Determines if the two DecisionVariableScalar objects are the same. This
   * comparison is only meaningful if the two DecisionVariableScalar objects
   * are created by the same MathematicalProgram object.
   */
  bool operator==(const DecisionVariableScalar& rhs) const;

  friend class MathematicalProgram;

 private:
  VarType type_;
  std::string name_;
  double* value_;
  size_t index_;
};

/**
 * Prints out the DecisionVariableScalar's name.
 * @relates DecisionVariableScalar.
 */
std::ostream& operator<<(std::ostream& os, const DecisionVariableScalar& var);

struct DecisionVariableScalarHash {
  size_t operator()(const DecisionVariableScalar& var) const;
};
}  // namespace solvers
}  // namespace drake

namespace Eigen {

/// Eigen scalar type traits for Matrix<DecisionVariableScalar>.
template <>
struct NumTraits<drake::solvers::DecisionVariableScalar> {
  static inline int digits10() { return 0; }
  enum {
    IsInteger = 0,
    IsSigned = 1,
    IsComplex = 0,
    RequireInitialization = 1,
    ReadCost = 1,
    AddCost = 1,
    MulCost = 1
  };

  template <bool Vectorized>
  struct Div {
    enum { Cost = 1 };
  };

  typedef drake::solvers::DecisionVariableScalar Real;
  typedef drake::solvers::DecisionVariableScalar Nested;
  typedef drake::solvers::DecisionVariableScalar Literal;
};
}  // namespace Eigen

namespace drake {
template <>
struct is_numeric<solvers::DecisionVariableScalar> {
  static constexpr bool value = false;
};
}  // namespace drake

namespace drake {
namespace solvers {
template <int rows, int cols>
using DecisionVariableMatrix =
    Eigen::Matrix<drake::solvers::DecisionVariableScalar, rows, cols>;
template <int rows>
using DecisionVariableVector = DecisionVariableMatrix<rows, 1>;
using DecisionVariableMatrixX =
    DecisionVariableMatrix<Eigen::Dynamic, Eigen::Dynamic>;
using DecisionVariableVectorX = DecisionVariableVector<Eigen::Dynamic>;

using VariableListRef = std::list<Eigen::Ref<const DecisionVariableMatrixX>>;

/**
 * This class stores a list of DecisionVariableMatrix objects. An instance
 * of this class is going to be bound to a constraint, indicating that a
 * constraint is imposed on one or several DecisionVariableMatrix objects.
 */
class VariableList {
 public:
  explicit VariableList(const VariableListRef& variable_list);

  /**
   * Returns all the stored DecisionVariableMatrix.
   */
  const std::list<DecisionVariableMatrixX>& variables() const {
    return variables_;
  }

  /**
   * Given a list of DecisionVariableMatrix @p vars, computes the TOTAL number
   * of unique scalar decision variables stored in @p vars.
   * Example
   * @code{.cc}
   * // Create a mathematical program with no decision variables.
   * MathematicalProgram prog;
   *
   * // Add a vector containing 4 decision variables.
   * auto x = prog.AddContinuousVariables<4>();
   *
   * // x1 contains x(0), x(1), x(2).
   * DecisionVariableVector<3> x1 = x.head<3>();
   *
   * // x2 contains x(2), x(3).
   * DecisionVariableVector<2> x2 = x.tail<2>();
   *
   * // Construct a VariableList containing both x1 and x2.
   * VariableList var_list({x1, x2});
   *
   * std::cout << "The number of unique variables is " <<
   *     var_list.num_unique_variables() << std::endl;
   *
   * std::cout << "The size of variable list (including duplication) is " <<
   *     var_list.size() << std::endl;
   * @endcode
   *
   * The output is
   * <pre>
   * The number of unique variables is 4
   * The size of variable list (including duplication) is 5
   * </pre>
   */
  size_t num_unique_variables() const {
    return unique_variable_indices_.size();
  }

  /**
   * Given a list of DecisionVariableMatrix @p vars, computes the TOTAL number
   * of scalar decision variables stored in @p vars, including duplication. The
   * duplicated variables will be counted for more than once.
   * @see num_unique_variables() for an example.
   */
  size_t size() const { return size_; }

  /**
   * Determines if the stored DecisionVariableMatrix objects are
   * all column vectors.
   */
  bool column_vectors_only() const { return column_vectors_only_; }

  /**
   * @return The all unique variables stored in the class.
   */
  const std::unordered_set<DecisionVariableScalar, DecisionVariableScalarHash>&
  unique_variables() const {
    return unique_variable_indices_;
  }

 private:
  std::list<DecisionVariableMatrixX> variables_;
  size_t size_;
  bool column_vectors_only_;
  std::unordered_set<DecisionVariableScalar, DecisionVariableScalarHash>
      unique_variable_indices_;
};

/**
 * A binding on constraint type C is a mapping of the decision
 * variables onto the inputs of C.  This allows the constraint to operate
 * on a vector made up of different elements of the decision variables.
 */
template <typename _Binding>
class DecisionVariableBinding {
 public:
  DecisionVariableBinding(const _Binding& b, const VariableList& v)
      : constraint_(b), variable_list_(v) {}

  DecisionVariableBinding(const _Binding& b, const VariableListRef& v)
      : constraint_(b), variable_list_(v) {}
  template <typename U>
  DecisionVariableBinding(
      const DecisionVariableBinding<U>& b,
      typename std::enable_if<std::is_convertible<U, _Binding>::value>::type* =
          nullptr)
      : DecisionVariableBinding(b.get(), b.variable_list()) {}

  const _Binding& get() const { return constraint_; }

  const VariableList& variable_list() const { return variable_list_; }

  /**
   * Get an Eigen vector containing all variable values. This only works if
   * every element in variable_list_ is a column vector.
   * @return A Eigen::VectorXd for all the variables in the variable vector.
   */
  Eigen::VectorXd VariableListToVectorXd() const {
    size_t dim = 0;
    Eigen::VectorXd X(GetNumElements());
    for (const auto& var : variable_list_.variables()) {
      DRAKE_ASSERT(var.cols() == 1);
      X.segment(dim, var.rows()) = GetSolution(var);
      dim += var.rows();
    }
    return X;
  }

  /**
   * Returns true iff the given @p index of the enclosing
   * MathematicalProgram is included in this Binding.*/
  bool ContainsVariableIndex(size_t index) const {
    for (const auto& view : variable_list_.variables()) {
      if (DecisionVariableMatrixContainsIndex(view, index)) {
        return true;
      }
    }
    return false;
  }

  size_t GetNumElements() const {
    // TODO(ggould-tri) assumes that no index appears more than once in the
    // view, which is nowhere asserted (but seems assumed elsewhere).
    return variable_list_.size();
  }

  /**
   * Writes the elements of @p solution to the bound elements of
   * the @p output vector.
   */
  void WriteThrough(const Eigen::VectorXd& solution,
                    Eigen::VectorXd* output) const {
    DRAKE_ASSERT(static_cast<size_t>(solution.rows()) == GetNumElements());
    size_t solution_index = 0;
    for (const auto& var : variable_list_.variables()) {
      DRAKE_ASSERT(var.cols() == 1);
      const auto& solution_segment =
          solution.segment(solution_index, var.rows());
      output->segment(var(0).index(), var.rows()) = solution_segment;
      solution_index += var.rows();
    }
  }

 private:
  _Binding constraint_;
  VariableList variable_list_;
};

template <typename C>
using DecisionVariableConstraintBinding =
    DecisionVariableBinding<std::shared_ptr<C>>;

/**
 * Given a DecisionVariableMatrix object, returns the Eigen::Matrix that
 * stores the values of each decision variable.
 * @tparam Derived A DecisionVariableMatrix class.
 * @param decision_variable_matrix A DecisionVariableMatrix object.
 */
template <typename Derived>
Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
GetSolution(const Eigen::MatrixBase<Derived>& decision_variable_matrix) {
  static_assert(
      std::is_same<typename Derived::Scalar, DecisionVariableScalar>::value,
      "The input should be a DecisionVariableMatrix object");
  Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
      double_matrix(decision_variable_matrix.rows(),
                    decision_variable_matrix.cols());
  for (int i = 0; i < decision_variable_matrix.rows(); ++i) {
    for (int j = 0; j < decision_variable_matrix.cols(); ++j) {
      double_matrix(i, j) = decision_variable_matrix(i, j).value();
    }
  }
  return double_matrix;
}

/**
 * Determines if a DecisionVariableMatrix object contains a variable with
 * given @p index.
 */
template <typename Derived>
bool DecisionVariableMatrixContainsIndex(
    const Eigen::MatrixBase<Derived>& decision_variable_matrix, size_t index) {
  for (int i = 0; i < decision_variable_matrix.rows(); ++i) {
    for (int j = 0; j < decision_variable_matrix.cols(); ++j) {
      if (decision_variable_matrix(i, j).index() == index) {
        return true;
      }
    }
  }
  return false;
}
}  // end namespace solvers
}  // end namespace drake
