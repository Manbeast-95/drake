
#include <typeinfo>
#include "drake/solvers/Optimization.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

void trivialLeastSquares() {
  OptimizationProblem prog;

  auto const &x = prog.addContinuousVariables(4);

  auto x2 = x(2);
  auto xhead = x.head(3);

  Vector4d b = Vector4d::Random();
  auto con = prog.addLinearEqualityConstraint(Matrix4d::Identity(), b, {x});
  prog.solve();
  valuecheckMatrix(b, x.value(), 1e-10);
  valuecheck(b(2), x2.value()(0), 1e-10);
  valuecheckMatrix(b.head(3), xhead.value(), 1e-10);
  valuecheck(b(2), xhead(2).value()(0), 1e-10); // a segment of a segment

  auto const &y = prog.addContinuousVariables(2);
  prog.addLinearEqualityConstraint(2 * Matrix2d::Identity(), b.topRows(2), {y});
  prog.solve();
  valuecheckMatrix(b.topRows(2) / 2, y.value(), 1e-10);
  valuecheckMatrix(b, x.value(), 1e-10);

  con->updateConstraint(3 * Matrix4d::Identity(), b);
  prog.solve();
  valuecheckMatrix(b.topRows(2) / 2, y.value(), 1e-10);
  valuecheckMatrix(b / 3, x.value(), 1e-10);

  std::shared_ptr<BoundingBoxConstraint> bbcon(
          new BoundingBoxConstraint({x.head(2)}, MatrixXd::Constant(2, 1, -1000.0), MatrixXd::Constant(2, 1, 1000.0)));
  prog.addConstraint(bbcon);
  prog.solve();  // now it will solve as a nonlinear program
  valuecheckMatrix(b.topRows(2) / 2, y.value(), 1e-10);
  valuecheckMatrix(b / 3, x.value(), 1e-10);
}


class SixHumpCamelObjective : public TemplatedDifferentiableFunction<SixHumpCamelObjective> {
public:
  SixHumpCamelObjective() : TemplatedDifferentiableFunction<SixHumpCamelObjective>(*this) {};

  template<typename ScalarType>
  void evalImpl(const Ref<const Matrix<ScalarType, Dynamic, 1>>& x, Matrix<ScalarType,Dynamic,1>& y) {
    y.resize(1);
    y(0) = x(0) * x(0) * (4 - 2.1 * x(0) * x(0) + x(0) * x(0) * x(0) * x(0) / 3) + x(0) * x(1) +
           x(1) * x(1) * (-4 + 4 * x(1) * x(1));
  }
};

void sixHumpCamel() {
  OptimizationProblem prog;
  auto x = prog.addContinuousVariables(2);
  std::shared_ptr<FunctionConstraint> objective(new FunctionConstraint({x},make_shared<SixHumpCamelObjective>(),1));
  prog.addObjective(objective);
  prog.solve();
  prog.printSolution();

  // check (numerically) if it is a local minima
  VectorXd ystar, y;
  objective->eval(x.value(),ystar);
  for (int i=0; i<10; i++) {
    objective->eval(x.value() + .01 * Eigen::Matrix<double, 2, 1>::Random(), y);
    if (y(0)<ystar(0)) throw std::runtime_error("not a local minima!");
  }
}

int main(int argc, char* argv[])
{
  trivialLeastSquares();
  sixHumpCamel();
  return 0;
}