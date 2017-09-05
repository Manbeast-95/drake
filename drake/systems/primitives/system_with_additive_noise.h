#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// Wraps any system with an additional vector input to represent a random
/// "process noise" that will be added to the state dynamics.  Diagrams can be
/// used to add noise to any output, but a wrapper like this is required augment
/// the internal dynamics of a system.
///
/// It is anticipated that this new input would be wired up in a diagram to a
/// RandomSource System.  A method named get_noise_input_port() is provided
/// by this class to facilitate that wiring.
template <typename T>
class SystemWithAdditiveNoise : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemWithAdditiveNoise)

  /// Constructs the wrapper system from an original system.  If the original
  /// system has the continuous-time and/or discrete-time dynamics
  ///   ̇x(t) = f₁(x(t),u(t)),  x[n+1] = f₂(x[n],u[n]), ...
  /// where x is the state and u is the (original) input(s), then the new system
  /// will have the form
  ///   ̇x(t) = f₁(x(t),u(t)) + Gain₁*w(t),
  ///   x[n+1] = f₂(x[n],u[n]) + Gain₂*w[n], ...
  /// where Gain₁ is the top number-of-continuous-state rows of @p Gain, etc.
  ///
  /// @param system A system to be wrapped with the additional noise input.
  /// Ownership of the unique_ptr will be transferred to the newly constructed
  /// wrapper system.
  /// @param Gain The Gain matrix must have the same total number of rows as the
  /// system has total number of states.  The dimension of the noise input will
  /// be set by the number of columns in Gain.
  SystemWithAdditiveNoise(std::unique_ptr<System<T>> system,
                          const Eigen::Ref<const Eigen::MatrixXd>& Gain);

  // TODO(russt): Consider supporting block-diagonal Gain matrices with an
  // additional constructor that takes a std::list of gains (possibly being fed
  // from different noise input ports).  It feels premature to design that now,
  // without any use cases in hand.

  /// Returns the input port containing the externally applied input.
  const InputPortDescriptor<T>& get_noise_input_port() const;
};

}  // namespace systems
}  // namespace drake
