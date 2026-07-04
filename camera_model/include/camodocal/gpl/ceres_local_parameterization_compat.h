#ifndef CERES_LOCAL_PARAMETERIZATION_COMPAT_H
#define CERES_LOCAL_PARAMETERIZATION_COMPAT_H

#include <ceres/manifold.h>
#include <ceres/autodiff_manifold.h>

namespace ceres {

// Compatibility wrapper: Ceres 2.2 removed LocalParameterization in favor of Manifold.
// This shim provides the old LocalParameterization API on top of Manifold,
// so existing code can be compiled with minimal changes.
//
// Usage: inherit from LocalParameterization and implement:
//   bool Plus(const double* x, const double* delta, double* x_plus_delta) const
//   bool ComputeJacobian(const double* x, double* jacobian) const
//   int GlobalSize() const
//   int LocalSize() const
class LocalParameterization : public Manifold {
public:
    virtual ~LocalParameterization() {}

    // Old API — user implements these
    virtual bool Plus(const double* x, const double* delta,
                      double* x_plus_delta) const = 0;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const = 0;
    virtual int GlobalSize() const = 0;
    virtual int LocalSize() const = 0;

    // Manifold interface — maps old names to new
    int AmbientSize() const final { return GlobalSize(); }
    int TangentSize() const final { return LocalSize(); }

    // Manifold::Plus is pure virtual; we provide it via the old Plus
    // (same signature, so the derived class's Plus satisfies both)

    bool PlusJacobian(const double* x, double* jacobian) const final {
        return ComputeJacobian(x, jacobian);
    }

    bool Minus(const double* y, const double* x, double* y_minus_x) const final {
        int n = LocalSize();
        for (int i = 0; i < n; ++i) {
            y_minus_x[i] = y[i] - x[i];
        }
        return true;
    }

    bool MinusJacobian(const double* x, double* jacobian) const final {
        int n = LocalSize();
        int m = GlobalSize();
        for (int i = 0; i < n * m; ++i) jacobian[i] = 0.0;
        for (int i = 0; i < n && i < m; ++i) {
            jacobian[i * m + i] = 1.0;
        }
        return true;
    }
};

// Compatibility for AutoDiffLocalParameterization
template <typename Functor, int kGlobalSize, int kLocalSize>
class AutoDiffLocalParameterization {
public:
    static LocalParameterization* Create() {
        return new AutoDiffManifold<Functor, kGlobalSize, kLocalSize>();
    }
};

}  // namespace ceres

#endif  // CERES_LOCAL_PARAMETERIZATION_COMPAT_H