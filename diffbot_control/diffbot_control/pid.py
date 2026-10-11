"""A discrete PID controller written from scratch (no control libraries).

The controller computes

    u(t) = Kp * e(t) + Ki * integral(e dt) + Kd * de/dt

with four practical additions that matter on a real (or simulated) robot:

1. Output clamping
   Actuators have limits (a wheel cannot spin infinitely fast), so the
   output is saturated to ``output_limits``.

2. Anti-windup by conditional integration (a.k.a. "clamping")
   While the output is saturated, integrating the error further only makes
   the integral term grow ("wind up") without changing what the robot does.
   When the controller later needs to reverse, that stored integral causes a
   large overshoot. We therefore *skip* the integration step whenever the
   output is saturated **and** the error would push it further into
   saturation. The integral is also hard-limited to ``integral_limits``.

3. Derivative low-pass filter
   The derivative of a noisy signal (e.g. a lidar range) is mostly noise.
   A first-order low-pass filter with time constant ``derivative_tau``
   smooths it:  d_f += alpha * (d_raw - d_f),  alpha = dt / (tau + dt).

4. No "derivative kick" on the first sample
   On the first call there is no previous error, so the derivative term is
   zero instead of a huge spike.

The class has no ROS dependency, so it can be unit tested with plain pytest
and reused in any Python control loop.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

Limits = Tuple[Optional[float], Optional[float]]


def clamp(value: float, limits: Limits) -> float:
    """Clamp ``value`` into ``limits`` (``None`` means unbounded)."""
    low, high = limits
    if low is not None and value < low:
        return low
    if high is not None and value > high:
        return high
    return value


@dataclass
class PIDTerms:
    """The individual contributions of the last update (useful for tuning)."""

    p: float = 0.0
    i: float = 0.0
    d: float = 0.0
    output: float = 0.0


class PID:
    """Discrete-time PID controller with anti-windup and a filtered derivative.

    Args:
        kp: Proportional gain.
        ki: Integral gain (per second).
        kd: Derivative gain (seconds).
        output_limits: ``(min, max)`` saturation of the output.
        integral_limits: ``(min, max)`` bound on the *integral term*
            contribution (``ki * integral``). Defaults to ``output_limits``.
        derivative_tau: Time constant [s] of the derivative low-pass filter.
            ``0`` disables filtering.
        anti_windup: Enable conditional integration. Only turn this off to
            see what windup looks like (see ``test_anti_windup_limits_overshoot``).
    """

    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        kd: float = 0.0,
        output_limits: Limits = (None, None),
        integral_limits: Optional[Limits] = None,
        derivative_tau: float = 0.0,
        anti_windup: bool = True,
    ) -> None:
        if output_limits[0] is not None and output_limits[1] is not None:
            if output_limits[0] > output_limits[1]:
                raise ValueError('output_limits: min must be <= max')
        if derivative_tau < 0.0:
            raise ValueError('derivative_tau must be >= 0')

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_limits = integral_limits if integral_limits is not None else output_limits
        self.derivative_tau = derivative_tau
        self.anti_windup = anti_windup

        self.terms = PIDTerms()
        self.reset()

    def reset(self) -> None:
        """Forget all history (call this when the setpoint jumps or on mode change)."""
        self._integral_term = 0.0  # already multiplied by ki
        self._prev_error: Optional[float] = None
        self._filtered_derivative = 0.0
        self.terms = PIDTerms()

    def update(self, error: float, dt: float) -> float:
        """Advance the controller by one step.

        Args:
            error: setpoint - measurement.
            dt: Time since the previous update [s]. Must be > 0.

        Returns:
            The saturated control output.
        """
        if dt <= 0.0:
            # A non-positive dt (e.g. repeated timestamp) gives no new
            # information; return the last output instead of dividing by zero.
            return self.terms.output

        # --- Proportional -------------------------------------------------
        p = self.kp * error

        # --- Derivative (filtered, no kick on first sample) ---------------
        if self._prev_error is None:
            raw_derivative = 0.0
        else:
            raw_derivative = (error - self._prev_error) / dt
        if self.derivative_tau > 0.0:
            alpha = dt / (self.derivative_tau + dt)
            self._filtered_derivative += alpha * (raw_derivative - self._filtered_derivative)
        else:
            self._filtered_derivative = raw_derivative
        d = self.kd * self._filtered_derivative
        self._prev_error = error

        # --- Integral with conditional integration (anti-windup) ----------
        candidate_integral = clamp(
            self._integral_term + self.ki * error * dt, self.integral_limits
        )
        unsaturated = p + candidate_integral + d
        saturated = clamp(unsaturated, self.output_limits)
        pushing_further = (unsaturated > saturated and error > 0.0) or (
            unsaturated < saturated and error < 0.0
        )
        if not (self.anti_windup and pushing_further):
            self._integral_term = candidate_integral

        output = clamp(p + self._integral_term + d, self.output_limits)
        self.terms = PIDTerms(p=p, i=self._integral_term, d=d, output=output)
        return output
