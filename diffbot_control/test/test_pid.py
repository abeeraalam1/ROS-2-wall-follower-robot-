"""Unit tests for the hand-written PID controller."""

from diffbot_control.pid import PID
import pytest


def simulate_first_order(pid, setpoint=1.0, tau=0.5, dt=0.01, steps=2000, disturbance=0.0):
    """Plant: tau * dy/dt = -y + u + disturbance. Returns the output trace."""
    y = 0.0
    trace = []
    for _ in range(steps):
        u = pid.update(setpoint - y, dt)
        y += dt * (-y + u + disturbance) / tau
        trace.append(y)
    return trace


def test_proportional_only():
    pid = PID(kp=2.0)
    assert pid.update(0.5, 0.1) == pytest.approx(1.0)
    assert pid.update(-0.25, 0.1) == pytest.approx(-0.5)


def test_p_only_leaves_steady_state_error():
    trace = simulate_first_order(PID(kp=2.0))
    # Classic result: y_ss = Kp / (1 + Kp) * r
    assert trace[-1] == pytest.approx(2.0 / 3.0, abs=1e-3)


def test_integral_removes_steady_state_error():
    trace = simulate_first_order(PID(kp=2.0, ki=2.0), disturbance=-0.3)
    assert trace[-1] == pytest.approx(1.0, abs=1e-3)


def test_output_is_clamped():
    pid = PID(kp=100.0, output_limits=(-1.0, 1.0))
    assert pid.update(5.0, 0.1) == 1.0
    assert pid.update(-5.0, 0.1) == -1.0


def test_anti_windup_limits_overshoot():
    """With a saturated actuator, anti-windup should give much less overshoot."""
    def run(anti_windup):
        pid = PID(kp=1.0, ki=5.0, output_limits=(-1.2, 1.2), integral_limits=(-1e9, 1e9),
                  anti_windup=anti_windup)
        return max(simulate_first_order(pid, steps=3000))

    with_anti_windup = run(True)
    without = run(False)
    assert with_anti_windup < 1.05      # barely overshoots the setpoint of 1.0
    assert without > with_anti_windup + 0.1


def test_conditional_integration_freezes_integral_while_saturated():
    pid = PID(kp=1.0, ki=1.0, output_limits=(-1.0, 1.0), integral_limits=(-10, 10))
    for _ in range(100):
        pid.update(10.0, 0.1)  # hugely saturated
    assert pid.terms.i < 1.0


def test_no_derivative_kick_on_first_sample():
    pid = PID(kp=0.0, kd=10.0)
    assert pid.update(5.0, 0.01) == 0.0
    assert pid.update(6.0, 0.01) == pytest.approx(10.0 * 100.0)


def test_derivative_filter_smooths_steps():
    raw = PID(kp=0.0, kd=1.0)
    filtered = PID(kp=0.0, kd=1.0, derivative_tau=0.1)
    raw.update(0.0, 0.01)
    filtered.update(0.0, 0.01)
    assert abs(filtered.update(1.0, 0.01)) < abs(raw.update(1.0, 0.01))


def test_reset_clears_state():
    pid = PID(kp=1.0, ki=1.0, kd=1.0)
    for _ in range(10):
        pid.update(1.0, 0.1)
    pid.reset()
    assert pid.update(1.0, 0.1) == pytest.approx(1.0 + 0.1)  # P + one step of I, no D


def test_non_positive_dt_returns_previous_output():
    pid = PID(kp=1.0, ki=1.0)
    out = pid.update(1.0, 0.1)
    assert pid.update(5.0, 0.0) == out


def test_invalid_limits_raise():
    with pytest.raises(ValueError):
        PID(kp=1.0, output_limits=(1.0, -1.0))
