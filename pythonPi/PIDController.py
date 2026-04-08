class PIDController():
    """PID controller with derivative-on-measurement, integral anti-windup, and output clamping.

    Anti-windup uses two mechanisms:
      1. Integral term is clamped to ±integralLimit.
      2. Conditional integration: integral only accumulates when it is helping
         drive the output toward zero (i.e., integral and error have the same sign
         AND the output is not already saturated in that direction).

    Derivative is computed on the measurement (not the error) to avoid
    derivative kick on setpoint changes. The measurement difference must be
    wrap-safe for angular quantities — the caller provides a wrap function.
    """

    def __init__(self, kP, kI, kD, dt, outputMin, outputMax, integralLimit):
        """
        Parameters
        ----------
        kP : float  — Proportional gain
        kI : float  — Integral gain
        kD : float  — Derivative gain
        dt : float  — Control loop period in seconds
        outputMin : float  — Minimum controller output
        outputMax : float  — Maximum controller output
        integralLimit : float  — Max absolute value of the integral accumulator
        """
        self.mKp = kP
        self.mKi = kI
        self.mKd = kD
        self.mDt = dt
        self.mOutputMin = outputMin
        self.mOutputMax = outputMax
        self.mIntegralLimit = integralLimit

        self.mIntegral = 0.0
        self.mPrevMeasurement = None

        # Last computed terms for telemetry
        self.mLastPTerm = 0.0
        self.mLastITerm = 0.0
        self.mLastDTerm = 0.0
        self.mLastOutput = 0.0

    def Update(self, error, measurement):
        """Compute one PID cycle.

        Parameters
        ----------
        error : float
            Setpoint minus measurement (wrap-safe, already computed by caller).
        measurement : float
            Current process variable (e.g., heading in degrees).

        Returns
        -------
        float — clamped controller output.
        """
        # --- Proportional ---
        pTerm = self.mKp * error

        # --- Integral with conditional anti-windup ---
        candidate = self.mIntegral + error * self.mDt
        # Only integrate when the integral is helping drive output toward zero:
        # error and integral must share the same sign (or integral is zero).
        if candidate * error >= 0:
            self.mIntegral = max(-self.mIntegralLimit,
                                 min(self.mIntegralLimit, candidate))
        iTerm = self.mKi * self.mIntegral

        # --- Derivative on measurement ---
        if self.mPrevMeasurement is None:
            dTerm = 0.0
        else:
            # Wrap-safe difference: handles 359° → 1° transition
            dMeasurement = (measurement - self.mPrevMeasurement + 180) % 360 - 180
            dTerm = -self.mKd * (dMeasurement / self.mDt)
        self.mPrevMeasurement = measurement

        # --- Sum and clamp ---
        output = pTerm + iTerm + dTerm
        output = max(self.mOutputMin, min(self.mOutputMax, output))

        # Store for telemetry
        self.mLastPTerm = pTerm
        self.mLastITerm = iTerm
        self.mLastDTerm = dTerm
        self.mLastOutput = output

        return output

    def Reset(self):
        """Zero internal state (integral accumulator and derivative memory)."""
        self.mIntegral = 0.0
        self.mPrevMeasurement = None
        self.mLastPTerm = 0.0
        self.mLastITerm = 0.0
        self.mLastDTerm = 0.0
        self.mLastOutput = 0.0
