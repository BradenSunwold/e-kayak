import math


class RCFilter():
    """First-order discrete RC low-pass filter.

    This is the digital equivalent of an analog RC low-pass circuit.
    Each call to Feed() advances one sample period.

    Smoothing coefficient:
        alpha = dt / (tau + dt)

    Step response:
        output(t) = 1 - e^(-t / tau)
        ~63% at t = 1*tau
        ~86% at t = 2*tau
        ~95% at t = 3*tau
        ~99% at t = 5*tau

    Calculating tau from a desired cutoff frequency:
        tau = 1 / (2 * pi * f_cutoff)

        Example: for a 1 Hz cutoff -> tau = 1 / (2*pi*1) ≈ 0.159 seconds
                 for a 0.5 Hz cutoff -> tau = 1 / (2*pi*0.5) ≈ 0.318 seconds

    Calculating tau from a desired rise time (10% to 90%):
        rise_time ≈ 2.2 * tau
        tau ≈ rise_time / 2.2

        Example: for a 3 second rise time -> tau ≈ 3.0 / 2.2 ≈ 1.36 seconds

    Constructors
    ------------
    RCFilter(tau, dt)             — specify time constant directly
    RCFilter.FromCutoff(fc, dt)   — specify cutoff frequency in Hz
    RCFilter.FromRiseTime(rt, dt) — specify 10-90% rise time in seconds
    """

    def __init__(self, tau, dt):
        """Create an RC filter from a time constant.

        Parameters
        ----------
        tau : float
            Time constant in seconds. Larger = slower / smoother.
        dt : float
            Sample period in seconds (time between calls to Feed).
        """
        self.mAlpha = dt / (tau + dt)
        self.mOutput = 0.0

    @classmethod
    def FromCutoff(cls, cutoffHz, dt):
        """Create an RC filter from a cutoff frequency (-3dB point).

        Parameters
        ----------
        cutoffHz : float
            Cutoff frequency in Hz.
        dt : float
            Sample period in seconds.
        """
        tau = 1.0 / (2.0 * math.pi * cutoffHz)
        return cls(tau, dt)

    @classmethod
    def FromRiseTime(cls, riseTime, dt):
        """Create an RC filter from a desired 10-90% rise time.

        Parameters
        ----------
        riseTime : float
            Desired rise time in seconds (time from 10% to 90% of step).
        dt : float
            Sample period in seconds.
        """
        tau = riseTime / 2.2
        return cls(tau, dt)

    def Feed(self, dataIn):
        self.mOutput += self.mAlpha * (dataIn - self.mOutput)
        return self.mOutput

    def Clear(self):
        self.mOutput = 0.0

    def LastOutput(self):
        return self.mOutput

    def SetTau(self, tau, dt):
        """Update the time constant and sample period."""
        self.mAlpha = dt / (tau + dt)
