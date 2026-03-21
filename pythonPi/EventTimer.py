import time
import math
from collections import deque


class EventTimer:
    """Tracks timing between named events with rolling statistics."""

    def __init__(self, logger=None, windowSize=100):
        self.mLogger = logger
        self.mWindowSize = windowSize
        self.mEvents = {}

    def mark(self, eventName):
        """Record an event occurrence.

        Returns None on first call for a given event.
        Returns (delta, rate, stats) on subsequent calls where stats is a dict
        with min, max, mean, and std of the rolling window of deltas.
        """
        now = time.time()

        if eventName not in self.mEvents:
            self.mEvents[eventName] = {
                'lastTime': now,
                'deltas': deque(maxlen=self.mWindowSize),
            }
            return None

        event = self.mEvents[eventName]
        delta = now - event['lastTime']
        event['lastTime'] = now
        event['deltas'].append(delta)

        rate = (1.0 / delta) if delta > 0 else 0.0
        stats = self._computeStats(event['deltas'])

        if self.mLogger:
            self.mLogger.debug(
                '%s: %.4fs (%.1f Hz) | min=%.4f max=%.4f mean=%.4f std=%.4f',
                eventName, delta, rate,
                stats['min'], stats['max'], stats['mean'], stats['std']
            )

        return delta, rate, stats

    def getStats(self, eventName):
        """Get current rolling statistics for an event without recording a new mark."""
        if eventName not in self.mEvents or len(self.mEvents[eventName]['deltas']) == 0:
            return None
        return self._computeStats(self.mEvents[eventName]['deltas'])

    def _computeStats(self, deltas):
        n = len(deltas)
        mean = sum(deltas) / n
        variance = sum((d - mean) ** 2 for d in deltas) / n
        return {
            'min': min(deltas),
            'max': max(deltas),
            'mean': mean,
            'std': math.sqrt(variance),
        }
