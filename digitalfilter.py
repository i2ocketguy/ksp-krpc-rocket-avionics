# partially derived and inspired by https://www.samproell.io/posts/yarppg/yarppg-live-digital-filter/
import scipy.signal
import numpy as np
from collections import deque

class low_pass_filter:
    """Base class for live filters.
    """
    def process(self, x):
        # do not process NaNs
        if np.isnan(x):
            return x

        return self._process(x)

    def __call__(self, x):
        return self.process(x)

    def _process(self, x):
        raise NotImplementedError("Derived class must implement _process")

class low_pass_filter(low_pass_filter):
    def __init__(self, order = 4, fc = 2, clock_rate = 10.0):
        self.b, self.a = scipy.signal.iirfilter(order, fc, fs=clock_rate, btype="low", ftype="butter")
        self._xs = deque([0] * len(self.b), maxlen=len(self.b))
        self._ys = deque([0] * (len(self.a) - 1), maxlen=len(self.a)-1)

    def _process(self, x):
        """Filter incoming data with standard difference equations.
        """
        self._xs.appendleft(x)
        y = np.dot(self.b, self._xs) - np.dot(self.a[1:], self._ys)
        y = y / self.a[0]
        self._ys.appendleft(y)

        return y


def test():
    np.random.seed(42)  # for reproducibility
    # create time steps and corresponding sine wave with Gaussian noise
    fs = 30  # sampling rate, Hz
    ts = np.arange(0, 5, 1.0 / fs)  # time vector - 5 seconds

    ys = np.sin(2*np.pi * 1.0 * ts)  # signal @ 1.0 Hz, without noise
    yerr = 0.5 * np.random.normal(size=len(ts))  # Gaussian noise
    yraw = ys + yerr
    live_lfilter = low_pass_filter(4, 1, 10.0)
    # simulate live filter - passing values one by one
    y_live_lfilter = [live_lfilter(y) for y in yraw]

    import matplotlib.pyplot as plt
    plt.figure(figsize=[6.4, 2.4])
    plt.plot(ts, yraw, label="Noisy signal")
    plt.plot(ts, y_live_lfilter, lw=4, ls="dashed", label="LiveLFilter")
    plt.legend(loc="lower center", bbox_to_anchor=[0.5, 1], ncol=3,
            fontsize="smaller")
    plt.xlabel("Time / s")
    plt.ylabel("Amplitude")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    test()