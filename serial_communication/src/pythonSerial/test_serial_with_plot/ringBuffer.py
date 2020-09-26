import numpy as np

class RingBuffer(object):
    def __init__(self, update_plot_samples, default_value=0.0, dtype=float):
        """
        initialization
        """
        self.size_max = update_plot_samples
        self._data = np.empty(update_plot_samples, dtype=dtype)
        self._data.fill(default_value)
        self.size = 0

    def append(self, value):
        """
        append an element
        :param value:
        """
        self._data = np.roll(self._data, 1)
        self._data[0] = value

        self.size += 1

        if self.size == self.size_max:
            self.__class__ = RingBufferFull

    def get_all(self):
        """
        return a list of elements from the oldest to the newest
        """
        return self._data

class RingBufferFull(RingBuffer):
    def append(self, value):
        """
        append an element when buffer is full
        :param value:
        """
        self._data = np.roll(self._data, 1)
        self._data[0] = value
