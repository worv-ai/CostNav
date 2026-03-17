from collections import deque

import numpy as np
from tqdm import tqdm

pbar = tqdm()


class VelData:
    def __init__(self, ref_clock, timeout: float = 3, name: str = "", rate: float = 20.0, ref_logger=None):
        self.ref_clock = ref_clock
        self.timeout = timeout
        self.last_time_received = float("-inf")
        self.data = None
        self.name = name
        self.rate = rate  # Hz, used for latency compensation calculations
        self.is_first_after_init = True  # Flag to track first data set after initialization
        self.ref_logger = ref_logger

    def get(self):
        """
        Get the next velocity command.

        Returns:
            tuple: (linear_velocity, angular_velocity) or None if no data
        """
        if self.data is None:
            return None

        return self.data.popleft() if self.data else None

    def set(self, data, delta_t=None):
        """
        Set the velocity prediction data with optional latency compensation.
        For the first time after initialization, use the whole action without latency compensation.

        Args:
            data: Array of shape (N, 2) containing velocity predictions
            delta_t: Optional time difference in seconds for latency compensation

        Returns:
            bool: True if data was set successfully, False if validation failed
        """
        # Validate input data
        if data is None:
            self.logwarn("Received None velocity data, clearing data")
            self.data = None
            self.last_time_received = self._stamp_to_sec(self.ref_clock.now().to_msg())
            return False

        # Convert to numpy array if needed for validation
        if not isinstance(data, np.ndarray):
            try:
                data = np.array(data)
            except Exception as e:
                self.logerr(f"Failed to convert velocity data to numpy array: {e}")
                return False

        # Validate data is not empty
        if data.size == 0:
            self.logwarn("Received empty velocity data, skipping update")
            return False

        # Validate data shape
        if len(data.shape) != 2 or data.shape[1] != 2:
            self.logwarn(f"Invalid velocity data shape {data.shape}, expected (N, 2)")
            return False

        # Process validated data
        if self.is_first_after_init:
            # First time after initialization: use whole action without latency compensation
            self.data = deque(data)
            self.is_first_after_init = False
            self.loginfo("First velocity data after initialization: using whole action (no latency compensation)")
        elif delta_t is not None:
            # Apply latency compensation at ingestion time
            compensated_data = self._apply_latency_compensation(data, delta_t)
            if compensated_data is not None and compensated_data.size > 0:
                self.data = deque(compensated_data)
            else:
                self.logwarn("Latency compensation resulted in empty data")
                return False
        else:
            # No latency compensation
            self.data = deque(data)

        self.last_time_received = self._stamp_to_sec(self.ref_clock.now().to_msg())
        return True

    def _stamp_to_sec(self, stamp) -> float:
        """Convert a ROS2 Time stamp to seconds as a float."""
        return stamp.sec + stamp.nanosec * 1e-9

    def logwarn(self, msg):
        if self.ref_logger is not None:
            self.ref_logger.warning(msg)

    def loginfo(self, msg):
        if self.ref_logger is not None:
            self.ref_logger.info(msg)

    def logerr(self, msg):
        if self.ref_logger is not None:
            self.ref_logger.error(msg)

    def _apply_latency_compensation(self, vel_data, delta_t: float):
        """
        Apply latency compensation to velocity data.

        Args:
            vel_data: Array of shape (N, 2) containing velocity predictions
            delta_t: Time difference in seconds (t2 - t1)

        Returns:
            numpy.ndarray: Compensated velocity data
        """
        if vel_data is None or len(vel_data) == 0:
            return None

        # Convert to numpy array if needed
        if not isinstance(vel_data, np.ndarray):
            vel_data = np.array(vel_data)

        if delta_t <= 0:
            # No compensation needed
            return vel_data

        # Calculate sample position and apply compensation
        exact_sample_position = delta_t * self.rate

        if exact_sample_position >= len(vel_data):
            # Use last available sample
            return vel_data[-1:]

        # Calculate next index using ceiling to handle both exact and non-exact alignment
        next_idx = int(np.ceil(exact_sample_position))
        return vel_data[next_idx:]

    def is_valid(self, verbose: bool = False):
        time_waited = self._stamp_to_sec(self.ref_clock.now().to_msg()) - self.last_time_received
        valid = time_waited < self.timeout
        if verbose and not valid:
            # print(f"Not receiving {self.name} data for {time_waited} seconds (timeout: {self.timeout} seconds)")
            pbar.set_description(
                f"Not receiving {self.name} data for {time_waited:.2f} seconds (timeout: {self.timeout} seconds)"
            )
        return valid
