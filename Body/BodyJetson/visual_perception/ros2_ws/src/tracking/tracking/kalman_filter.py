from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class KalmanState:
    """
    Container for one Kalman filter state.
    """
    x: np.ndarray  # state mean vector, shape (8, 1)
    P: np.ndarray  # state covariance, shape (8, 8)


class KalmanFilter:
    """
    Linear Kalman filter for bounding box tracking in image space.

    State:
        x = [cx, cy, w, h, vx, vy, vw, vh]^T

    Measurement:
        z = [cx, cy, w, h]^T

    Control:
        u = [ux, uy]^T
        For now, default no camera motion: u = [0, 0]^T
    """

    def __init__(self) -> None:
        self.state_dim = 8
        self.measurement_dim = 4
        self.control_dim = 2

        # Default no-motion control input.
        self.u_k = np.zeros((self.control_dim, 1), dtype=np.float32)

        # Observation model H
        self.H = np.array(
            [
                [1, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 0],
            ],
            dtype=np.float32,
        )

        # Control model B
        # u_k shifts the predicted image-space center only.
        self.B = np.array(
            [
                [1, 0],
                [0, 1],
                [0, 0],
                [0, 0],
                [0, 0],
                [0, 0],
                [0, 0],
                [0, 0],
            ],
            dtype=np.float32,
        )

        # Observation noise covariance R
        self.R = np.diag(
            [
                8.0**2,   # cx measurement noise
                8.0**2,   # cy measurement noise
                12.0**2,  # w measurement noise
                12.0**2,  # h measurement noise
            ]
        ).astype(np.float32)

        # Process noise covariance Q
        self.Q = np.diag(
            [
                4.0**2,   # cx process noise
                4.0**2,   # cy process noise
                6.0**2,   # w process noise
                6.0**2,   # h process noise
                20.0**2,  # vx process noise
                20.0**2,  # vy process noise
                10.0**2,  # vw process noise
                10.0**2,  # vh process noise
            ]
        ).astype(np.float32)

    def set_control_input(self, u_x: float, u_y: float) -> None:
        """
        Set current control input u_k from outside the filter.
        (managed by motor command subscriber)
        """
        self.u_k[0, 0] = np.float32(u_x)
        self.u_k[1, 0] = np.float32(u_y)

    def _build_F(self, dt: float) -> np.ndarray:
        """
        Build the state transition matrix for constant-velocity motion.
        """
        return np.array(
            [
                [1, 0, 0, 0, dt, 0,  0,  0],
                [0, 1, 0, 0, 0,  dt, 0,  0],
                [0, 0, 1, 0, 0,  0,  dt, 0],
                [0, 0, 0, 1, 0,  0,  0,  dt],
                [0, 0, 0, 0, 1,  0,  0,  0],
                [0, 0, 0, 0, 0,  1,  0,  0],
                [0, 0, 0, 0, 0,  0,  1,  0],
                [0, 0, 0, 0, 0,  0,  0,  1],
            ],
            dtype=np.float32,
        )

    def initiate(self, cx: float, cy: float, w: float, h: float) -> KalmanState:
        """
        Initialize posterior state from the first detection.

        Initial state mean:
            x_0|0 = [cx, cy, w, h, 0, 0, 0, 0]^T
        """
        x0 = np.array(
            [[cx], [cy], [w], [h], [0.0], [0.0], [0.0], [0.0]],
            dtype=np.float32,
        )

        P0 = np.diag(
            [
                10.0**2,  # cx uncertainty
                10.0**2,  # cy uncertainty
                15.0**2,  # w uncertainty
                15.0**2,  # h uncertainty
                50.0**2,  # vx uncertainty
                50.0**2,  # vy uncertainty
                20.0**2,  # vw uncertainty
                20.0**2,  # vh uncertainty
            ]
        ).astype(np.float32)

        return KalmanState(x=x0, P=P0)

    def predict(self, state: KalmanState, dt: float) -> KalmanState:
        """
        Kalman prediction step:

            x_k|k-1 = F x_k-1|k-1 + B u_k
            P_k|k-1 = F P_k-1|k-1 F^T + Q
        """
        F = self._build_F(dt)

        x_pred = F @ state.x + self.B @ self.u_k
        P_pred = F @ state.P @ F.T + self.Q

        return KalmanState(x=x_pred, P=P_pred)

    def update(self, state: KalmanState, z: np.ndarray) -> KalmanState:
        """
        Kalman update step with measurement z of shape (4, 1):

            y = z - H x
            S = H P H^T + R
            K = P H^T S^-1
            x = x + K y
            P = (I - K H) P
        """
        if z.shape != (self.measurement_dim, 1):
            raise ValueError(f"Measurement z must have shape (4, 1), got {z.shape}")

        y = z - self.H @ state.x
        S = self.H @ state.P @ self.H.T + self.R
        K = state.P @ self.H.T @ np.linalg.inv(S)

        I = np.eye(self.state_dim, dtype=np.float32)

        x_upd = state.x + K @ y
        P_upd = (I - K @ self.H) @ state.P

        return KalmanState(x=x_upd, P=P_upd)

    @staticmethod
    def measurement_from_bbox(cx: float, cy: float, w: float, h: float) -> np.ndarray:
        """
        Build measurement vector z from a detection box.
        """
        return np.array([[cx], [cy], [w], [h]], dtype=np.float32)

    @staticmethod
    def bbox_from_state(state: KalmanState) -> tuple[float, float, float, float]:
        """
        Extract current box estimate [cx, cy, w, h] from state.
        """
        return (
            float(state.x[0, 0]),
            float(state.x[1, 0]),
            float(state.x[2, 0]),
            float(state.x[3, 0]),
        )