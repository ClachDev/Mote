"""Appearance-based floor / free-space segmentation for monocular obstacle marking.

Approach (a) from the vision roadmap: assume the patch of floor directly ahead of
the robot is traversable, model its appearance, and label everything that does not
match as an obstacle. The lowest obstacle pixel in each image column is the nearest
obstacle in that bearing; back-projected onto the floor plane (GroundProjector) it
gives a metric obstacle range, i.e. the camera acts like a floor-boundary scanner.

This is deliberately light (classical CV, real-time on the Pi CPU). The flat-floor
+ known-mount geometry pins the metric scale, so there is no monocular scale
ambiguity. The module is shared by the offline bag harness and the ROS node.
"""

import cv2
import numpy as np


class FloorSegmenter:
    """Segment the floor by matching pixels to a seed patch's hue/saturation model.

    The seed patch is a rectangle near the bottom-centre of the image — the floor
    just ahead of the robot. A 2D hue-saturation histogram of that patch models the
    floor appearance; pixels whose (H, S) bin is well represented in the seed are
    labelled floor. Hue/saturation (not value) keeps it robust to the brightness
    changes that dominate indoor lighting.
    """

    def __init__(
        self,
        seed_rows=(0.80, 0.99),
        seed_cols=(0.30, 0.70),
        h_bins=30,
        s_bins=32,
        thresh=0.02,
        horizon_row=None,
        open_ksize=5,
        close_ksize=15,
    ):
        self.seed_rows = seed_rows
        self.seed_cols = seed_cols
        self.h_bins = h_bins
        self.s_bins = s_bins
        self.thresh = thresh
        self.horizon_row = horizon_row
        self.open_ksize = open_ksize
        self.close_ksize = close_ksize

    def floor_mask(self, bgr):
        """Return a uint8 {0,255} mask of pixels classified as floor."""
        h, w = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        r0, r1 = int(self.seed_rows[0] * h), int(self.seed_rows[1] * h)
        c0, c1 = int(self.seed_cols[0] * w), int(self.seed_cols[1] * w)
        seed = hsv[r0:r1, c0:c1]

        hist = cv2.calcHist(
            [seed], [0, 1], None, [self.h_bins, self.s_bins], [0, 180, 0, 256]
        )
        # Smooth so plank-to-plank and lighting variation in the floor (nearby
        # H/S bins) still counts as floor, not just the exact seed colours.
        hist = cv2.GaussianBlur(hist, (0, 0), 1.0)
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)

        back = cv2.calcBackProject([hsv], [0, 1], hist, [0, 180, 0, 256], scale=1)
        mask = (back >= self.thresh).astype(np.uint8) * 255

        # Everything at/above the horizon cannot be floor.
        horizon = self.horizon_row if self.horizon_row is not None else int(0.5 * h)
        mask[:horizon, :] = 0

        if self.open_ksize:
            k = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.open_ksize, self.open_ksize)
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        if self.close_ksize:
            k = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.close_ksize, self.close_ksize)
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        # Keep only floor connected to the seed band at the bottom: the traversable
        # floor is one region growing up from directly ahead of the robot. This
        # rejects stray colour matches inside obstacles (e.g. wood furniture).
        mask = self._keep_bottom_connected(mask, r0, r1, c0, c1)
        return mask

    @staticmethod
    def _keep_bottom_connected(mask, r0, r1, c0, c1):
        n, labels = cv2.connectedComponents(mask)
        if n <= 1:
            return mask
        seed_labels = np.unique(labels[r0:r1, c0:c1])
        seed_labels = seed_labels[seed_labels != 0]
        if len(seed_labels) == 0:
            return np.zeros_like(mask)
        return (np.isin(labels, seed_labels).astype(np.uint8)) * 255

    def boundary_rows(self, mask):
        """Per-column row of the floor/obstacle boundary.

        Scanning up from the bottom, the boundary is the first row where the
        contiguous floor run from the bottom ends. Columns with no floor at the
        bottom get the image bottom (obstacle right at the robot); columns that are
        floor all the way to the horizon get the horizon row (free / no obstacle).
        Returns an (w,) int array of boundary rows and an (w,) bool 'is an obstacle'.
        """
        h, w = mask.shape
        horizon = self.horizon_row if self.horizon_row is not None else int(0.5 * h)
        floor = mask > 0
        # Contiguous floor run upward from the bottom, per column (vectorised).
        contig = np.cumprod(floor[::-1, :], axis=0)
        run = contig.sum(axis=0).astype(np.int32)  # floor rows from the bottom
        boundary = np.clip((h - 1) - run, horizon, h - 1)
        is_obstacle = run < (h - horizon)  # didn't reach the horizon: obstacle
        return boundary, is_obstacle

    def detect(self, bgr, proj, col_step=4):
        """Full pipeline: floor boundary -> metric obstacle points in the base frame.

        Returns (points_xy, mask, boundary, is_obstacle): points_xy is (M, 2) of
        obstacle ground points (only columns flagged as obstacles and whose boundary
        ray hits the floor), in the base frame.
        """
        mask = self.floor_mask(bgr)
        boundary, is_obstacle = self.boundary_rows(mask)
        cols = np.arange(0, mask.shape[1], col_step)
        uv = np.column_stack([cols, boundary[cols]]).astype(np.float64)
        pts, valid = proj.pixels_to_ground(uv)
        keep = valid & is_obstacle[cols]
        return pts[keep, :2], mask, boundary, is_obstacle
