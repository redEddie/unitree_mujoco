"""
Real-time IMU data plotting utilities using PyQtGraph
"""
import numpy as np
import threading
from collections import deque
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore


class IMUPlotter:
    """Real-time IMU vs Ground Truth acceleration plotting using PyQtGraph"""

    def __init__(self, window_size: int = 200):
        """
        Initialize IMU plotter

        Args:
            window_size: Number of data points to display in the plot
        """
        self.window_size = window_size
        self.timestamps = deque(maxlen=window_size)
        self.imu_acc_x = deque(maxlen=window_size)
        self.imu_acc_y = deque(maxlen=window_size)
        self.imu_acc_z = deque(maxlen=window_size)
        self.gt_acc_x = deque(maxlen=window_size)
        self.gt_acc_y = deque(maxlen=window_size)
        self.gt_acc_z = deque(maxlen=window_size)

        self.running = False
        self.plot_thread = None
        self.app = None
        self.win = None

    def add_data(self, timestamp: float, imu_acc: np.ndarray, gt_acc: np.ndarray):
        """
        Add new data point to the plot (fast, non-blocking)

        Args:
            timestamp: Current timestamp
            imu_acc: IMU acceleration [x, y, z]
            gt_acc: Ground truth acceleration [x, y, z]
        """
        self.timestamps.append(timestamp)
        self.imu_acc_x.append(imu_acc[0])
        self.imu_acc_y.append(imu_acc[1])
        self.imu_acc_z.append(imu_acc[2])
        self.gt_acc_x.append(gt_acc[0])
        self.gt_acc_y.append(gt_acc[1])
        self.gt_acc_z.append(gt_acc[2])

    def start(self):
        """Start the plotting thread"""
        self.running = True
        self.plot_thread = threading.Thread(target=self._plot_loop, daemon=True)
        self.plot_thread.start()

    def stop(self):
        """Stop the plotting thread"""
        self.running = False
        if self.app is not None:
            self.app.quit()
        if self.plot_thread:
            self.plot_thread.join(timeout=1.0)

    def _plot_loop(self):
        """Main plotting loop (runs in separate thread with PyQtGraph)"""
        # Create Qt application
        self.app = QtWidgets.QApplication.instance()
        if self.app is None:
            self.app = QtWidgets.QApplication([])

        # Create window with plots
        self.win = pg.GraphicsLayoutWidget(show=True, title="IMU vs Ground Truth Acceleration")
        self.win.resize(1200, 800)
        self.win.setWindowTitle('IMU vs Ground Truth Acceleration')

        # Configure PyQtGraph for better performance
        pg.setConfigOptions(antialias=True)

        # Create three plots for X, Y, Z
        plots = []
        curves_imu = []
        curves_gt = []

        labels = ['X (Forward)', 'Y (Left)', 'Z (Up)']
        colors_imu = ['r', 'g', 'b']
        colors_gt = ['#8B0000', '#006400', '#00008B']  # Dark red, dark green, dark blue

        for i, (label, color_imu, color_gt) in enumerate(zip(labels, colors_imu, colors_gt)):
            p = self.win.addPlot(row=i, col=0)
            p.setTitle(f'{label} Acceleration')
            p.setLabel('left', 'Acc', units='m/s²')
            p.setLabel('bottom', 'Time', units='s')
            p.setYRange(-15, 15)
            p.showGrid(x=True, y=True, alpha=0.3)
            p.addLegend()

            # Add curves
            curve_imu = p.plot(pen=pg.mkPen(color_imu, width=2), name='IMU')
            curve_gt = p.plot(pen=pg.mkPen(color_gt, width=2, style=QtCore.Qt.DashLine), name='GT')

            plots.append(p)
            curves_imu.append(curve_imu)
            curves_gt.append(curve_gt)

        # Manual event loop for better real-time updates in separate thread
        import time as time_module

        while self.running:
            # Update plots if we have data
            if len(self.timestamps) > 1:
                t = np.array(self.timestamps)

                # Update all three plots
                curves_imu[0].setData(t, np.array(self.imu_acc_x))
                curves_gt[0].setData(t, np.array(self.gt_acc_x))
                curves_imu[1].setData(t, np.array(self.imu_acc_y))
                curves_gt[1].setData(t, np.array(self.gt_acc_y))
                curves_imu[2].setData(t, np.array(self.imu_acc_z))
                curves_gt[2].setData(t, np.array(self.gt_acc_z))

            # CRITICAL: Process Qt events to update GUI display
            # This makes the plot update in real-time
            QtWidgets.QApplication.processEvents()

            # Limit update rate to ~20Hz (50ms sleep in plotter thread only)
            time_module.sleep(0.05)

        # Clean up when stopped
        self.win.close()
        self.app.quit()
