import sys, os
import serial
import numpy as np
import math
import pyqtgraph as pg
import time
from collections import deque
from scipy.optimize import minimize
from functools import partial
from serial.tools import list_ports
from PyQt5 import QtGui, QtCore, QtWidgets
from mirror_lib import *

class mirror_gui(QtWidgets.QMainWindow):
    def __init__(self, Parent=None):
        super(mirror_gui, self).__init__(Parent)
        self.setWindowTitle("Mirror Mount Control")
        self.initialize()

    def initialize(self):
        # Select and initialize serial port
        ports = list_ports.comports()
        port_names = [p.name+": "+ p.description for p in ports]
        port_names.append("None: Simulated Port")
        print(port_names)
        port, ok = QtWidgets.QInputDialog.getItem(self, "Select Arduino Port", "Select Arduino Port", port_names, 0, False)
        if not ok:
            sys.exit()
        # If the port cannot be connected to, open a port to a simulated device.
        try:
            self.port = serial.Serial(port.partition(":")[0], baudrate=9600, timeout=1)
            self.simulated = False
        except:
            self.port = FakeSerial(port.partition(":")[0])
            self.simulated = True
            msg = QtWidgets.QMessageBox()
            msg.setText("Could not connect! Using simulated device.")
            msg.exec()

        # Set up window
        channel_names = ["Rb Back X", "Rb Back Y", "Rb Front X", "Rb Front Y", "K Back X", "K Back Y", "K Front X", "K Front Y"]
        steps = ["1", "10", "100", "1000"]
        inputs = ["Rb Photodiode", "K Photodiode"]

        self.n_motors = len(channel_names)

        channels = QtWidgets.QGridLayout()
        # object attributes for widgets and state information
        self.pos_displays = []
        self.min_edits = []
        self.max_edits = []
        self.opt_checks = []
        self.zero_buttons = []
        self.inc_buttons = []
        self.dec_buttons = []
        self.min_pos = [-100 for _ in range(self.n_motors)]
        self.max_pos = [100 for _ in range(self.n_motors)]
        self.optimizing = False
        self.opt_axes = []

        # Column label for stepper positions
        pos_label = QtWidgets.QLabel()
        pos_label.setText("Position")
        channels.addWidget(pos_label, 0, 1)

        # Column label for whether to optimize axis
        opt_label = QtWidgets.QLabel()
        opt_label.setText("Opt?")
        channels.addWidget(opt_label, 0, 5)

        # Column label for minimum stepper positions
        min_label = QtWidgets.QLabel()
        min_label.setText("Min")
        channels.addWidget(min_label, 0, 6)

        # Column label for maximum stepper positions
        max_label = QtWidgets.QLabel()
        max_label.setText("Max")
        channels.addWidget(max_label, 0, 7)

        for i, name in enumerate(channel_names):
            # Row label for axis names
            label = QtWidgets.QLabel()
            label.setText(name)
            channels.addWidget(label, i+1, 0)

            # Positions of each axis
            position_display = QtWidgets.QLineEdit()
            position_display.setReadOnly(True)
            position_display.setFixedWidth(70)
            channels.addWidget(position_display, i+1, 1)
            self.pos_displays.append(position_display)

            # Buttons to zero each axis
            zero_button = QtWidgets.QPushButton()
            zero_button.setText("Zero")
            channels.addWidget(zero_button, i+1, 2)
            self.zero_buttons.append(zero_button)

            # Buttons to manually increment axis position
            inc_button = QtWidgets.QPushButton()
            inc_button.setText("+")
            channels.addWidget(inc_button, i+1, 3)
            self.inc_buttons.append(inc_button)

            # Buttons to manually decrement axis position
            dec_button = QtWidgets.QPushButton()
            dec_button.setText("-")
            channels.addWidget(dec_button, i+1, 4)
            self.dec_buttons.append(dec_button)

            # Connect per-axis buttons to functions
            zero_f =  partial(self.zero, i)
            zero_button.clicked.connect(zero_f)
            inc_f =  partial(self.increment, i, [inc_button, dec_button, zero_button])
            inc_button.clicked.connect(inc_f)
            dec_f = partial(self.decrement, i, [inc_button, dec_button, zero_button])
            dec_button.clicked.connect(dec_f)

            # Checkboxes to enable optimization of each axis
            opt_checkbox = QtWidgets.QCheckBox()
            channels.addWidget(opt_checkbox, i+1, 5)
            opt_checkbox.stateChanged.connect(self.set_opt_axes)
            self.opt_checks.append(opt_checkbox)
            print(self.opt_checks)

            # Minimum position of each axis
            min_edit = QtWidgets.QLineEdit()
            min_edit.setFixedWidth(70)
            min_edit.setText(str(self.min_pos[i]))
            min_edit.setValidator(QtGui.QIntValidator())
            min_f = partial(self.set_min, i, None)
            min_edit.editingFinished.connect(min_f)
            channels.addWidget(min_edit, i+1, 6)
            self.min_edits.append(min_edit)

            # Maximum position of each axis
            max_edit = QtWidgets.QLineEdit()
            max_edit.setFixedWidth(70)
            max_edit.setText(str(self.max_pos[i]))
            max_edit.setValidator(QtGui.QIntValidator())
            max_f = partial(self.set_max, i, None)
            max_edit.editingFinished.connect(max_f)
            channels.addWidget(max_edit, i+1, 7)
            self.max_edits.append(max_edit)

        settings = QtWidgets.QGridLayout()

        # Label menu to select increment for manual axis positioning
        steps_label = QtWidgets.QLabel()
        steps_label.setText("Increment")
        settings.addWidget(steps_label, 0,0)

        # Menu to select increment for manual axis positioning
        self.steps_list = QtWidgets.QComboBox()
        self.steps_list.addItems(steps)
        self.inc = int(self.steps_list.currentText())
        self.steps_list.activated.connect(lambda: self.set_inc(self.steps_list.currentText()))
        settings.addWidget(self.steps_list, 0,1)

        # Label for selection of sensor measuring coupling efficiency
        inputs_label = QtWidgets.QLabel()
        inputs_label.setText("Sensor Input")
        settings.addWidget(inputs_label, 1,0)

        # Menu for selection of sensor measuring coupling efficiency
        self.inputs_list = QtWidgets.QComboBox()
        self.inputs_list.addItems(inputs)
        self.input = self.inputs_list.currentIndex()
        self.inputs_list.activated.connect(lambda: self.set_input(self.inputs_list.currentIndex()))
        settings.addWidget(self.inputs_list, 1,1)

        # Display for sensor value
        self.inputs_val = QtWidgets.QLineEdit()
        self.inputs_val.setReadOnly(True)
        self.inputs_val.setFixedWidth(120)
        settings.addWidget(self.inputs_val, 1,2)

        # Button to start optimization
        self.opt_button = QtWidgets.QPushButton()
        self.opt_button.setText("Optimize")
        self.opt_button.pressed.connect(self.optimize)
        self.opt_button.setEnabled(False)
        settings.addWidget(self.opt_button, 2,0)

        # Button to stop optimization
        self.stop_button = QtWidgets.QPushButton()
        self.stop_button.setText("Stop")
        self.stop_button.setEnabled(False)
        self.stop_button.pressed.connect(self.stop)
        settings.addWidget(self.stop_button, 2,1)

        vleft = QtWidgets.QVBoxLayout()
        vleft.addLayout(channels)
        vleft.addLayout(settings)
        
        # Plot of sensor value over time
        pg.setConfigOption('foreground','k')
        pg.setConfigOption('background','w')
        axis = pg.DateAxisItem(orientation='bottom')
        self.graph = pg.PlotWidget(axisItems={'bottom':axis})        
        self.graph.setLabel('left', 'Sensor Reading', color='k')
        self.graph.setLabel('bottom', 'Time', color='k')
        self.sens_times = deque(maxlen=3000)
        self.sens_vals = deque(maxlen=3000)
        pen = pg.mkPen(color=(0,0,0))
        self.data_line = self.graph.plot(self.sens_times, self.sens_vals, pen=pen)

        hmain = QtWidgets.QHBoxLayout()
        hmain.addLayout(vleft)
        hmain.addWidget(self.graph, 1)

        self.mainWidget = QtWidgets.QWidget()
        self.mainWidget.setLayout(hmain)
        self.setCentralWidget(self.mainWidget)

        # Initialize stepper controller and get initial positions
        self.controller = stepper_controller(self.port, self.n_motors)
        self.positions = [self.controller.get_status(i)[0] for i in range(self.n_motors)]
        for i, pos in enumerate(self.positions):
            self.pos_displays[i].setText(str(pos))
            self.set_min(i, pos - 100)
            self.set_max(i, pos + 100)

        # Initialize sensor reading
        input_timer = QtCore.QTimer(self)
        input_timer.timeout.connect(self.read_sensor)
        input_timer.start(100)

    # Keep track of which axes are checked to be optimized
    def set_opt_axes(self):
        self.opt_axes = []
        for i in range(self.n_motors):
            if self.opt_checks[i].isChecked():
                self.opt_axes.append(i)
        if len(self.opt_axes) == 0:
            self.opt_button.setEnabled(False)
        else:
            self.opt_button.setEnabled(True)
        print("setting optimization axes to ", self.opt_axes)

    # Set the minimum position of an axis
    def set_min(self, i, val):
        lineedit = self.min_edits[i]
        if val is None:
            val = int(lineedit.text())
        print("setmin {} to {}".format(i,val))
        if val < self.max_pos[i] and val <= self.positions[i]:
            self.min_pos[i] = val
        else:
            self.min_pos[i] = min(self.max_pos[i], self.positions[i])
        lineedit.setText(str(self.min_pos[i]))

    # Set the maximum position of an axis
    def set_max(self, i, val):
        lineedit = self.max_edits[i]
        if val is None:
            val = int(lineedit.text())
        print("setmax {} to {}".format(i,val))
        if val > self.min_pos[i] and val >= self.positions[i]:
            self.max_pos[i] = val
        else:
            self.max_pos[i] = max(self.min_pos[i], self.positions[i])
        lineedit.setText(str(self.max_pos[i]))

    # Set the increment for manually moving axes
    def set_inc(self, val):
        print("set_inc to {}".format(val))
        self.inc = int(val)

    # Select the input sensor
    def set_input(self, val):
        print("set_input to {}".format(val))
        self.input = val

    # Updates the displayed position of an axis
    def set_position(self, motor, pos):
        self.pos_displays[motor].setText(str(pos))
        self.positions[motor] = pos

    # Zeros an axis and updates min/max as appropriate
    def zero(self, motor):
        print("zeroing motor {} with range {} to {}".format(motor, self.min_pos[motor], self.max_pos[motor]))
        if self.max_pos[motor] < 0:
            self.set_max(motor, 0)
        if self.min_pos[motor] > 0:
            self.set_min(motor, 0)
        self.controller.reset_pos(motor)
        self.set_position(motor, 0)

    # Moves a motor to a position, or as close as possible if out of range. Can disable buttons during move
    def abs_move(self, motor, pos, buttons=None):
        pos = min(max(pos, self.min_pos[motor]), self.max_pos[motor])
        if pos == self.positions[motor]:
            return
        if buttons is not None:
            [b.setEnabled(False) for b in buttons]
        self.controller.move_abs(motor, pos)
        def check_done():
            self.set_position(motor, self.controller.get_status(motor)[3])
            if pos == self.positions[motor]:
                if buttons is not None:
                    [b.setEnabled(True) for b in buttons]
            else:
                timer = QtWidgets.QTimer(self)
                timer.setSingleShot(100, check_done)
                timer.start()
        if self.simulated:
            self.set_position(motor, pos)
            if buttons is not None:
                [b.setEnabled(True) for b in buttons]
        else:
            check_done()
        return pos
    
    # Increments an axis. Can disable buttons during move.
    def increment(self, motor, buttons):
        print("increment motor {}".format(motor))
        final_pos = min(self.max_pos[motor], self.positions[motor] + self.inc)
        self.abs_move(motor, final_pos, buttons)

    # Decrements an axis. Can disable buttons during move.
    def decrement(self, motor, buttons):
        print("decrement motor {}".format(motor))
        final_pos = max(self.min_pos[motor], self.positions[motor] - self.inc)
        self.abs_move(motor, final_pos, buttons)

    # Reads the value of the selected sensor and updates display and data for plotting
    def read_sensor(self):
        self.sensor_val = self.controller.get_sensor(self.input)
        self.inputs_val.setText('{:.3E}'.format(self.sensor_val))
        self.sens_times.appendleft(time.time())
        self.sens_vals.appendleft(self.sensor_val)
        self.data_line.setData(self.sens_times, self.sens_vals)
        return self.sensor_val
    
    # Optimizes the sensor value by moving the selected axes per the Nelder Mead algorithm. Respects min/max constraints and modifies loss function to stay away from edges of domain.
    def optimize(self):
        self.optimizing = True
        self.stop_button.setEnabled(True)
        for i in range(self.n_motors):
            self.opt_checks[i].setEnabled(False)
            self.min_edits[i].setEnabled(False)
            self.max_edits[i].setEnabled(False)
            self.inc_buttons[i].setEnabled(False)
            self.dec_buttons[i].setEnabled(False)
            self.zero_buttons[i].setEnabled(False)
        self.opt_button.setEnabled(False)
        self.steps_list.setEnabled(False)
        self.inputs_list.setEnabled(False)

        x0 = np.array([1000*self.positions[i] for i in self.opt_axes])
        def opt_fun(x):
            for i, a in enumerate(self.opt_axes):
                self.abs_move(a, int(round(0.001*x[i])))
            obj = self.read_sensor()
            for a in self.opt_axes:
                obj -= math.log(abs(self.positions[a] - self.max_pos[a] + 0.001))
                obj -= math.log(abs(self.positions[a] - self.min_pos[a] + 0.001))
            return obj
        def callback(xk):
            return not self.optimizing
        # reference for Nelder Mead for fiber coupling: https://ieeexplore.ieee.org/document/1288283?anchor=authors
        # see here for ideas to improve minimize: https://stackoverflow.com/questions/12180822/integer-step-size-in-scipy-optimize-minimize
        minimize(opt_fun, x0, method='Nelder-Mead', options={'xatol': 0.1, 'fatol': 1E-6, 'maxfev':150, 'disp':True})
        self.stop()

    # Stops running optimization
    def stop(self):
        if not self.optimizing:
            return
        self.optimizing = False
        self.stop_button.setEnabled(False)
        for i in range(self.n_motors):
            self.opt_checks[i].setEnabled(True)
            self.min_edits[i].setEnabled(True)
            self.max_edits[i].setEnabled(True)
            self.inc_buttons[i].setEnabled(True)
            self.dec_buttons[i].setEnabled(True)
            self.zero_buttons[i].setEnabled(True)
        self.opt_button.setEnabled(True)
        self.steps_list.setEnabled(True)
        self.inputs_list.setEnabled(True)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    # Call Python every 100 ms so Ctrl-c works
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)

    # Set up window
    w = mirror_gui()
    w.setGeometry(100, 100, 1200, 380)
    w.show()

    # Run event loop
    ret = app.exec()
    if w.port is not None:
        w.port.close()
    sys.exit(ret)