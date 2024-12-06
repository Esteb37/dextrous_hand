import sys
import json
import rclpy
from rclpy.node import Node
from ruamel.yaml import YAML
from PyQt5.QtCore import QTimer
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QPushButton, QLabel, QWidget, QHBoxLayout, QLineEdit

from dextrous_hand.utils.constants import RETARGETER_PARAMS
from dextrous_hand.utils.utils import parent_dir

configuration = 'default'

class ParameterAdjuster(Node):
    def __init__(self, parameters):
        super().__init__('parameter_adjuster')
        # Declare parameters to adjust
        self.parameters = parameters

        self.param_publisher = self.create_publisher(String, 'retargeter_params', 10)

        self.publish_loop = self.create_timer(0.1, self.publish)

    def publish(self):
        msg = String()
        msg.data = json.dumps(self.parameters)
        self.param_publisher.publish(msg)

    def set_parameter_value(self, param, keyvector, value):
        self.parameters[param][keyvector] = value

    def set_mano_value(self, finger, param, axis, value):
        self.parameters["retargeter_adjustments"][finger][param][["x", "y", "z"].index(axis)] = value

    def save(self):
        # Save the parameters to a file
        yaml_path = parent_dir() + "/data/retargeter/" + configuration + ".yaml"

        yaml = YAML()
        yaml.preserve_quotes = True  # Preserves quotes and structure

        with open(yaml_path) as file:
            data = yaml.load(file)

        for param, value in self.parameters["loss_coeffs"].items():
            data["loss_coeffs"][param] = value

        for param, value in self.parameters["scale_coeffs"].items():
            data["scale_coeffs"][param] = value

        for finger, finger_dict in self.parameters["retargeter_adjustments"].items():
            for param, array in finger_dict.items():
                for i, axis in enumerate(["x", "y", "z"]):
                    data["retargeter_adjustments"][finger][param][i] = array[i]

        with open(yaml_path, 'w') as file:
            yaml.dump(data, file)

        self.get_logger().warning('File saved.')



class ParameterTuningGUI(QMainWindow):
    def __init__(self, node, parameters):
        super().__init__()
        self.node = node
        self.parameters = parameters
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('ROS2 Parameter Tuning')
        central_widget = QWidget()
        outer_layout = QVBoxLayout()
        main_layout = QHBoxLayout()

        # Create two vertical layouts for the two lists
        loss_coeffs_layout = QVBoxLayout()
        scale_coeffs_layout = QVBoxLayout()
        mano_adjust_layout = QVBoxLayout()

        # Loss Coefficients
        title = QLabel("Loss Coefficients")
        loss_coeffs_layout.addWidget(title)

        for finger, default_value in self.parameters["loss_coeffs"].items():
            param_layout = QHBoxLayout()

            label = QLabel(f"{finger}:")
            param_layout.addWidget(label)

            input_field = QLineEdit(self)
            input_field.setMaximumWidth(50)
            input_field.setText(f"{default_value:.2f}")

            param_layout.addWidget(input_field)

            minus_button = QPushButton("-")
            minus_button.clicked.connect(lambda _, p=finger, f=input_field: self.update_param("loss_coeffs", p, f, -1))
            minus_button.setMaximumWidth(20)
            param_layout.addWidget(minus_button)

            plus_button = QPushButton("+")
            plus_button.clicked.connect(lambda _, p=finger, f=input_field: self.update_param("loss_coeffs", p, f, 1))
            plus_button.setMaximumWidth(20)
            param_layout.addWidget(plus_button)

            loss_coeffs_layout.addLayout(param_layout)

        # Scale Coefficients
        title = QLabel("Scale Coefficients")
        scale_coeffs_layout.addWidget(title)

        for finger, default_value in self.parameters["scale_coeffs"].items():
            param_layout = QHBoxLayout()

            label = QLabel(f"{finger}:")
            param_layout.addWidget(label)

            input_field = QLineEdit(self)
            input_field.setMaximumWidth(50)
            input_field.setText(f"{default_value:.2f}")

            param_layout.addWidget(input_field)

            minus_button = QPushButton("-")
            minus_button.clicked.connect(lambda _, p=finger, f=input_field: self.update_param("scale_coeffs", p, f, -0.1))
            minus_button.setMaximumWidth(20)
            param_layout.addWidget(minus_button)

            plus_button = QPushButton("+")
            plus_button.clicked.connect(lambda _, p=finger, f=input_field: self.update_param("scale_coeffs", p, f, 0.1))
            plus_button.setMaximumWidth(20)
            param_layout.addWidget(plus_button)

            scale_coeffs_layout.addLayout(param_layout)

        title = QLabel("Mano Adjustments")
        mano_adjust_layout.addWidget(title)

        for finger, finger_dict in self.parameters["retargeter_adjustments"].items():
            finger_layout = QHBoxLayout()
            finger_title = QLabel(f"{finger.upper()}")
            mano_adjust_layout.addWidget(finger_title)
            for param, array in finger_dict.items():
                param_title = QLabel(f"{param}")
                finger_layout.addWidget(param_title)
                for i, axis in enumerate(["x", "y", "z"]):
                    default_value = array[i]
                    label = QLabel(f"{axis}:")

                    param_layout = QVBoxLayout()

                    param_layout.addWidget(label)

                    input_field = QLineEdit(self)
                    input_field.setMaximumWidth(50)
                    input_field.setText(f"{default_value:.2f}")

                    param_layout.addWidget(input_field)

                    minus_button = QPushButton("-")
                    minus_button.setMaximumWidth(50)
                    minus_button.clicked.connect(lambda _, p=finger, n=param, a=axis, f=input_field: self.update_mano_adjust(p, n, a, f, -0.1))
                    param_layout.addWidget(minus_button)

                    plus_button = QPushButton("+")
                    plus_button.setMaximumWidth(50)
                    plus_button.clicked.connect(lambda _, p=finger, n=param, a=axis, f=input_field: self.update_mano_adjust(p, n, a, f, 0.1))
                    param_layout.addWidget(plus_button)

                    finger_layout.addLayout(param_layout)

            mano_adjust_layout.addLayout(finger_layout)

        # Add the two vertical layouts to the main horizontal layout
        main_layout.addLayout(loss_coeffs_layout)
        main_layout.addLayout(scale_coeffs_layout)
        main_layout.addLayout(mano_adjust_layout)

        # Add a save button
        save_button = QPushButton("Save")
        save_button.clicked.connect(self.save)

        outer_layout.addLayout(main_layout)
        outer_layout.addWidget(save_button)

        central_widget.setLayout(outer_layout)

        font = central_widget.font()
        font.setPointSize(10)
        central_widget.setFont(font)


        self.setCentralWidget(central_widget)

    def update_param(self, param_name, keyvector, input_field, delta):
        current_value = float(input_field.text())
        new_value = round(current_value + delta, 2)  # Increment/Decrement and round to 2 decimals
        input_field.setText(f"{new_value:.2f}")
        self.node.set_parameter_value(param_name, keyvector, new_value)

    def update_mano_adjust(self, finger, param_name, axis, input_field, delta):
        current_value = float(input_field.text())
        new_value = round(current_value + delta, 2)
        input_field.setText(f"{new_value:.2f}")
        self.node.set_mano_value(finger, param_name, axis, new_value)

    def save(self):
        self.node.save()

def main(args=None):
    rclpy.init(args=args)
    parameters = RETARGETER_PARAMS[configuration]
    node = ParameterAdjuster(parameters)

    app = QApplication(sys.argv)
    gui = ParameterTuningGUI(node, parameters)
    gui.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.1))
    timer.start(100)  # Trigger every 100 ms

    sys.exit(app.exec_())

    rclpy.shutdown()

if __name__ == '__main__':
    main()
