import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as plt
import numpy as np


class UltrasonicImager:

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    max_dist = 30  # cm

    def __init__(self, servo_pin=32, transistor_pins=None, frequency=50, steps=30):
        if transistor_pins is None:
            transistor_pins = [11, 13, 15, 19, 21, 23]
        self.servo_pin = servo_pin
        self.transistor_pins = transistor_pins
        self.freq = frequency  # Hz
        self.steps = steps

    def setup_servo(self):
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.servo = GPIO.PWM(self.servo_pin, self.freq)
        self.servo.start(0)
        time.sleep(1)

    def servo_move(self, angle):
        # Assumes 5% = 0°, 10% = 180°.
        if not 0 <= angle <= 180:
            raise ValueError("Angle must be between 0 and 180 degrees.")
        duty = 5 + (angle / 180) * 5
        self.servo.ChangeDutyCycle(duty)
        time.sleep(0.5)
        self.servo.ChangeDutyCycle(0)

    def servo_zero(self):
        print("Turning back to 0 degrees")
        self.servo.ChangeDutyCycle(2)
        time.sleep(0.5)
        self.servo.ChangeDutyCycle(0)
        self.servo.stop()
        GPIO.cleanup()
        print("Servo At Zero")

    def rotate_read(self):
        self.setup_servo()
        print("Rotating 180 degrees in", self.steps, " steps")
        # empty array for the distance from the transducers to be placed in for each angle
        d_lists = [[], [], [], [], [], []]
        angle_list = np.linspace(0, 180, self.steps)
        for angle in angle_list:
            self.servo_move(angle)

            new_distances = []
            for pin in self.transistor_pins:
                new_distances.append(read_distance(pin))
            for d_list, distance in zip(d_lists, new_distances):
                d_list.append(distance)

            time.sleep(0.5)
        time.sleep(2)
        self.servo_zero()
        readings = d_lists + [angle_list]
        self.plot_2d(readings)
        self.plot_3d(readings)

    def plot_2d(self, data):
        readings = data[:6]
        n_t = len(readings)
        angle_axis = data[-1]
        n_angle = len(angle_axis)
        t_axis = np.linspace(1, n_t, n_t)
        plot_data = np.zeros((n_t, n_angle))
        for i, a in enumerate(angle_axis):
            for j, t in enumerate(t_axis):
                if readings[j][i] <= self.max_dist:
                    plot_data[j][i] = readings[j][i]
                else:
                    plot_data[j][i] = None
        plt.figure(figsize=(6, 6))
        im = plt.imshow(plot_data, extent=(angle_axis[0], angle_axis[-1], t_axis[0], t_axis[-1]), origin='lower',
                        aspect='auto')
        plt.xlim(angle_axis[0], angle_axis[-1])
        plt.ylim(t_axis[0], t_axis[-1])
        plt.xlabel('Angle (degs)')
        plt.ylabel('Height (cm)')
        plt.title('Ultrasonic Image')
        cbar = plt.colorbar(im, orientation='vertical')
        cbar.set_label('Distance (cm)', fontsize=12)
        plt.show()

    def plot_3d(self, data):
        readings = data[:6]
        n_t = len(readings)
        angle_axis = data[-1]

        t_axis = np.linspace(1, n_t, n_t * 2)
        X = np.array(np.empty([n_t * 2, len(data[6])]))
        Y = np.array(np.empty([n_t * 2, len(data[6])]))
        for i in range(n_t):
            k = 2 * i + 1
            for j in range(len(data[0])):
                if data[i][j] >= self.max_dist:
                    X[k - 1][j], X[k][j] = None, None
                    Y[k - 1][j], Y[k][j] = None, None
                else:
                    X[k - 1][j] = np.sin(np.deg2rad(angle_axis[j] - 90)) * data[i][j]
                    X[k][j] = np.sin(np.deg2rad(angle_axis[j] - 90)) * data[i][j]
                    Y[k - 1][j] = np.cos(np.deg2rad(angle_axis[j] - 90)) * data[i][j]
                    Y[k][j] = np.cos(np.deg2rad(angle_axis[j] - 90)) * data[i][j]
        Z = np.array(len(angle_axis) * [t_axis]).transpose()
        ax = plt.axes(projection='3d')
        ax.plot_surface(X, Y, Z)
        ax.view_init(angle_axis[0], angle_axis[-1])
        ax = plt.gca()
        ax.set_ylim([0, self.max_dist])
        plt.show()


def setup_gpio(pin):
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)
    time.sleep(5e-6)
    GPIO.output(pin, 1)
    time.sleep(5e-6)
    GPIO.output(pin, 0)
    GPIO.setup(pin, GPIO.IN)


speed_of_sound = 34300


def read_distance(pin):
    setup_gpio(pin)
    while GPIO.input(pin) == 0:
        starttime = time.time()
    while GPIO.input(pin) == 1:
        endtime = time.time()
    duration = endtime - starttime
    distance = duration * speed_of_sound / 2
    time.sleep(0.02)
    return distance
