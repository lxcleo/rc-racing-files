#!/usr/bin/env python
import rospy
import math
import time
import numpy
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int8
from sensor_msgs.msg import Joy
from time import sleep
import pigpio

# This ROS Node converts inputs from the receiver
# into commands for the RC car.


class reader:
    """
    A class to read PWM pulses and calculate their frequency
    and duty cycle.  The frequency is how often the pulse
    happens per second.  The duty cycle is the percentage of
    pulse high time per cycle.
    """

    def __init__(self, pi, gpio):
        """
        Instantiate with the Pi and gpio of the PWM signal
        to monitor.

        Optionally a weighting may be specified.  This is a number
        between 0 and 1 and indicates how much the old reading
        affects the new reading.  It defaults to 0 which means
        the old reading has no effect.  This may be used to
        smooth the data.
        """
        self.pi = pi
        self.gpio = gpio

        self._new = 1.0
        self._old = 0.0

        self._high_tick = None
        self._period = None
        self._high = None
        self._jitter = 0.025

        pi.set_mode(gpio, pigpio.INPUT)

        self._cb = pi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)

    def _cbf(self, gpio, level, tick):

        if level == 1:

            if self._high_tick is not None:
                t = pigpio.tickDiff(self._high_tick, tick)

                if self._period is not None:
                    self._period = (self._old * self._period) + (self._new * t)
                else:
                    self._period = t

            self._high_tick = tick

        elif level == 0:

            if self._high_tick is not None:
                t = pigpio.tickDiff(self._high_tick, tick)

                if self._high is not None:
                    self._high = (self._old * self._high) + (self._new * t)
                else:
                    self._high = t

    def frequency(self):
        """
        Returns the PWM frequency.
        """
        if self._period is not None:
            return 1000000.0 / self._period
        else:
            return 0.0

    def pulse_width(self):
        """
        Returns the PWM pulse width in microseconds.
        """
        if self._high is not None:
            return self._high
        else:
            return 0.0

    def duty_cycle(self):
        """
        Returns the PWM duty cycle percentage.
        """
        if self._high is not None and self._period is not None:
            return 100.0 * self._high / self._period
        else:
            return 0.0

    def cancel(self):
        """
        Cancels the reader and releases resources.
        """
        self._cb.cancel()


def talker():
    pub = rospy.Publisher('receiver', String, queue_size=1)
    pi1 = pigpio.pi()
    pi2 = pigpio.pi()
    pi3 = pigpio.pi()
    PWM_GPIO_SERVO = 18
    PWM_GPIO_ESC = 13
    PWM_GPIO_SWITCH = 7
    servo = rc_receiver.reader(pi1, PWM_GPIO_SERVO)
    esc = rc_receiver.reader(pi2, PWM_GPIO_ESC)
    r = rospy.Rate(90)
    while not rospy.is_shutdown():
        servo_signal = servo.duty_cycle()
        esc_signal = esc.duty_cycle()
        pub.publish(
            "steering: " +
            str(servo_signal) +
            " throttle: " +
            str(esc_signal)
            + " switch:" +
            str(12345678))
        r.sleep()


if __name__ == '__main__':
    import rc_receiver
    rospy.init_node('rc_receiver')
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
