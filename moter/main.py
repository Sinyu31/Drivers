#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -*- author:       -*-
# -*- date:         -*-

from enum import Enum
from time import sleep
import sys
#import pigpio

"""
@config
-front motors

 pin  | GPIO | purpose
---------------------
   8  |  18  | Right motor IN1
  10  |  17  | Right motor IN2
  12  |  23  | Left motor  IN3
  11  |  22  | Left motor  IN4      

-behind motors

 pin  | GPIO | purpose
---------------------
 TBD  | TBD  | Right motor IN1
 TBD  | TBD  | Right motor IN2
 TBD  | TBD  | Left motor  IN3
 TBD  | TBD  | Left motor  IN4  


wheel: MecanumWheel
type of direction of movement
(1) forward,       (2) backward,      (3) right,         (4) left,      
(5) right forward  (6) left forward   (7) right backward (8)left backward
(9) turn right     (10) turn left     (11) stop
"""

class MotorGpio(Enum):
# front motor
    RightFrontIn1 = 18
    RightFrontIn2 = 17
    LeftFrontIn3  = 23
    LeftFrontIn4  = 22

# behind motor: Unimplemented
    RightBehindIn1 = -1
    RightBehindIn2 = -1
    LeftBehindIn3  = -1
    LeftBehindIn4  = -1

class WheelController:
    def __init__(self, pi) -> None:
        if not pi.connected:
            raise RuntimeError("Can not connect to pigpio deamon")
        
        self.raspberryPi_ = pi
        self.__SetGpioPinAsOutPut()
    
    def Forward(self, speed: float):
        pwm = self.__ToPwm(speed)
        #TODO

    def Backward(self, speed: float):
        pwm = self.__ToPwm(speed)
        #TODO


    def Right(self, speed: float):
        pwm = self.__ToPwm(speed)
        #TODO

    def Left(self, speed: float):
        pwm = self.__ToPwm(speed)
        #TODO

    def RightForward(self, speed: float):
        pwm = self.__ToPwm(speed)
        #TODO

    def LeftForward(self, speed: float):
        pwm = self.__ToPwm(speed)
        #TODO

    def RightBackward(self, speed: float):
        pwm = self.__ToPwm(speed)
        #TODO

    def LeftBackward(self, speed: float):
        pwm = self.__ToPwm(speed)
        #TODO

    def TurnRight(self, speed: float):
        pwm = self.__ToPwm(speed)
        #TODO

    def TurnLeft(self, speed: float):
        pwm = self.__ToPwm(speed)
        #TODO

    def Stop(self):
        pass
        #TODO

    """
        Maps speed from the range -1 to 1 (which is the range of Twist messages) 
        to the range 0 to maxPwmThreshold.
        :param speed: The speed of the robot (-1 to 1, corresponding to Twist message range)
        :param maxPwmThreshold: The maximum PWM signal strength (default is 255)
        :return: The PWM signal (0 to maxPwmThreshold)
    """
    def __ToPwm(self, speed: float, maxPwmThreshold: int = 255) -> int:
        if speed < -1 or speed > 1:
            speed = self.__Clamp(speed, -1, 1)
        
        pwmValue = int((speed + 1) / 2 * maxPwmThreshold)

        return pwmValue
    
    """ 
        Clamps the value to the range [minValue, maxValue].
        :param value: The value to clamp.
        :param minValue: The minimum allowed value.
        :param maxValue: The maximum allowed value.
        :return: The clamped value.
    """
    def __Clamp(self, value: float, minValue: float, maxValue: float) -> float:
        return max(minValue, min(value, maxValue))
    
    """
        Sets all GPIO pins defined in MotorGpio as output,
        except the ones with value -1.
    """
    def __SetGpioPinAsOutPut(self) -> None:
        for gpio in MotorGpio:
            if gpio.value != -1:
                self.raspberryPi_.set_mode(gpio.value, pigpio.OUTPUT)