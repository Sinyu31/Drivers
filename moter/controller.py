#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -*- author:       -*-
# -*- date:         -*-

from enum import Enum
from typing import Final
import time
import utils
import numpy as np
import pigpio

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

HIGH: Final[int] = 1
LOW:  Final[int] = 0
PWM_RANGE: Final[int] = utils.MAX_PWM_VALUE
FREQUENCY: Final[int] = 60


class MotorGpio(Enum):
# front motor
    #RF
    RightFrontIn1 = 18
    RightFrontIn2 = 17
    #LF
    LeftFrontIn3  = 23
    LeftFrontIn4  = 22

# behind motor: Unimplemented
    #RB
    RightBehindIn1 = -1
    RightBehindIn2 = -1
    #LB
    LeftBehindIn3  = -1
    LeftBehindIn4  = -1


    # return to [LF, RF, LB, RB]
    @classmethod
    def ToArray(cls) -> np.array:
        return np.array([
            [MotorGpio.LeftFrontIn3.value  , MotorGpio.LeftFrontIn4.value  ],  # Left Front (LF)
            [MotorGpio.RightFrontIn1.value , MotorGpio.RightFrontIn2.value ],  # Right Front (RF)
            [MotorGpio.LeftBehindIn3.value , MotorGpio.LeftBehindIn4.value ],  # Left Back (LB)
            [MotorGpio.RightBehindIn1.value, MotorGpio.RightBehindIn2.value]   # Right Back (RB)
        ])


"""
    Array representing motor directions for each movement type.
    Each row corresponds to a movement command (e.g. forward, backward, etc.),
    and each value in the row indicates the rotation of a specific wheel:
    1 = forward, -1 = backward, 0 = stop.
    Order of wheels: [Front-Left, Front-Right, Back-Left, Back-Right]
    reference: memo/movements.png
"""
DIRECTIONS: Final[np.array] = np.array(
   #[LF, RF, LB, RB]

    # forward,
    [ 1,  1,  1,  1], 
    # backward
    [-1, -1, -1, -1], 
    #right,
    [ 1, -1, -1,  1], 
     #left,
    [-1,  1,  1, -1],
    #right forward
    [ 1,  0,  0,  1], 
    #left forward
    [ 0,  1,  1,  0], 
    #right backward 
    [ 0, -1, -1,  0], 
    #left backward
    [-1,  0,  0, -1], 
    #turn right
    [ 1, -1,  1, -1], 
    #turn left
    [-1,  1, -1,  1], 
    #stop
    [ 0,  0,  0,  0] 
)


class WheelController:
    def __init__(self, pi: pigpio.pi) -> None:
        if not pi.connected:
            raise RuntimeError("Can not connect to pigpio deamon")
        
        self.raspberryPi_ = pi
        self.__Setup()

    
    def __Setup(self) -> None:
        self.__SetGpioPinAsOutput()
        self.__InitPwmFrequencyAndRange()

    """
        Sets all GPIO pins defined in MotorGpio as output,
    """
    def __SetGpioPinAsOutput(self) -> None:
        for gpio in MotorGpio:
            try:
                self.raspberryPi_.set_mode(gpio.value, pigpio.OUTPUT)
            except Exception as ex:
                print(f"Failed to setup GPIO {gpio.name}: {ex}")
                raise
    

    def __InitPwmFrequencyAndRange(self):
        try:
            for pin in MotorGpio:
                self.raspberryPi_.set_PWM_frequency(pin.value, FREQUENCY)
                self.raspberryPi_.set_PWM_range(pin.value, PWM_RANGE)

        except Exception as ex:
            print(f"Failed to setup frequency/range at pin {pin.name} : {ex} ")
            raise


    def __SetMotorDirection(self, pwm: int, direction: np.array) -> None:
        for idx, (pin1, pin2) in enumerate(MotorGpio.ToArray()):
            dor: int = direction[idx]

            if dor == 1:
                self.raspberryPi_.write(pin1, HIGH)
                self.raspberryPi_.write(pin2, LOW)

            elif dor == -1:
                self.raspberryPi_.write(pin1, LOW)
                self.raspberryPi_.write(pin2, HIGH)
                
            else:
                self.raspberryPi_.write(pin1, LOW)
                self.raspberryPi_.write(pin2, LOW)

            self.raspberryPi_.set_PWM_dutycycle(pin1, pwm)
            self.raspberryPi_.set_PWM_dutycycle(pin2, pwm)


    def Forward(self, speed: float) -> None:
        pwm: int = utils.ToPwm(speed)
        self.__SetMotorDirection(pwm, DIRECTIONS[0])


    def Backward(self, speed: float) -> None: 
        pwm: int = utils.ToPwm(speed) 
        self.__SetMotorDirection(pwm, DIRECTIONS[1])


    def Right(self, speed: float) -> None:
        pwm: int = utils.ToPwm(speed)
        self.__SetMotorDirection(pwm, DIRECTIONS[2])
        

    def Left(self, speed: float) -> None:
        pwm: int = utils.ToPwm(speed)
        self.__SetMotorDirection(pwm, DIRECTIONS[3])
        

    def RightForward(self, speed: float) -> None:
        pwm: int = utils.ToPwm(speed)
        self.__SetMotorDirection(pwm, DIRECTIONS[4])


    def LeftForward(self, speed: float) -> None:
        pwm: int = utils.ToPwm(speed)
        self.__SetMotorDirection(pwm, DIRECTIONS[5])


    def RightBackward(self, speed: float) -> None:
        pwm: int = utils.ToPwm(speed)
        self.__SetMotorDirection(pwm, DIRECTIONS[6])


    def LeftBackward(self, speed: float) -> None:
        pwm: int = utils.ToPwm(speed)
        self.__SetMotorDirection(pwm, DIRECTIONS[7])


    def TurnRight(self, speed: float) -> None:
        pwm: int = utils.ToPwm(speed)
        self.__SetMotorDirection(pwm, DIRECTIONS[8])        

    def TurnLeft(self, speed: float) -> None:
        pwm: int = utils.ToPwm(speed)
        self.__SetMotorDirection(pwm, DIRECTIONS[9])


    def Stop(self) -> None:
        pwm: int = 0
        self.__SetMotorDirection(pwm, DIRECTIONS[10])

    