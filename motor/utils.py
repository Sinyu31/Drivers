from typing import Final

MAX_PWM_VALUE: Final[int] = 255

"""
    Maps speed from the range 0 to 1 (which is the range of Twist messages) 
    to the range 0 to maxPwmThreshold.
    :param speed: The speed of the robot (0 to 1, corresponding to Twist message range)
    :param maxPwmThreshold: The maximum PWM signal strength (default is 255)
    :return: The PWM signal (0 to maxPwmThreshold)
"""
def ToPwm(speed: float, maxPwmThreshold: int = MAX_PWM_VALUE) -> int:
    if speed < 0 or speed > 1:
        speed = Clamp(speed)

    pwmValue = int(speed * maxPwmThreshold)

    return pwmValue
    
""" 
    Clamps the value to the range [minValue, maxValue].
    :param value: The value to clamp.
    :param minValue: The minimum allowed value.
    :param maxValue: The maximum allowed value.
    :return: The clamped value.
"""
def Clamp(value: float, minValue: float = 0, maxValue: float = 1) -> float:
        return max(minValue, min(value, maxValue))
    
