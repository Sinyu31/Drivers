import numpy as np
from numpy.typing import NDArray
from typing import Final
from enum import Enum

class MotorGpio(Enum):
# front motor
    #RF
    RightFrontIn1 = 18
    RightFrontIn2 = 17
    #LF
    LeftFrontIn3  = 23
    LeftFrontIn4  = 22

    #RB
    RightBehindIn1 = 13
    RightBehindIn2 = 12
    #LB
    LeftBehindIn3  = 6
    LeftBehindIn4  = 5


    # return to [LF, RF, LB, RB]
    @classmethod
    def ToArray(cls) -> NDArray[np.int32]:
        return np.array([
            [MotorGpio.LeftFrontIn3.value  , MotorGpio.LeftFrontIn4.value  ],  # Left Front (LF)
            [MotorGpio.RightFrontIn1.value , MotorGpio.RightFrontIn2.value ],  # Right Front (RF)
            [MotorGpio.LeftBehindIn3.value , MotorGpio.LeftBehindIn4.value ],  # Left Back (LB)
            [MotorGpio.RightBehindIn1.value, MotorGpio.RightBehindIn2.value]   # Right Back (RB)
        ],  dtype=np.int32)




DIRECTIONS: Final[NDArray[np.int32]] = np.array([
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
        [ 0,  0,  0,  0],

    ], dtype=np.int32
)


def main() -> None:
    for i, (p1, p2) in enumerate(MotorGpio.ToArray()):
        print(f"{p1}, {p2} : dir is {DIRECTIONS[i]}")
        print()

if __name__ == "__main__":
    main()