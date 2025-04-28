from controller import WheelController
from pynput import keyboard
import pigpio
from typing import Final

# 同時押しのキーをトラッキングするセット
keys = set()
speed: Final[float] = 0.5


def on_press(key):
    try:
        if hasattr(key, 'char') and key.char is not None:
            keys.add(key.char)

        # 同時押しに対応した動作
        if 'w' in keys and 'd' in keys:  # w + d → 右前進
            controller.RightForward(speed)
        elif 'w' in keys and 'a' in keys:  # w + a → 左前進
            controller.LeftForward(speed)
        elif 's' in keys and 'd' in keys:  # s + d → 右後進
            controller.RightBackward(speed)
        elif 's' in keys and 'a' in keys:  # s + a → 左後進
            controller.LeftBackward(speed)
        elif 'w' in keys:  # w → 前進
            controller.Forward(speed)
        elif 's' in keys:  # s → 後退
            controller.Backward(speed)
        elif 'a' in keys:  # a → 左
            controller.Left(speed)
        elif 'd' in keys:  # d → 右
            controller.Right(speed)
        elif 'q' in keys:  # q → 左回転
            controller.TurnLeft(speed)
        elif 'e' in keys:  # e → 右回転
            controller.TurnRight(speed)
        elif 'x' in keys:  # x → 停止
            controller.Stop()

    except AttributeError:
        pass


def on_release(key):
    try:
        if hasattr(key, 'char') and key.char is not None:
            keys.discard(key.char)

        if not keys:
            controller.Stop()

        if key == keyboard.Key.esc:
            controller.Stop()
            return False

    except AttributeError:
        pass


def main() -> None:
    try:
        pi = pigpio.pi()

        global controller
        controller = WheelController(pi)

        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

    finally:
        pi.stop()


if __name__ == "__main__":
    main()
