from pynput import keyboard
import pigpio
from controller import WheelController
from typing import Final

class RobotController:
    def __init__(self, pi: pigpio.pi, speed: Final[float] = 0.5):
        self.keys = set()
        self.speed = speed
        self.controller = WheelController(pi)

    def on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char is not None:
                self.keys.add(key.char)

            self.handle_keys()
        except Exception as ex:
            print(f"Error in on_press: {ex}")

    def on_release(self, key):
        try:
            if hasattr(key, 'char') and key.char is not None:
                self.keys.discard(key.char)

            if not self.keys:
                self.controller.Stop()

            if key == keyboard.Key.esc:
                self.controller.Stop()
                return False

        except Exception as ex:
            print(f"Error in on_release: {ex}")

    def handle_keys(self):
        movement = {
            ('w', 'd'): self.controller.RightForward,
            ('w', 'a'): self.controller.LeftForward,
            ('s', 'd'): self.controller.RightBackward,
            ('s', 'a'): self.controller.LeftBackward,
            ('w',): self.controller.Forward,
            ('s',): self.controller.Backward,
            ('a',): self.controller.Left,
            ('d',): self.controller.Right,
            ('q',): self.controller.TurnLeft,
            ('e',): self.controller.TurnRight,
            ('x',): self.controller.Stop,
        }

        for key_combination, action in movement.items():
            if key_combination.issubset(self.keys):
                action(self.speed)
                break

def main() -> None:
    try:
        pi = pigpio.pi()

        robot_controller = RobotController(pi)

        with keyboard.Listener(
            on_press=robot_controller.on_press,
            on_release=robot_controller.on_release
        ) as listener:
            listener.join()

    finally:
        pi.stop()

if __name__ == "__main__":
    main()
