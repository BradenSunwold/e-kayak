from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import time

# Use pigpio backend for precise PWM
factory = PiGPIOFactory()

servo = AngularServo(
    12,
    min_angle=0,
    max_angle=180,
    min_pulse_width=0.0005,  # 500us
    max_pulse_width=0.0025,  # 2500us
    pin_factory=factory,
    initial_angle=None  # Do NOT move on startup
)

LOW_ANGLE = 65
HIGH_ANGLE = 116
DELAY = 5  # seconds


def main():
    try:
        print("Oscillating between 60° and 150° every 5 seconds.")
        current_angle = LOW_ANGLE

        while True:
            print(f"Moving to {current_angle}°")
            servo.angle = current_angle

            time.sleep(DELAY)

            # Toggle angle
            if current_angle == LOW_ANGLE:
                current_angle = HIGH_ANGLE
            else:
                current_angle = LOW_ANGLE

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        # Do NOT detach — leave servo holding position
        print("Leaving servo at last position.")


if __name__ == "__main__":
    main()
