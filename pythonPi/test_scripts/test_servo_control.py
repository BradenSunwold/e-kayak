from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import time

# Use pigpio backend for precise PWM
factory = PiGPIOFactory()

# Create servo object on GPIO12
# The servo expects ~500–2500us in microseconds,
# so min_pulse_width and max_pulse_width set accordingly:
servo = AngularServo(
    12,
    min_angle=0,
    max_angle=180,
    min_pulse_width=0.0005,  # 500us
    max_pulse_width=0.0025,  # 2500us
    pin_factory=factory,
    initial_angle=None
)

def move_servo(angle: float):
    """Move the servo to an angle between 0 and 180."""
    if angle < 0 or angle > 180:
        raise ValueError("Angle must be between 0 and 180")
    print(f"Moving to {angle}°")
    servo.angle = angle
    time.sleep(1)  # give time to move

def main():
    try:
        while True:
            user = input("Enter angle (0 to 180, or 'q' to quit): ")
            if user.lower() == 'q':
                break
            try:
                angle = float(user)
                move_servo(angle)
            except ValueError:
                print("Invalid input — enter a number 0–180 or 'q'")
    except KeyboardInterrupt:
        print("Exiting.")
    finally:
        # Set to neutral or stop
        #servo.detach()
        print("exiting")

if __name__ == "__main__":
    main()
