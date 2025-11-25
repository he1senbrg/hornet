from __future__ import division
import time
import Adafruit_PCA9685

# --- Initialize PCA9685 with busnum=1 ---
try:
    pwm = Adafruit_PCA9685.PCA9685(busnum=1)
    pwm.set_pwm_freq(60)
except Exception as e:
    print(f"Error initializing PCA9685: {e}")
    print("Make sure I2C is enabled and the board is connected.")
    exit(1)


def set_servo_angle(channel, angle):
    """
    Sets the servo to a specific angle.
    Handles inversion for Leg 2 and Leg 3.
    """
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180

    # Leg 2 uses channels 3, 4, 5
    # Leg 3 uses channels 6, 7, 8
    # You requested all channels on these legs be inverted.
    if channel in [3, 4, 5, 6, 7, 8]:
        pulse = int((angle * -2.5) + 600)
    else:
        # Standard pulse calculation for Legs 1 and 4
        pulse = int((angle * 2.5) + 150)

    pwm.set_pwm(channel, 0, pulse)


def wiggle_servo(channel):
    """
    Moves the servo from 90 -> 120 -> 90 to show it is active.
    """
    print(f"  > Moving Channel {channel} to 90...")
    set_servo_angle(channel, 90)
    time.sleep(0.3)

    print(f"  > Moving Channel {channel} to 120...")
    set_servo_angle(channel, 120)
    time.sleep(0.3)

    print(f"  > Moving Channel {channel} back to 90...")
    set_servo_angle(channel, 90)
    time.sleep(0.3)


if __name__ == "__main__":
    print("--- Sequential Servo Tester ---")
    print("This script will iterate through channels 0 to 11.")
    print("Leg 2 (Ch 3-5) and Leg 3 (Ch 6-8) are configured to move INVERTED.")
    print("Press ENTER to test the current channel.")
    print("Press 'q' then ENTER to quit early.")
    print("-----------------------------------")

    try:
        for i in range(12):
            user_input = input(
                f"\n[Channel {i}] Ready? Press ENTER to move (or 'q' to quit): "
            )

            if user_input.lower().strip() == "q":
                print("Quitting...")
                break

            wiggle_servo(i)

        print("\nDone testing all 12 channels.")

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")

    finally:
        pass
