import RPi.GPIO as GPIO
import time

#interval
INTERVAL = 1
#GPIO
IR_DIGITAL_PIN = 2
IR_ANALOG_PIN = 4

GPIO.setmode(GPIO.BCM)
GPIO.cleanup()
#GPIO.setup(IR_DIGITAL_PIN, GPIO.IN)
GPIO.setup(IR_ANALOG_PIN, GPIO.IN)


if __name__  == '__main__':
    try:
        while True:
            #print(GPIO.input(IR_DIGITAL_PIN))
            print(GPIO.input(IR_ANALOG_PIN))
            time.sleep(INTERVAL)
    except KeyboardInterrupt:
        print("finishing...")
    finally:
        GPIO.cleanup()
        print("GPIO cleaned")