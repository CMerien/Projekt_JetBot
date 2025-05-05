#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry  # Import pre odometriu
import sys, select, termios, tty

class PIDController:
    def __init__(self, kp, ki, kd, max_acceleration):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_acceleration = max_acceleration
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_output = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0

        self.integral += error * dt
        max_integral = 1.0
        self.integral = max(-max_integral, min(self.integral, max_integral))
	
	# PID rovnica        
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
	
	# Limit zrychlenia
        delta_output = output - self.last_output
        output = self.last_output + max(-self.max_acceleration, min(delta_output, self.max_acceleration))
        self.last_output = output

        return output


class AutoDrive:
    def __init__(self):
        rospy.init_node('auto_drive_pid', anonymous=True)
        
        # Konfigurácia parametrov
        self.max_speed = rospy.get_param("~max_speed", 0.8)
        self.max_acceleration = rospy.get_param("~max_acceleration", 0.1)
        self.emergency_stop_distance = 0.15

        # Inicializácia stavových premenných
        self.distance = float('inf')
        self.stop_distance = None  # Potvrdená cieľová vzdialenosť
        self.running = False
        self.current_speed = 0.0  # Aktuálna rýchlosť z /odom_raw

        # Režim pre asynchrónny vstup novej cieľovej vzdialenosti
        self.target_input_mode = False
        self.input_buffer = ""

        # Režim pre výber typu regulátora
        self.reg_input_mode = False
        self.reg_input_buffer = ""
        self.regulator_type = "PID"  # Predvolene PID

        # Definícia parametrov regulátorov na jednom mieste
        self.regulator_params = {
	#    	     kp	   ki	  kd
            "PID": (1.11,  0.003, 0.028),
            "PI":  (1.1,  0.001,  0.0),
            "P":   (1.1,  0.0,   0.0)
        }

	
	# Teminalovy setup
        self.original_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Inicializácia PID regulátora s predvolenými parametrami načítanými zo slovníka
        kp, ki, kd = self.regulator_params[self.regulator_type]
        self.pid = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            max_acceleration=self.max_acceleration
        )

	# Meranie casu pre dynemicke dt
	self.last_time = rospy.Time.now()


        # ROS komunikácia
        rospy.Subscriber("/ultrasonic_range", Float32, self.sensor_callback)
        rospy.Subscriber("/odom_raw", Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.log_pub = rospy.Publisher("/auto_drive_log", Float32MultiArray, queue_size=10)

    def set_regulator_params(self, choice):
        """Nastavenie parametrov regulátora podľa voľby používateľa."""
        if choice == 1:
            chosen = "PID"
        elif choice == 2:
            chosen = "PI"
        elif choice == 3:
            chosen = "P"
        else:
            print("\nCHYBA: Neplatná voľba regulátora!")
            return
        
        self.regulator_type = chosen
        
        # Načítanie hodnôt z centralizovaného slovníka
        kp, ki, kd = self.regulator_params[chosen]
        self.pid.kp = kp
        self.pid.ki = ki
        self.pid.kd = kd
        
        # Reset integrálnych a derivačných hodnôt pre hladký prechod
        self.pid.integral = 0.0
        self.pid.prev_error = 0.0
        self.pid.last_output = 0.0
	
	# Reser pozadovanej vzdialenosti a zastavenie pohybu pri zmene regulatora
	self.stop_distance = 0.0
	self.running = False
 
        print("\nRegulátor nastavený na: %s" % self.regulator_type)


    def display_interface(self):
        """Vykreslenie ovládacieho rozhrania."""
        sys.stdout.write("\033[H\033[J")  # Vyčistiť obrazovku
        print("=== OVLÁDANIE ROBOTA ===")
        print("Aktuálny regulátor: %s" % self.regulator_type)
        if self.reg_input_mode:
            print("Vyberte typ regulátora:")
            print("1: PID    2: PI    3: P")
            print("Vstup: %s" % self.reg_input_buffer)
        elif self.target_input_mode:
            print("Nový cieľ")
            print("ZADAJTE CIEĽOVÚ VZDIALENOSŤ (v metroch):")
            print("Aktuálny vstup: %s" % self.input_buffer)
        else:
            print("n: Nový cieľ")
            print("s: Okamžité zastavenie")
            print("r: Zmena typu regulátora")
            print("q: Ukončenie programu")
        print("========================")
        print("Aktuálna rýchlosť: %.2f m/s" % self.current_speed)
        print("Aktuálna vzdialenosť: %.2f m" % self.distance)
        if not self.target_input_mode and not self.reg_input_mode:
            print("Cieľová vzdialenosť: %s" % (
                "%.2f m" % self.stop_distance if self.stop_distance is not None else "nenastavená"
            ))
        print("Stav: %s\n" % ("POHYB" if self.running else "STOJ"))

    def start_new_target_input(self):
        self.running = False
	self.cmd_pub.publish(Twist())
	self.last_time = rospy.Time.now()
	self.target_input_mode = True
        self.input_buffer = ""
        sys.stdout.write("\nZADAJTE CIEĽOVÚ VZDIALENOSŤ (v metroch):\n> ")
        sys.stdout.flush()

    def start_regulator_input(self):
        self.reg_input_mode = True
        self.reg_input_buffer = ""
        sys.stdout.write("\nVyberte typ regulátora (1: PID, 2: PI, 3: P):\n> ")
        sys.stdout.flush()

    def process_new_target_input(self, key):
        if key in ['\r', '\n']:
            if self.input_buffer:
                try:
                    new_target = float(self.input_buffer)
                    if new_target <= self.emergency_stop_distance:
                        new_target = self.emergency_stop_distance + 0.05
                        print("\nUPOZORNENIE: Cieľ nastavený na minimum (%.2f m)" % new_target)
                    self.stop_distance = new_target
                    self.running = True
		# Resetovanie PID a Casovej referencie
		    self.pid.integral = 0.0
		    self.pid.prev_error = 0.0
		    self.pid.last_output = 0.0
	 	    self.last_time = rospy.Time.now()
                    print("\nSPUSTENÉ! Robot sa pohybuje k cieľu: %.2f m" % self.stop_distance)
                except Exception as e:
                    print("\nCHYBA: Neplatný formát čísla!")
                    self.stop_distance = None
            self.target_input_mode = False
            self.input_buffer = ""
        elif key == '\x7f':
            if self.input_buffer:
                self.input_buffer = self.input_buffer[:-1]
                sys.stdout.write("\b \b")
                sys.stdout.flush()
        elif key and key in '0123456789.':
            self.input_buffer += key
            sys.stdout.write(key)
            sys.stdout.flush()

    def process_regulator_input(self, key):
        if key in ['\r', '\n']:
            if self.reg_input_buffer:
                try:
                    choice = int(self.reg_input_buffer)
                    self.set_regulator_params(choice)
                except Exception as e:
                    print("\nCHYBA: Neplatná voľba!")
            self.reg_input_mode = False
            self.reg_input_buffer = ""
        elif key == '\x7f':
            if self.reg_input_buffer:
                self.reg_input_buffer = self.reg_input_buffer[:-1]
                sys.stdout.write("\b \b")
                sys.stdout.flush()
        elif key and key in '123':
            self.reg_input_buffer += key
            sys.stdout.write(key)
            sys.stdout.flush()

    def get_key(self):
        if select.select([sys.stdin], [], [], 0.1)[0]:
            return sys.stdin.read(1)
        return None

    def sensor_callback(self, data):
        self.distance = data.data / 100.0  # Konverzia cm -> m
        
        if self.running and self.distance <= self.emergency_stop_distance:
            self.stop_robot()
            print("\nNÚDZOVÉ ZASTAVENIE! Vzdialenosť pod 15 cm!")

    def odom_callback(self, data):
        vx = data.twist.twist.linear.x
        vy = data.twist.twist.linear.y
        speed = (vx**2 + vy**2)**0.5
        if vx < 0:
            speed = -speed
        self.current_speed = speed

    def compute_speed(self):
	now = rospy.Time.now()
	dt = (now - self.last_time).to_sec()
	self.last_time = now

        error = self.distance - self.stop_distance
        raw_speed = self.pid.compute(error, dt)
	speed = max(-self.max_speed, min(raw_speed, self.max_speed))
        return speed

    def stop_robot(self):
        self.running = False
	self.stop_distance = 0.0
	self.stop_distance = 0.0
        self.pid.integral = 0.0
        self.pid.prev_error = 0.0
        self.pid.last_output = 0.0
	self.last_time = rospy.Time.now()
        self.cmd_pub.publish(Twist())
        print("\nZASTAVENÉ! Aktuálna vzdialenosť: %.2f m" % self.distance)

    def publish_log(self, speed):
        log_msg = Float32MultiArray()
        target_value = self.stop_distance if self.stop_distance is not None else 0.0

	regulator_mapping = {"PID": 1.0, "PI": 2.0, "P": 3.0}
	regulator_code = regulator_mapping.get(self.regulator_type, 0.0) 	
	
	# Log pole: [cas, poz. vzdialenost, akt. vzdialenost, rychlost z regulatora, rychlost z odometrie, ktory regulator sa vyuziva]
        log_msg.data = [
            time.time(), 
            target_value, 
            self.distance, 
            speed if self.running else 0.0,
            self.current_speed,
	    regulator_code
        ]
        self.log_pub.publish(log_msg)



    def main_loop(self):
        try:
            while not rospy.is_shutdown():
                self.display_interface()
                key = self.get_key()
                
                if self.reg_input_mode:
                    if key is not None:
                        self.process_regulator_input(key)
                elif self.target_input_mode:
                    if key is not None:
                        self.process_new_target_input(key)
                else:
                    if key == 'n':
                        self.start_new_target_input()
                    elif key == 's':
                        self.stop_robot()
                    elif key == 'r':
                        self.start_regulator_input()
                    elif key == 'q':
                        break

                    if self.running:
                        speed = self.compute_speed()
                        move_cmd = Twist()
                        move_cmd.linear.x = speed
                        self.cmd_pub.publish(move_cmd)
                    else:
                        speed = 0.0

                self.publish_log(speed if self.running else 0.0)
                rospy.sleep(0.1)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_settings)
            self.stop_robot()

if __name__ == '__main__':
    try:
        driver = AutoDrive()
        driver.main_loop()
    except rospy.ROSInterruptException:
        pass

