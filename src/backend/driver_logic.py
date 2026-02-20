import time
import threading
import math
from .physical_motor import VirtualPhysicalMotor
from .motor_config import MOTOR_DEFAULTS

class MotorDriverLogic:
    def __init__(self):
    	# ... keep all your other __init__ variables the same ...
        self.running = False
        
        # --- ZLTECH PROPRIETARY 32-BIT ERROR CODE (0x603F) ---
        # High 16-bit = Right Motor | Low 16-bit = Left Motor
        self.zlac_error_code = 0x00000000
        self.lock = threading.Lock()
        
        # 1. Instantiate the Physical World (The "Muscle")
        self.motor_l = VirtualPhysicalMotor()
        self.motor_r = VirtualPhysicalMotor()
        
        # 2. CANopen Interface Variabless
        self.operating_mode = 3
        self.is_enabled = False
        self.running = False
        self.fault_code = 0x0000  # 0 = No Fault
        
        # Target Commands (Raw from CAN)
        self.target_position_left = 0.0;  self.target_position_right = 0.0
        self.target_velocity_left = 0.0;  self.target_velocity_right = 0.0
        self.target_torque_left = 0.0;    self.target_torque_right = 0.0
        
        # Output States (Raw to CAN)
        self.current_position_left = 0.0; self.current_position_right = 0.0
        self.current_velocity_left = 0.0; self.current_velocity_right = 0.0
        self.current_torque_left = 0.0;   self.current_torque_right = 0.0
        
        # S-Curve Kinematic Limits (Matches ZLTECH Factory Defaults)
        self.profile_vel_l = 120.0;  self.profile_vel_r = 120.0
        self.profile_acc_l = 500.0;  self.profile_acc_r = 500.0
        self.profile_dec_l = 500.0;  self.profile_dec_r = 500.0
        
        # Internal S-Curve Generator Tracking (In Radians)
        self.prof_pos_l_rad = 0.0; self.prof_pos_r_rad = 0.0
        self.prof_vel_l_rads = 0.0; self.prof_vel_r_rads = 0.0
        
        # PID Controller States
        self.integral_error_vel_l = 0.0; self.integral_error_vel_r = 0.0

    # ==========================================
    # --- CANOPEN SETTERS (From the Master) ---
    # ==========================================
    def set_mode(self, mode):
        with self.lock: 
            self.operating_mode = mode
            # --- FIX: Sync targets to physical reality when switching modes ---
            self.target_position_left = self.motor_l.get_encoder_counts()
            self.target_position_right = self.motor_r.get_encoder_counts()
            self.prof_pos_l_rad = self.motor_l.position_rad
            self.prof_pos_r_rad = self.motor_r.position_rad
    def set_enabled(self, enabled):
        with self.lock: 
            self.is_enabled = enabled
            if not enabled: # Reset PID integrals when disabled
                self.integral_error_vel_l = 0.0
                self.integral_error_vel_r = 0.0
                # Snap S-Curve to current physical reality to prevent jerking upon re-enable
                self.prof_pos_l_rad = self.motor_l.position_rad
                self.prof_pos_r_rad = self.motor_r.position_rad
                self.prof_vel_l_rads = self.motor_l.velocity_rads
                self.prof_vel_r_rads = self.motor_r.velocity_rads

    def update_velocity_target(self, left, right):
        with self.lock: self.target_velocity_left = left; self.target_velocity_right = right
    def update_position_target(self, left, right):
        with self.lock: self.target_position_left = left; self.target_position_right = right
    def update_torque_target(self, left, right):
        with self.lock: self.target_torque_left = left; self.target_torque_right = right
        
    def update_kinematic_profiles(self, p_vel_l, p_vel_r, p_acc_l, p_acc_r, p_dec_l, p_dec_r):
        with self.lock:
            self.profile_vel_l = p_vel_l; self.profile_acc_l = p_acc_l; self.profile_dec_l = p_dec_l
            self.profile_vel_r = p_vel_r; self.profile_acc_r = p_acc_r; self.profile_dec_r = p_dec_r

    # ==========================================
    # --- MATH HELPERS ---
    # ==========================================
    def _rpm_to_rads(self, rpm): return rpm * (2 * math.pi / 60.0)
    def _counts_to_rad(self, counts, cpr): return (counts / cpr) * (2 * math.pi)
    def _rad_to_counts(self, rad, cpr): return (rad / (2 * math.pi)) * cpr

    def _generate_s_curve_step(self, cur_pos, cur_vel, targ_pos, v_max, a_max, d_max, dt):
        """Mathematical Trajectory Generator (Now handles on-the-fly speed changes perfectly!)"""
        error = targ_pos - cur_pos
        direction = 1.0 if error > 0 else -1.0
        
        # Protect against divide-by-zero if deceleration is 0
        safe_d_max = max(0.1, d_max)
        braking_dist = (cur_vel**2) / (2.0 * safe_d_max)

        # 1. Have we arrived?
        if abs(error) <= 0.001 and abs(cur_vel) <= (safe_d_max * dt): 
            return targ_pos, 0.0
            
        # 2. Do we need to brake for the target, OR because the user suddenly lowered v_max?
        elif abs(error) <= braking_dist or abs(cur_vel) > v_max:
            # Smoothly apply deceleration limits
            cur_vel -= math.copysign(safe_d_max * dt, cur_vel)
            
            # Clamp to 0 if we were braking for the target
            if abs(error) <= braking_dist:
                if (direction == 1.0 and cur_vel < 0) or (direction == -1.0 and cur_vel > 0): 
                    cur_vel = 0.0
            # Clamp to the new v_max if we were braking because of a speed limit change
            elif abs(cur_vel) <= v_max:
                cur_vel = v_max * math.copysign(1.0, cur_vel)
                
        # 3. Otherwise, accelerate! (This is where it ramps up to your new 250 RPM)
        else:
            cur_vel += direction * a_max * dt
            if abs(cur_vel) > v_max: 
                cur_vel = v_max * direction

        # 4. Calculate new position
        new_pos = cur_pos + (cur_vel * dt)
        
        # Prevent overshooting the final target
        if (direction == 1.0 and new_pos > targ_pos) or (direction == -1.0 and new_pos < targ_pos):
            new_pos = targ_pos
            cur_vel = 0.0

        return new_pos, cur_vel

    def _run_cascaded_pid(self, motor, target_rad, target_rads, target_amps, dt, is_left):
        """The Microprocessor Brain: Position Error -> Velocity -> Current -> Voltage"""
        if not self.is_enabled or self.zlac_error_code != 0x00000000:
            motor.apply_physics_step(0.0, dt) # Cut power
            return

        # 1. Position P-Loop (Only active in Mode 1)
        if self.operating_mode == 1:
            Kp_pos = 10.0
            pos_error = target_rad - motor.position_rad
            vel_command = target_rads + (Kp_pos * pos_error)
        else:
            vel_command = target_rads

        # 2. Velocity PI-Loop (Active in Mode 1 & 3)
        if self.operating_mode in [1, 3]:
            Kp_vel = 0.8
            Ki_vel = 2.0
            vel_error = vel_command - motor.velocity_rads
            
            # Integrate Error
            if is_left:
                self.integral_error_vel_l += vel_error * dt
                current_command = (Kp_vel * vel_error) + (Ki_vel * self.integral_error_vel_l)
            else:
                self.integral_error_vel_r += vel_error * dt
                current_command = (Kp_vel * vel_error) + (Ki_vel * self.integral_error_vel_r)
        else:
            current_command = target_amps # Mode 4 (Torque Mode) directly commands current

        # 3. Hardware Limiter (Prevents blowing up the virtual motor)
        max_A = MOTOR_DEFAULTS["max_current_A"]
        if current_command > max_A: current_command = max_A
        elif current_command < -max_A: current_command = -max_A

        # 4. Voltage Feedforward (Sends final PWM voltage to the physical wires)
        # V = (I_target * Resistance) + Back-EMF
        voltage_out = (current_command * motor.R) + (motor.Ke * motor.velocity_rads)
        
        # Max Battery Voltage is 24V
        voltage_out = max(-24.0, min(24.0, voltage_out))

        # 5. Apply to Physical World!
        motor.apply_physics_step(voltage_out, dt)

    def _check_hardware_faults(self):
        """Monitors physical reality and trips official ZLAC8015D 0x603F Fault Codes"""
        max_amps = MOTOR_DEFAULTS["max_current_A"]
        max_temp = MOTOR_DEFAULTS["max_temp_celsius"]
        
        # Once an error trips, hardware latches it until the Master sends a Reset command (0x80)
        if self.zlac_error_code != 0x00000000:
            return

        # 1. OVERCURRENT (Left: 0x0000 0004h | Right: 0x0004 0000h)
        if abs(self.motor_l.current_A) >= max_amps + 0.5:
            self.zlac_error_code |= 0x00000004
            print(f"[ZLAC FAULT] Left Overcurrent! ({self.motor_l.current_A:.1f}A)")
        if abs(self.motor_r.current_A) >= max_amps + 0.5:
            self.zlac_error_code |= 0x00040000
            print(f"[ZLAC FAULT] Right Overcurrent! ({self.motor_r.current_A:.1f}A)")

        # 2. OVER-TEMPERATURE (Left: 0x0000 0400h | Right: 0x0400 0000h)
        if self.motor_l.temperature_C >= max_temp:
            self.zlac_error_code |= 0x00000400
            print("[ZLAC FAULT] Left Motor High Temp!")
        if self.motor_r.temperature_C >= max_temp:
            self.zlac_error_code |= 0x04000000
            print("[ZLAC FAULT] Right Motor High Temp!")
                
        # 3. ENCODER OUT OF TOLERANCE / FOLLOWING ERROR (Left: 0x0000 0020h | Right: 0x0020 0000h)
        cpr = MOTOR_DEFAULTS["counts_per_rev"]
        prof_counts_l = self._rad_to_counts(self.prof_pos_l_rad, cpr)
        prof_counts_r = self._rad_to_counts(self.prof_pos_r_rad, cpr)
        
        pos_err_l_counts = abs(self.motor_l.get_encoder_counts() - prof_counts_l)
        pos_err_r_counts = abs(self.motor_r.get_encoder_counts() - prof_counts_r)
        
        if self.operating_mode == 1 and self.is_enabled:
            if pos_err_l_counts > 5000:
                self.zlac_error_code |= 0x00000020
                print(f"[ZLAC FAULT] Left Following Error: {pos_err_l_counts} counts lag.")
            if pos_err_r_counts > 5000:
                self.zlac_error_code |= 0x00200000
                print(f"[ZLAC FAULT] Right Following Error: {pos_err_r_counts} counts lag.")
                
        # 4. SPEED SETTING ERROR (0x0000 2000h)
        # The manual states speed cannot exceed the rated speed 
        max_rpm = 205.0 # From your motor datasheet
        if self.operating_mode == 3:
            if abs(self.target_velocity_left) > max_rpm:
                self.zlac_error_code |= 0x00002000
                print("[ZLAC FAULT] Left Speed Setting Error!")
            if abs(self.target_velocity_right) > max_rpm:
                self.zlac_error_code |= 0x20000000
                print("[ZLAC FAULT] Right Speed Setting Error!")
    # ==========================================
    # --- MAIN THREAD LOOP ---
    # ==========================================
    def _driver_loop(self):
        last_time = time.time()
        cpr = MOTOR_DEFAULTS["counts_per_rev"]
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            with self.lock:
                # 1. ZLTECH Kinematic Limits (Software Clamping & Unit Conversion)
                # The physical motor tops out at ~205 RPM. We forbid the math brain from exceeding this.
                safe_vel_l = min(205.0, abs(self.profile_vel_l))
                safe_vel_r = min(205.0, abs(self.profile_vel_r))
                
                v_max_l = self._rpm_to_rads(safe_vel_l)
                v_max_r = self._rpm_to_rads(safe_vel_r)
                
                # ZLTECH Manual: 0x6083 and 0x6084 are Acceleration TIME in milliseconds!
                acc_time_l_sec = max(0.001, self.profile_acc_l / 1000.0)
                acc_time_r_sec = max(0.001, self.profile_acc_r / 1000.0)
                dec_time_l_sec = max(0.001, self.profile_dec_l / 1000.0)
                dec_time_r_sec = max(0.001, self.profile_dec_r / 1000.0)
                
                a_max_l = v_max_l / acc_time_l_sec
                a_max_r = v_max_r / acc_time_r_sec
                d_max_l = v_max_l / dec_time_l_sec
                d_max_r = v_max_r / dec_time_r_sec
                
                # 2. Run the Mathematical S-Curve Generator
                t_pos_rad_l = self._counts_to_rad(self.target_position_left, cpr)
                t_pos_rad_r = self._counts_to_rad(self.target_position_right, cpr)
                
                if self.operating_mode == 1:
                    # Notice we now correctly pass d_max_l and d_max_r for braking!
                    self.prof_pos_l_rad, self.prof_vel_l_rads = self._generate_s_curve_step(
                        self.prof_pos_l_rad, self.prof_vel_l_rads, t_pos_rad_l, v_max_l, a_max_l, d_max_l, dt)
                    self.prof_pos_r_rad, self.prof_vel_r_rads = self._generate_s_curve_step(
                        self.prof_pos_r_rad, self.prof_vel_r_rads, t_pos_rad_r, v_max_r, a_max_r, d_max_r, dt)
                
                # 3. Convert Targets for PID Loop
                t_vel_rads_l = self._rpm_to_rads(self.target_velocity_left) if self.operating_mode == 3 else self.prof_vel_l_rads
                t_vel_rads_r = self._rpm_to_rads(self.target_velocity_right) if self.operating_mode == 3 else self.prof_vel_r_rads
                
                t_amps_l = (self.target_torque_left / 1000.0) # Assume input is mA
                t_amps_r = (self.target_torque_right / 1000.0)

                # 4. Execute Cascaded PID & Apply Physics!
                self._run_cascaded_pid(self.motor_l, self.prof_pos_l_rad, t_vel_rads_l, t_amps_l, dt, is_left=True)
                self._run_cascaded_pid(self.motor_r, self.prof_pos_r_rad, t_vel_rads_r, t_amps_r, dt, is_left=False)

                # 5. Monitor Hardware for Smoke & Jamming
                self._check_hardware_faults()

                # 6. Read Physical Sensors back to CANopen registers
                self.current_position_left = self.motor_l.get_encoder_counts()
                self.current_position_right = self.motor_r.get_encoder_counts()
                self.current_velocity_left = self.motor_l.get_rpm() * 10.0
                self.current_velocity_right = self.motor_r.get_rpm() * 10.0
                self.current_torque_left = self.motor_l.current_A * 1000.0 # Output in mA
                self.current_torque_right = self.motor_r.current_A * 1000.0

            time.sleep(0.01) # 100Hz Hardware Loop

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._driver_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'): self.thread.join()
