import time
import threading

class MotorPhysics:
    def __init__(self):
        self.operating_mode = 3 
        
        # Kinematic State (Internal Math uses Counts and Counts/Sec for perfection)
        self.counts_per_rev = 10000.0
        
        self.pos_l = 0.0 # Counts
        self.pos_r = 0.0
        self.vel_l_cps = 0.0 # Counts per second
        self.vel_r_cps = 0.0
        
        # Target State
        self.target_position_left = 0.0
        self.target_position_right = 0.0
        self.target_velocity_left = 0.0 # 0.1 RPM
        self.target_velocity_right = 0.0
        self.target_torque_left = 0.0
        self.target_torque_right = 0.0
        
        # Current Output State (For CAN/GUI)
        self.current_velocity_left = 0.0 # 0.1 RPM
        self.current_velocity_right = 0.0
        self.current_position_left = 0.0 # Counts
        self.current_position_right = 0.0
        self.current_torque_left = 0.0
        self.current_torque_right = 0.0
        
        # Kinematic Profile Limits
        self.profile_vel_l = 1000.0 # 0.1 RPM (Default 100 RPM)
        self.profile_vel_r = 1000.0
        self.profile_acc_l = 200.0  # RPM/s
        self.profile_acc_r = 200.0
        self.profile_dec_l = 200.0  # RPM/s
        self.profile_dec_r = 200.0

        self.is_enabled = False
        self.running = False
        self.lock = threading.Lock()

    def set_mode(self, mode):
        with self.lock: self.operating_mode = mode

    def update_velocity_target(self, left, right):
        with self.lock:
            self.target_velocity_left = left
            self.target_velocity_right = right

    def update_position_target(self, left, right):
        with self.lock:
            self.target_position_left = left
            self.target_position_right = right
            
    def update_torque_target(self, left, right):
        with self.lock:
            self.target_torque_left = left
            self.target_torque_right = right
            
    def update_kinematic_profiles(self, p_vel_l, p_vel_r, p_acc_l, p_acc_r, p_dec_l, p_dec_r):
        with self.lock:
            self.profile_vel_l = p_vel_l if p_vel_l > 0 else 1000.0
            self.profile_vel_r = p_vel_r if p_vel_r > 0 else 1000.0
            self.profile_acc_l = p_acc_l if p_acc_l > 0 else 200.0
            self.profile_acc_r = p_acc_r if p_acc_r > 0 else 200.0
            self.profile_dec_l = p_dec_l if p_dec_l > 0 else 200.0
            self.profile_dec_r = p_dec_r if p_dec_r > 0 else 200.0

    def set_enabled(self, enabled):
        with self.lock: self.is_enabled = enabled

    def _rpm_to_cps(self, rpm_01):
        """Convert 0.1 RPM to Counts Per Second"""
        return (rpm_01 / 10.0 / 60.0) * self.counts_per_rev

    def _cps_to_rpm(self, cps):
        """Convert Counts Per Second to 0.1 RPM"""
        return (cps / self.counts_per_rev) * 60.0 * 10.0

    def _calculate_axis_kinematics(self, current_pos, current_vel_cps, target_pos, prof_vel_01, prof_acc, prof_dec, dt):
        """Perfect Trapezoidal Trajectory Generator"""
        
        # 1. Convert profile limits to internal Counts/Second
        v_max = self._rpm_to_cps(prof_vel_01)
        a_max = self._rpm_to_cps(prof_acc * 10.0) # Assume accel is RPM/s, convert to 0.1 RPM/s then CPS
        d_max = self._rpm_to_cps(prof_dec * 10.0)

        error = target_pos - current_pos
        direction = 1.0 if error > 0 else -1.0
        abs_error = abs(error)

        # 2. Calculate exact braking distance: d = v^2 / 2a
        braking_distance = (current_vel_cps ** 2) / (2.0 * d_max)

        # 3. Decision Tree: Brake or Accelerate?
        if abs_error <= 1.0 and abs(current_vel_cps) <= (d_max * dt):
            # Target Reached - Snap to exactly 0 to prevent micro-oscillations
            return target_pos, 0.0
            
        elif abs_error <= braking_distance:
            # We are inside the braking zone! Slam on the brakes.
            current_vel_cps -= direction * d_max * dt
            # Prevent reversing direction due to over-braking
            if (direction == 1.0 and current_vel_cps < 0) or (direction == -1.0 and current_vel_cps > 0):
                current_vel_cps = 0.0
        else:
            # We are outside the braking zone! Accelerate to cruising speed.
            current_vel_cps += direction * a_max * dt
            if abs(current_vel_cps) > v_max:
                current_vel_cps = v_max * direction

        # 4. Integrate velocity into position
        new_pos = current_pos + (current_vel_cps * dt)
        
        # Prevent stepping past the target in the final frame
        if (direction == 1.0 and new_pos > target_pos) or (direction == -1.0 and new_pos < target_pos):
            new_pos = target_pos
            current_vel_cps = 0.0

        return new_pos, current_vel_cps

    def _physics_loop(self):
        last_time = time.time()
        trq_ramp_rate = 1000.0  # mA per second (how fast torque applies)
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            with self.lock:
                if not self.is_enabled:
                    # Emergency Deceleration (using decel profile)
                    dec_l_cps = self._rpm_to_cps(self.profile_dec_l * 10.0)
                    dec_r_cps = self._rpm_to_cps(self.profile_dec_r * 10.0)
                    
                    if self.vel_l_cps > 0: self.vel_l_cps = max(0, self.vel_l_cps - dec_l_cps * dt)
                    elif self.vel_l_cps < 0: self.vel_l_cps = min(0, self.vel_l_cps + dec_l_cps * dt)
                    
                    if self.vel_r_cps > 0: self.vel_r_cps = max(0, self.vel_r_cps - dec_r_cps * dt)
                    elif self.vel_r_cps < 0: self.vel_r_cps = min(0, self.vel_r_cps + dec_r_cps * dt)
                    
                    self.pos_l += self.vel_l_cps * dt
                    self.pos_r += self.vel_r_cps * dt

                    # Drop torque to 0 safely when disabled
                    if self.current_torque_left > 0: self.current_torque_left = max(0, self.current_torque_left - trq_ramp_rate * dt)
                    elif self.current_torque_left < 0: self.current_torque_left = min(0, self.current_torque_left + trq_ramp_rate * dt)
                    
                    if self.current_torque_right > 0: self.current_torque_right = max(0, self.current_torque_right - trq_ramp_rate * dt)
                    elif self.current_torque_right < 0: self.current_torque_right = min(0, self.current_torque_right + trq_ramp_rate * dt)

                else:
                    if self.operating_mode == 3: # Velocity Mode
                        v_targ_l_cps = self._rpm_to_cps(self.target_velocity_left)
                        a_l_cps = self._rpm_to_cps(self.profile_acc_l * 10.0)
                        if self.vel_l_cps < v_targ_l_cps: self.vel_l_cps = min(self.vel_l_cps + a_l_cps * dt, v_targ_l_cps)
                        elif self.vel_l_cps > v_targ_l_cps: self.vel_l_cps = max(self.vel_l_cps - a_l_cps * dt, v_targ_l_cps)
                        
                        v_targ_r_cps = self._rpm_to_cps(self.target_velocity_right)
                        a_r_cps = self._rpm_to_cps(self.profile_acc_r * 10.0)
                        if self.vel_r_cps < v_targ_r_cps: self.vel_r_cps = min(self.vel_r_cps + a_r_cps * dt, v_targ_r_cps)
                        elif self.vel_r_cps > v_targ_r_cps: self.vel_r_cps = max(self.vel_r_cps - a_r_cps * dt, v_targ_r_cps)
                        
                        self.pos_l += self.vel_l_cps * dt
                        self.pos_r += self.vel_r_cps * dt

                    elif self.operating_mode == 1: # Kinematic Position Mode!
                        self.pos_l, self.vel_l_cps = self._calculate_axis_kinematics(
                            self.pos_l, self.vel_l_cps, self.target_position_left, 
                            self.profile_vel_l, self.profile_acc_l, self.profile_dec_l, dt)
                            
                        self.pos_r, self.vel_r_cps = self._calculate_axis_kinematics(
                            self.pos_r, self.vel_r_cps, self.target_position_right, 
                            self.profile_vel_r, self.profile_acc_r, self.profile_dec_r, dt)

                    # --- ADDED TORQUE MODE BACK IN ---
                    elif self.operating_mode == 4: # Torque Mode!
                        # Ramp Left Torque
                        if self.current_torque_left < self.target_torque_left:
                            self.current_torque_left = min(self.current_torque_left + trq_ramp_rate * dt, self.target_torque_left)
                        elif self.current_torque_left > self.target_torque_left:
                            self.current_torque_left = max(self.current_torque_left - trq_ramp_rate * dt, self.target_torque_left)
                            
                        # Ramp Right Torque
                        if self.current_torque_right < self.target_torque_right:
                            self.current_torque_right = min(self.current_torque_right + trq_ramp_rate * dt, self.target_torque_right)
                        elif self.current_torque_right > self.target_torque_right:
                            self.current_torque_right = max(self.current_torque_right - trq_ramp_rate * dt, self.target_torque_right)

                # Export states back to CANopen format
                self.current_position_left = self.pos_l
                self.current_position_right = self.pos_r
                self.current_velocity_left = self._cps_to_rpm(self.vel_l_cps)
                self.current_velocity_right = self._cps_to_rpm(self.vel_r_cps)

            time.sleep(0.01) # 100Hz Integration
            
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._physics_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
