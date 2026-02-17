import time
import threading

class MotorPhysics:
    def __init__(self):
        # Physics state variables (Units: 0.1 r/min to match 0x606C)
        self.current_velocity_left = 0.0
        self.current_velocity_right = 0.0
        
        # Target variables
        self.target_velocity_left = 0.0
        self.target_velocity_right = 0.0
        
        # Parameters (Default 500ms accel/decel as per manual)
        self.accel_time_ms = 500.0
        self.decel_time_ms = 500.0
        
        # System state reference
        self.is_enabled = False
        
        # Thread control
        self.running = False
        self.lock = threading.Lock()

    def update_parameters(self, target_l, target_r, accel, decel, is_enabled):
        """Thread-safe update of targets from the CAN bus"""
        with self.lock:
            self.target_velocity_left = target_l
            self.target_velocity_right = target_r
            self.accel_time_ms = accel if accel > 0 else 1.0 # Prevent div by 0
            self.decel_time_ms = decel if decel > 0 else 1.0
            self.is_enabled = is_enabled

    def _calculate_step(self, current, target, delta_time_s):
        """Calculates the new velocity for a single time step"""
        # If motor is disabled, force target to 0 (freewheel/stop)
        actual_target = target if self.is_enabled else 0.0
        
        if current == actual_target:
            return current
            
        # Determine if we are accelerating or decelerating
        # Decelerating means moving towards 0. Accelerating means moving away from 0.
        is_decel = (actual_target == 0) or (abs(actual_target) < abs(current)) or (current * actual_target < 0)
        
        time_param = self.decel_time_ms if is_decel else self.accel_time_ms
        
        # Standard ZLAC assumes accel time is from 0 to 1000 RPM (10000 in 0.1 RPM units)
        # Rate = Max_Speed / Time_to_Max
        rate_per_sec = 10000.0 / (time_param / 1000.0) 
        
        step_change = rate_per_sec * delta_time_s
        
        if current < actual_target:
            current = min(current + step_change, actual_target)
        elif current > actual_target:
            current = max(current - step_change, actual_target)
            
        return current

    def _physics_loop(self):
        last_time = time.time()
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            with self.lock:
                self.current_velocity_left = self._calculate_step(
                    self.current_velocity_left, self.target_velocity_left, dt)
                    
                self.current_velocity_right = self._calculate_step(
                    self.current_velocity_right, self.target_velocity_right, dt)
                    
            time.sleep(0.01) # 100Hz physics loop

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._physics_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
