import threading
import math
from .motor_config import MOTOR_DEFAULTS

class VirtualPhysicalMotor:
    def __init__(self):
        self.lock = threading.Lock()
        
        # 1. Load Base Physical Properties from Datasheet
        self.J = MOTOR_DEFAULTS["rotor_inertia_kgm2"]
        self.Kt = MOTOR_DEFAULTS["torque_constant_Nm_per_A"]
        self.Ke = MOTOR_DEFAULTS["back_emf_constant_V_per_rads"]
        self.R = MOTOR_DEFAULTS["coil_resistance_ohms"]
        self.B = MOTOR_DEFAULTS["viscous_friction_Nms"]
        self.cpr = MOTOR_DEFAULTS["counts_per_rev"]

        # 2. Interactive "World" Variables (For Fault Injection API)
        self.load_torque_Nm = 0.0  # External resistance (mud, hills)
        self.is_jammed = False     # Simulates a mechanical lock-up

        # 3. Live Physical State
        self.position_rad = 0.0
        self.velocity_rads = 0.0
        self.current_A = 0.0
        self.temperature_C = 25.0  # Starts at room temperature

    # ==========================================
    # --- DYNAMIC SETTERS FOR THE GUI/API ---
    # ==========================================
    def set_load_torque(self, torque_Nm):
        """Simulate a hill, collision, or friction drag"""
        with self.lock:
            self.load_torque_Nm = torque_Nm

    def set_inertia(self, inertia_kgm2):
        """Simulate picking up or dropping the 150KG payload"""
        with self.lock:
            self.J = max(0.001, inertia_kgm2) 

    # ==========================================
    # --- OUTPUTS TO THE DRIVER ---
    # ==========================================
    def get_encoder_counts(self):
        """Converts physical radians into electrical encoder ticks (0-4095 scale)"""
        with self.lock:
            revolutions = self.position_rad / (2 * math.pi)
            return int(revolutions * self.cpr)
            
    def get_rpm(self):
        """Converts physical radians/sec to RPM"""
        with self.lock:
            return self.velocity_rads * (60.0 / (2 * math.pi))

    # ==========================================
    # --- THE PHYSICS ENGINE ---
    # ==========================================
    def apply_physics_step(self, applied_voltage_V, dt):
        """
        Takes Voltage from the Driver and calculates the physical result.
        """
        with self.lock:
            # --- SABOTAGE HOOK ---
            if self.is_jammed:
                self.velocity_rads = 0.0
                # If jammed, motor stalls and acts like a pure resistor (I = V/R)
                self.current_A = applied_voltage_V / self.R 
                return self.current_A, self.velocity_rads, self.temperature_C

            # 1. Electrical Equation: Calculate Current (Amps)
            back_emf = self.Ke * self.velocity_rads
            
            # Ohm's Law: I = (V - V_emf) / R
            self.current_A = (applied_voltage_V - back_emf) / self.R

            # 2. Magnetic Equation: Calculate raw Torque (Nm)
            motor_torque = self.Kt * self.current_A

            # 3. Mechanical Equation (F = ma): Calculate Acceleration
            # External load must ALWAYS oppose the direction of motion
            if self.velocity_rads != 0:
                opposing_load = math.copysign(self.load_torque_Nm, self.velocity_rads)
            else:
                opposing_load = self.load_torque_Nm

            net_torque = motor_torque - opposing_load - (self.B * self.velocity_rads)
            acceleration_rads2 = net_torque / self.J

            # 4. Integrate to find new physical reality
            self.velocity_rads += acceleration_rads2 * dt
            self.position_rad += self.velocity_rads * dt
            
            # 5. Thermal Equation (I^2R Heating)
            # Simulates the coils heating up if pushed past 6A rated current
            heating_rate = (self.current_A ** 2) * self.R * 0.005 
            cooling_rate = (self.temperature_C - 25.0) * 0.1      
            self.temperature_C += (heating_rate - cooling_rate) * dt

            return self.current_A, self.velocity_rads, self.temperature_C
