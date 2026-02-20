# Physical parameters derived from ZLLG65ASM250-4096 V2.0 Datasheet
MOTOR_DEFAULTS = {
    # Encoder parameters
    "counts_per_rev": 4096.0,            # 12-bit Magnetic Encoder
    
    # Mechanical parameters
    "rotor_inertia_kgm2": 0.014,         # J: Calculated from 3.75kg mass and 173mm diameter
    "viscous_friction_Nms": 0.01,        # B: Nominal bearing resistance
    
    # Electrical parameters
    "torque_constant_Nm_per_A": 1.0,     # Kt: 6 N.m / 6 A rated
    "back_emf_constant_V_per_rads": 1.0, # Ke: Numerically matches Kt in SI units
    "coil_resistance_ohms": 1.2,         # R: Calculated voltage drop at rated load
    
    # Safety Limits (Used by the Driver to trigger Faults)
    "max_current_A": 18.0,               # Peak current from datasheet
    "max_temp_celsius": 90.0             # Standard safety cutoff for BLDC coils
}
