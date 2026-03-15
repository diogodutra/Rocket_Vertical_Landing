def compute_theoretical_gains(rocket,
                              wn_att=4.0, zeta_att=0.8,   # Faster, well-damped inner loop
                              wn_alt=1.0, zeta_alt=1.0,   # Critically damped altitude
                              wn_pos=0.7, zeta_pos=1.2):  # Large separation from attitude
    """
    Computes theoretical PID/PD gains based on linearized 2nd-order dynamics.
    
    Args:
        rocket: An instance of the Rocket class (for m, J, l, g).
        wn_att, zeta_att: Desired frequency/damping for the Attitude loop.
        wn_alt, zeta_alt: Desired frequency/damping for the Altitude loop.
        wn_pos: Desired frequency for the Position (outer) loop.
        
    Returns:
        dict: A dictionary containing sets of Kp, Ki, Kd for each loop.
    """
    # Altitude Gains (Vertical)
    # Based on m*z_ddot + Kd*z_dot + Kp*z = 0
    kp_alt = rocket.m * (wn_alt**2)
    kd_alt = 2 * zeta_alt * wn_alt * rocket.m
    ki_alt = 0.0
    
    # Attitude Gains (Inner Loop)
    # Based on J*theta_ddot + (T*l*Kd)*theta_dot + (T*l*Kp)*theta = 0
    # We assume a nominal thrust (T) equal to hover weight (m*g)
    T_hover = rocket.m * rocket.g
    
    kp_att = -(rocket.J * (wn_att**2)) / (T_hover * rocket.l)
    kd_att = -(2 * zeta_att * wn_att * rocket.J) / (T_hover * rocket.l)
    ki_att = 0.0
    
    # Position Gains (Outer Loop)
    # Based on x_ddot = g * theta -> Kp_pos = wn^2 / g
    ki_pos = 0.0
    kp_pos = (wn_pos**2) / rocket.g
    kd_pos = (2 * zeta_pos * wn_pos) / rocket.g
    
    return {
        'altitude': {'kp': kp_alt, 'ki': ki_alt, 'kd': kd_alt},
        'attitude': {'kp': kp_att, 'ki': ki_att, 'kd': kd_att},
        'position': {'kp': kp_pos, 'ki': ki_pos, 'kd': kd_pos}
    }

if __name__ == "__main__":
    from src.model.rocket import Rocket
    rocket = Rocket()
    gains = compute_theoretical_gains(rocket)

    print("-" * 30)
    print(" GNC THEORETICAL GAINS OUTPUT")
    print("-" * 30)
    
    for loop, values in gains.items():
        print(f"\n[{loop.upper()} LOOP]")
        print(f"  Kp: {values['kp']:10.4f}")
        print(f"  Kd: {values['kd']:10.4f}")
        print(f"  Ki: {values['ki']:10.4f}")
    
    print("\n" + "-" * 30)
    print("Use these values in GNCController")