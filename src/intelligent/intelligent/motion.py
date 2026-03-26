import math

def MoveToOdom(x_target, y_target, x_input, y_input, theta_input, threshold):
    """
    Navigasi waypoint untuk robot Ackerman (forward-only, tidak bisa belok di tempat).
    
    Args:
        x_target, y_target: Koordinat target
        x_input, y_input: Posisi robot saat ini (dari odom)
        theta_input: Yaw robot saat ini (radian)
        threshold: Jarak toleransi untuk dianggap "sampai"
    
    Returns:
        motion: 1 = maju, 0 = berhenti
        theta_output: Command angular (radian/s) untuk ROS
        is_done: True jika sudah sampai waypoint
    """
    motion = 0
    theta_output = 0.0
    is_done = False
    
    # 1. Hitung jarak ke target
    dx = x_target - x_input
    dy = y_target - y_input
    dist_to_goal = math.hypot(dx, dy)
    
    # 2. Cek apakah sudah sampai
    if dist_to_goal < threshold:
        motion = 0
        theta_output = 0.0
        is_done = True
        return motion, theta_output, is_done
    
    # 3. Hitung sudut target (yaw yang diinginkan)
    target_yaw = math.atan2(dy, dx)
    
    # 4. Hitung error sudut (ternormalisasi [-pi, pi])
    angle_error = target_yaw - theta_input
    angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
    
    # 5. Ackerman: SELALU MAJU (tidak bisa belok di tempat)
    motion = 1  # Selalu jalan selama belum sampai
    
    # 6. Kontrol angular proporsional terhadap error
    # Error besar = belok tajam, error kecil = lurus
    angular_gain = 0.8
    theta_output = angular_gain * angle_error
    
    # Clamp angular speed (ackerman punya batas belok maksimal)
    theta_output = max(-0.5, min(0.5, theta_output))
    
    return int(motion), float(theta_output), bool(is_done)