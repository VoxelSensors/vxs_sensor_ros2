import math

def calculate_lidar_hits(distance_m, 
                         wire_diameter_mm=2.5, 
                         wire_length_m=1.0, 
                         points_per_sec=200000, 
                         h_fov_deg=360, 
                         v_fov_deg=59):
    """
    Calculates the expected number of points per second hitting a wire.
    
    Formula: f(R) = (N * A_target) / A_fov(R)
    A_fov = R^2 * (h_fov_rad) * 2 * sin(v_fov_rad / 2)
    """
    
    # Constants
    d = wire_diameter_mm / 1000.0
    L = wire_length_m
    N = points_per_sec
    R = distance_m
    
    # Convert angles to radians
    h_fov_rad = math.radians(h_fov_deg)
    v_fov_rad = math.radians(v_fov_deg)
    
    # Area of a spherical segment wedge at distance R
    # Area = R^2 * (Horizontal_Radians) * (2 * sin(Vertical_Radians / 2))
    scan_area_at_R = (R**2) * h_fov_rad * (2 * math.sin(v_fov_rad / 2))
    
    # Target Area (Silhouette)
    target_area = d * L
    
    # Calculation
    pts_per_sec = (N * target_area) / scan_area_at_R
    
    return pts_per_sec


def calculate_lyra_hits(distance_m, 
                         wire_diameter_mm=2.5, 
                         wire_length_m=1.0, 
                         points_per_sec=5000000, 
                         fov_deg=50):
        
    """
    Calculates the expected number of points per second hitting a wire.
    
    Formula: f(R) = (N * A_target) / A_fov(R)
    A_fov = R^2 * (h_fov_rad) * 2 * sin(v_fov_rad / 2)
    """
    
    # Constants
    d = wire_diameter_mm / 1000.0
    L = wire_length_m
    N = points_per_sec
    R = distance_m
    
    # Convert angles to radians
    fov_rad = math.radians(fov_deg)
    
    # Area of a spherical segment wedge at distance R
    # Area = R^2 * (Horizontal_Radians) * (2 * sin(Vertical_Radians / 2))
    scan_area_at_R = 4 * R**2 * math.tan(fov_rad/2)**2
    
    # Target Area (Silhouette)
    target_area = d * L

    # Calculation
    pts_per_sec = (N * target_area) / scan_area_at_R
    
    return pts_per_sec


distances = [0.5, 1.0, 2.0, 3.0]
widths_cm = [0.75, 1.5]

print(f"{'Distance (m)':<15} | {'Width (cm)':<12} | {'Lyra (pts/s)':<15} | {'360-LiDAR (pts/s)':<15}")
print("-" * 65)

for width in widths_cm:
    for dist in distances:
        # Width in table is cm, function takes mm
        w_mm = width * 10 
        
        lyra_val = calculate_lyra_hits(dist, wire_diameter_mm=w_mm)
        lidar_val = calculate_lidar_hits(dist, wire_diameter_mm=w_mm)
        
        print(f"{dist:<15} | {width:<12} | {lyra_val:<15.2f} | {lidar_val:<15.2f}")