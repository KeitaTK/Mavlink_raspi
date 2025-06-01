def set_gps_origin_for_indoor():
    """屋内用GPS原点設定"""
    
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    
    # 任意の原点設定（東京周辺の例）
    lat = 35.6895  # 緯度
    lon = 139.6917 # 経度  
    alt = 50       # 高度[m]
    
    master.mav.set_gps_global_origin_send(
        master.target_system,
        int(lat * 10000000),   # 緯度×10^7
        int(lon * 10000000),   # 経度×10^7
        int(alt * 1000)        # 高度[mm]
    )
    
    print(f"GPS origin set: {lat}, {lon}, {alt}m")
    master.close()
