def precision_control_accurate_2cm(master, start_position):
    """高精度2cm移動制御"""
    print("\n" + "="*60)
    print("HIGH PRECISION 2CM CONTROL MODE")
    print("="*60)
    print("Controls:")
    print("  ↑ : South (-2cm)     | ↓ : North (+2cm)")
    print("  ← : East (+2cm)      | → : West (-2cm)")
    print("  w : Up (+2cm)        | x : Down (-2cm)")
    print("  s : Status           | h : Hold position")
    print("  r : Return origin    | q : Quit")
    print("EXACT movement distance: 2.00cm per key press")
    print("="*60)
    
    # 初期化
    lat0_deg = start_position.lat / 1e7
    lon0_deg = start_position.lon / 1e7
    alt0_msl = start_position.alt / 1000.0
    
    # 累積移動量（メートル単位）
    total_north = 0.0
    total_east = 0.0
    current_altitude = TAKEOFF_ALTITUDE
    
    # 初期ヨー角記録
    initial_yaw = get_current_yaw(master)
    print(f"Initial position: {lat0_deg:.7f}, {lon0_deg:.7f}")
    print(f"Initial yaw angle: {initial_yaw:.1f}° (FIXED)")
    print("Ready for EXACT 2cm control...")
    
    while True:
        key = get_key()
        moved = False
        altitude_changed = False
        
        if key == 'q':
            print("Exiting precision control")
            break
        elif key == 's':
            # 詳細状態表示
            print("\n--- DETAILED STATUS ---")
            status = get_current_status(master)
            
            # 現在の理論位置
            current_lat, current_lon = simple_ned_to_gps(lat0_deg, lon0_deg, total_north, total_east)
            
            print(f"Theoretical Position:")
            print(f"  North offset: {total_north*100:+.1f}cm")
            print(f"  East offset:  {total_east*100:+.1f}cm")
            print(f"  Target GPS:   {current_lat:.7f}, {current_lon:.7f}")
            
            if 'lat' in status:
                print(f"Actual GPS:     {status['lat']:.7f}, {status['lon']:.7f}")
                
                # 実際の移動距離計算
                actual_north = (status['lat'] - lat0_deg) * 111111.0
                actual_east = (status['lon'] - lon0_deg) * 111111.0 * math.cos(math.radians(lat0_deg))
                
                print(f"Actual Position:")
                print(f"  North offset: {actual_north*100:+.1f}cm")
                print(f"  East offset:  {actual_east*100:+.1f}cm")
                
                # 誤差計算
                north_error = abs(actual_north - total_north) * 100
                east_error = abs(actual_east - total_east) * 100
                print(f"Position Error: N={north_error:.1f}cm, E={east_error:.1f}cm")
            
            if 'altitude' in status:
                print(f"Target Altitude: {current_altitude:.3f}m")
                print(f"Actual Altitude: {status['altitude']:.3f}m")
                alt_error = abs(status['altitude'] - current_altitude) * 100
                print(f"Altitude Error:  {alt_error:.1f}cm")
            
            print("--- END ---\n")
            continue
        elif key == 'h':
            print("→ Holding current position")
            moved = True
        elif key == 'r':
            print("→ Returning to origin")
            total_north = 0.0
            total_east = 0.0
            current_altitude = TAKEOFF_ALTITUDE
            moved = True
            altitude_changed = True
        
        # 水平移動（EXACT 2cm）
        elif key == 'up':  # 南へ移動
            total_north -= 0.02  # EXACTLY 2cm
            moved = True
            print(f"↓ South EXACTLY 2.0cm (total: {total_north*100:+.1f}cm)")
        elif key == 'down':  # 北へ移動
            total_north += 0.02  # EXACTLY 2cm
            moved = True
            print(f"↑ North EXACTLY 2.0cm (total: {total_north*100:+.1f}cm)")
        elif key == 'right':  # 西へ移動
            total_east -= 0.02  # EXACTLY 2cm
            moved = True
            print(f"→ West EXACTLY 2.0cm (total: {total_east*100:+.1f}cm)")
        elif key == 'left':  # 東へ移動
            total_east += 0.02  # EXACTLY 2cm
            moved = True
            print(f"← East EXACTLY 2.0cm (total: {total_east*100:+.1f}cm)")
        
        # 高度制御
        elif key == 'w':  # 上昇
            current_altitude += 0.02  # EXACTLY 2cm
            altitude_changed = True
            moved = True
            print(f"↑ Up EXACTLY 2.0cm (altitude: {current_altitude:.3f}m)")
        elif key == 'x':  # 下降
            if current_altitude - 0.02 >= 0.05:
                current_altitude -= 0.02  # EXACTLY 2cm
                altitude_changed = True
                moved = True
                print(f"↓ Down EXACTLY 2.0cm (altitude: {current_altitude:.3f}m)")
            else:
                print("⚠ Minimum altitude limit (5cm)")
        
        if moved:
            # 高精度GPS座標計算
            target_lat, target_lon = simple_ned_to_gps(lat0_deg, lon0_deg, total_north, total_east)
            
            # degE7形式に変換（精度保持）
            target_lat_int = int(round(target_lat * 1e7))
            target_lon_int = int(round(target_lon * 1e7))
            
            print(f"  Target GPS: {target_lat:.7f}, {target_lon:.7f}")
            print(f"  degE7: {target_lat_int}, {target_lon_int}")
            
            # 移動コマンド送信
            move_to_position_with_fixed_yaw(master, target_lat_int, target_lon_int, 
                                          current_altitude, initial_yaw)
            
            if altitude_changed:
                print(f"  Target altitude: {current_altitude:.3f}m")
            
            time.sleep(0.1)

def simple_ned_to_gps(lat0_deg, lon0_deg, north_m, east_m):
    """シンプルで正確なNED→GPS変換"""
    # WGS84楕円体での1度あたりの距離（より正確）
    lat_m_per_deg = 111132.92 - 559.82 * math.cos(2 * math.radians(lat0_deg)) + 1.175 * math.cos(4 * math.radians(lat0_deg))
    lon_m_per_deg = 111412.84 * math.cos(math.radians(lat0_deg)) - 93.5 * math.cos(3 * math.radians(lat0_deg))
    
    # 緯度経度変化計算
    lat_change = north_m / lat_m_per_deg
    lon_change = east_m / lon_m_per_deg
    
    new_lat = lat0_deg + lat_change
    new_lon = lon0_deg + lon_change
    
    return new_lat, new_lon
