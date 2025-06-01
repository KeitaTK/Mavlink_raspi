from pymavlink import mavutil
import time

def read_all_system_parameters():
    """システムパラメータの包括的確認"""
    
    print("=== System Parameter Check (Improved) ===")
    
    try:
        master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        master.wait_heartbeat()
        
        print(f"USB Connection - System ID: {master.target_system}")
        
        # パラメータリスト要求
        print("\nRequesting all parameters...")
        master.mav.param_request_list_send(
            master.target_system,
            master.target_component
        )
        
        # システム関連パラメータを収集
        system_params = {}
        start_time = time.time()
        
        while time.time() - start_time < 10:  # 10秒間収集
            msg = master.recv_match(type='PARAM_VALUE', blocking=False)
            if msg:
                try:
                    param_id = msg.param_id
                    if isinstance(param_id, bytes):
                        param_id = param_id.decode('utf-8').rstrip('\x00')
                    elif isinstance(param_id, str):
                        param_id = param_id.rstrip('\x00')
                    
                    # システム・シリアル関連パラメータを記録
                    if any(keyword in param_id for keyword in ['SYSID', 'SERIAL1', 'BRD_SER1']):
                        system_params[param_id] = msg.param_value
                        print(f"   {param_id} = {msg.param_value}")
                        
                except Exception as e:
                    pass
            
            time.sleep(0.01)
        
        # 重要パラメータの確認
        print(f"\n=== Critical Parameters ===")
        critical_params = ['SYSID_THISMAV', 'SERIAL1_PROTOCOL', 'SERIAL1_BAUD', 'BRD_SER1_RTSCTS']
        
        for param in critical_params:
            if param in system_params:
                value = system_params[param]
                status = "✅" if is_param_correct(param, value) else "⚠️"
                print(f"{status} {param} = {value}")
            else:
                print(f"❌ {param} = NOT FOUND")
        
        return system_params
        
    except Exception as e:
        print(f"Error: {e}")
        return {}
    finally:
        master.close()

def is_param_correct(param_name, value):
    """パラメータ値の正当性チェック"""
    correct_values = {
        'SYSID_THISMAV': lambda x: x >= 1,
        'SERIAL1_PROTOCOL': lambda x: x == 2,
        'SERIAL1_BAUD': lambda x: x == 115200,
        'BRD_SER1_RTSCTS': lambda x: x == 0
    }
    
    if param_name in correct_values:
        return correct_values[param_name](value)
    return True

if __name__ == "__main__":
    read_all_system_parameters()

