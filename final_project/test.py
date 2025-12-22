import socketio
import time
import sys

# 建立 Socket.IO 客戶端
sio = socketio.Client()

# 伺服器設定 (如果是跑在同一台電腦，IP 用 localhost 即可)
SERVER_IP = "http://192.168.50.105:5000/" 

@sio.event
def connect():
    print(f"成功連線至伺服器: {SERVER_IP}")

@sio.event
def disconnect():
    print("與伺服器斷開連線")

def simulate_attack(label):
    """模擬發送特定招式的攻擊"""
    print(f"正在模擬樹梅派發動招式 {label}...")
    # 發送與樹梅派 C++ 端相同的事件名稱
    sio.emit('renesas_cast_spell', {'label': str(label)})

if __name__ == '__main__':
    try:
        sio.connect(SERVER_IP)
        
        print("\n--- 樹梅派攻擊模擬器 ---")
        print("1: 烈焰衝擊 (Pyro-Blast)")
        print("2: 寒冰星辰 (Frost-Vortex)")
        print("3: 大地崩裂 (Earth-Quake)")
        print("4: 雷鳴亂流 (Thunder-Gale)")
        print("q: 退出")
        
        while True:
            choice = input("\n請輸入招式編號 (0-3): ")
            if choice.lower() == 'q':
                break
            if choice in ['0','1', '2', '3']:
                simulate_attack(choice)
                time.sleep(0.5) # 防止連續連點
            else:
                print("無效的輸入，請輸入 0-3 或 q")
                
        sio.disconnect()
    except Exception as e:
        print(f"連線失敗: {e}")