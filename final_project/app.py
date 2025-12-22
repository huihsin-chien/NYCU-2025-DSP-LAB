from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import threading
import time

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# 玩家狀態資料
rpi_player_stats = {
    "hp": 100,
    "mp": 100,
    "max_hp": 100,
    "max_mp": 100
}
renesas_player_stats = {
    "hp": 100,
    "mp": 100,
    "max_hp": 100,
    "max_mp": 100
}

# 魔力恢復線程
def recover_mp():
    while True:
        time.sleep(2)  # 每2秒恢復
        # 樹莓派 MP 恢復
        if rpi_player_stats["mp"] < rpi_player_stats["max_mp"]:
            rpi_player_stats["mp"] += 5
            if rpi_player_stats["mp"] > rpi_player_stats["max_mp"]:
                rpi_player_stats["mp"] = rpi_player_stats["max_mp"]
            socketio.emit('update_rpi_stats', rpi_player_stats)
        
        # 瑞薩 MP 恢復
        if renesas_player_stats["mp"] < renesas_player_stats["max_mp"]:
            renesas_player_stats["mp"] += 5
            if renesas_player_stats["mp"] > renesas_player_stats["max_mp"]:
                renesas_player_stats["mp"] = renesas_player_stats["max_mp"]
            socketio.emit('update_renesas_stats', renesas_player_stats)

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('pi_cast_spell')
def handle_pi_attack(data):
    """樹莓派施放咒語"""
    label = data.get('label')
    print(f">>> [SERVER接收] 來自樹莓派的咒語: {label}") 

    # 檢查 MP 是否足夠
    if rpi_player_stats["mp"] >= 20:
        # 扣除施法者的 MP
        rpi_player_stats["mp"] -= 20
        
        # 對瑞薩造成傷害
        if renesas_player_stats["hp"] > 0:
            renesas_player_stats["hp"] -= 15
            if renesas_player_stats["hp"] < 0:
                renesas_player_stats["hp"] = 0
        
        # 廣播特效和狀態更新
        socketio.emit('spell_effect', {'label': str(label), 'from': 'pi'})
        socketio.emit('vibrate_request', {'to': 'renesas', 'pattern': str(label)})
        socketio.emit('update_rpi_stats', rpi_player_stats)
        socketio.emit('update_renesas_stats', renesas_player_stats)
        
        print(f"樹莓派施放咒語 {label}！瑞薩 HP: {renesas_player_stats['hp']}")
    else:
        print("--- 提示：樹莓派 MP 不足 ---")

@socketio.on('renesas_cast_spell')
def handle_renesas_attack(data):
    """瑞薩施放咒語"""
    label = data.get('label')
    print(f">>> [SERVER接收] 來自瑞薩的咒語: {label}") 

    # 檢查 MP 是否足夠
    if renesas_player_stats["mp"] >= 20:
        # 扣除施法者的 MP
        renesas_player_stats["mp"] -= 20
        
        # 對樹莓派造成傷害
        if rpi_player_stats["hp"] > 0:
            rpi_player_stats["hp"] -= 15
            if rpi_player_stats["hp"] < 0:
                rpi_player_stats["hp"] = 0
        
        # 廣播特效和狀態更新
        socketio.emit('spell_effect', {'label': str(label), 'from': 'renesas'})
        socketio.emit('vibrate_request', {'to': 'pi', 'pattern': str(label)})
        socketio.emit('update_rpi_stats', rpi_player_stats)
        socketio.emit('update_renesas_stats', renesas_player_stats)
        
        print(f"瑞薩施放咒語 {label}！樹莓派 HP: {rpi_player_stats['hp']}")
    else:
        print("--- 提示：瑞薩 MP 不足 ---")

# 保留舊的 renesas_attack 以便相容測試按鈕
@socketio.on('renesas_attack')
def handle_renesas_attack_legacy(data):
    """網頁測試按鈕用的攻擊（相容舊版）"""
    label = data['label']
    handle_renesas_attack({'label': label})

@socketio.on('reset_game')
def handle_reset():
    """重置遊戲"""
    global rpi_player_stats, renesas_player_stats
    rpi_player_stats = {
        "hp": 100,
        "mp": 100,
        "max_hp": 100,
        "max_mp": 100
    }
    renesas_player_stats = {
        "hp": 100,
        "mp": 100,
        "max_hp": 100,
        "max_mp": 100
    }
    socketio.emit('update_rpi_stats', rpi_player_stats)
    socketio.emit('update_renesas_stats', renesas_player_stats)
    socketio.emit('msg', "遊戲已重置")
    print("=== 遊戲已重置 ===")

if __name__ == '__main__':
    threading.Thread(target=recover_mp, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)