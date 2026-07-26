import network
import socket
import time
import omni
from MotorConfig import *

o = omni.Omni(PWM_LIST,IN_2_LIST,IN_1_LIST)
# ----------------------------------------------------
# 1. AP（親機）モードのセットアップ
# ----------------------------------------------------
ap = network.WLAN(network.AP_IF)
ap.active(True)

# SSIDとパスワードの設定（パスワードは8文字以上）
ap.config(essid="Pico2W-AP", password="password123")

time.sleep(1)

ip = ap.ifconfig()[0]
print("====================================")
print("APモード起動完了!")
print("SSID       : Pico2W-AP")
print("IP Address :", ip)
print("====================================")

# ----------------------------------------------------
# 2. 配信するHTMLの定義（CSSなし・JSによる高速非同期送信）
# ----------------------------------------------------
html = """<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>Pico 2 W Controller</title>
</head>
<body>
    <h1>Pico 2 W RC Controller</h1>
    <p>現在のコマンド: <b id="status">STOP</b></p>
    <hr>
    
    <!-- CSSなしのレイアウト（全方向ボタン：押している間だけ送信、離したらSTOP） -->
    <div style="-webkit-user-select: none; user-select: none;">
        <br>
        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
        <button 
            onmousedown="send('UP')" 
            onmouseup="send('STOP')" 
            onmouseleave="send('STOP')"
            ontouchstart="event.preventDefault(); send('UP');" 
            ontouchend="event.preventDefault(); send('STOP');"
            ontouchcancel="event.preventDefault(); send('STOP');">▲ 前進 (UP)</button>
        <br><br>
        <button 
            onmousedown="send('LEFT')" 
            onmouseup="send('STOP')" 
            onmouseleave="send('STOP')"
            ontouchstart="event.preventDefault(); send('LEFT');" 
            ontouchend="event.preventDefault(); send('STOP');"
            ontouchcancel="event.preventDefault(); send('STOP');">◄ 左回転 (LEFT)</button>
        &nbsp;&nbsp;
        <button 
            onmousedown="send('RIGHT')" 
            onmouseup="send('STOP')" 
            onmouseleave="send('STOP')"
            ontouchstart="event.preventDefault(); send('RIGHT');" 
            ontouchend="event.preventDefault(); send('STOP');"
            ontouchcancel="event.preventDefault(); send('STOP');">右回転 (RIGHT) ►</button>
        <br><br>
        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
        <button 
            onmousedown="send('DOWN')" 
            onmouseup="send('STOP')" 
            onmouseleave="send('STOP')"
            ontouchstart="event.preventDefault(); send('DOWN');" 
            ontouchend="event.preventDefault(); send('STOP');"
            ontouchcancel="event.preventDefault(); send('STOP');">▼ 後進 (DOWN)</button>
    </div>

    <script>
    let lastCmd = '';

    // 画面をリロードせずにバックグラウンドで高速送信する関数
    function send(dir) {
        // 同じコマンドの連続送信による負荷を軽減しつつ即時更新
        if (lastCmd === dir && dir !== 'STOP') return;
        lastCmd = dir;
        
        document.getElementById('status').innerText = dir;
        // Picoに超軽量なリクエストを非同期で送信
        fetch('/cmd?dir=' + dir).catch(err => console.log(err));
    }
    </script>
</body>
</html>
"""

# ----------------------------------------------------
# 3. 簡易Webサーバーの立ち上げ
# ----------------------------------------------------
addr = socket.getaddrinfo("0.0.0.0", 80)[0][-1]
server = socket.socket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(addr)
server.listen(5)

print("Webサーバー待機中... ブラウザから http://" + ip + " にアクセスしてください。")

# モーターやアクションを割り当てる関数
def handle_command(cmd):
    if cmd == "UP":
        print(">> [前進] モーター前進")
        o.move(0,1,25000,0)
    elif cmd == "DOWN":
        print(">> [後進] モーター後進")
        o.move(0,-1,25000,0)
    elif cmd == "LEFT":
        print(">> [左旋回] モーター左")
        o.move(0,0,25000,-1)
    elif cmd == "RIGHT":
        print(">> [右旋回] モーター右")
        o.move(0,0,25000,1)
    elif cmd == "STOP":
        print(">> [停止] モーター停止")
        o.stop()
# ----------------------------------------------------
# 4. 超高速リクエスト処理ループ
# ----------------------------------------------------
while True:
    try:
        conn, addr = server.accept()
        
        # 受信データの読み込み
        request = conn.recv(512).decode("utf-8")
        
        # コマンド送信（/cmd?dir=XXX）の判定
        if "GET /cmd?dir=" in request:
            # URLからコマンド名を切り出し（例: UP, DOWN, LEFT, RIGHT, STOP）
            try:
                cmd = request.split("GET /cmd?dir=")[1].split(" ")[0]
                handle_command(cmd)
            except Exception:
                pass
            
            # APIレスポンスは画面を描画しないため超軽量な「200 OK」だけを即返答して高速化
            conn.send("HTTP/1.1 200 OK\r\nContent-Length: 2\r\n\r\nOK".encode("utf-8"))
        
        # 初回アクセス（ブラウザでページを開いた時）
        else:
            response = (
                "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
                + html
            )
            conn.send(response.encode("utf-8"))
            
        conn.close()

    except Exception as e:
        print("通信エラー:", e)