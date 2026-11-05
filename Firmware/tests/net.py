import network
import socket
import select
import time
import hashlib
import ubinascii
import errno
import omni
from MotorConfig import *

MAX_POWER = 30000

_EAGAIN = errno.EAGAIN
try:
    _EWOULDBLOCK = errno.EWOULDBLOCK
except AttributeError:
    _EWOULDBLOCK = _EAGAIN

WATCHDOG_MS = 250
_last_cmd_ms = time.ticks_ms()
_cmd_stopped = False

o = omni.Omni(PWM_LIST, IN_2_LIST, IN_1_LIST)
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
# 2. 配信するHTML（ジョイスティックUI・WebSocketで連続送信）
# ----------------------------------------------------
html = """<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>Pico 2 W RC Controller</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  html, body { touch-action: none; overscroll-behavior: none; }
  body {
    font-family: system-ui, -apple-system, "Segoe UI", Roboto, sans-serif;
    background: #1b1f27; color: #e8ecf3;
    min-height: 100vh; display: flex; flex-direction: column;
    align-items: center; padding: 12px; gap: 12px;
    user-select: none; -webkit-user-select: none;
  }
  h1 { font-size: 18px; letter-spacing: .5px; }
  #conn { font-size: 12px; padding: 3px 10px; border-radius: 999px; background: #333; }
  #conn.ok { background: #1e6b3a; }
  #conn.err { background: #7a2d2d; }
  #status {
    font-size: 13px; font-variant-numeric: tabular-nums;
    background: #10141b; border: 1px solid #2c3542; border-radius: 8px;
    padding: 6px 12px; min-width: 240px; text-align: center;
  }
  #modes { display: flex; gap: 6px; flex-wrap: wrap; justify-content: center; }
  .modebtn {
    border: 1px solid #3a4554; background: #252c38; color: #c9d2de;
    padding: 6px 12px; border-radius: 8px; font-size: 12px; cursor: pointer;
  }
  .modebtn.on { background: #2f6fd0; border-color: #5b93e8; color: #fff; }
  #stage { display: flex; gap: 24px; align-items: center; }
  .stickwrap { display: flex; flex-direction: column; align-items: center; gap: 6px; }
  .stick {
    position: relative; width: 160px; height: 160px; border-radius: 50%;
    background: radial-gradient(circle at 50% 35%, #2b3442, #171c25 70%);
    border: 2px solid #3a4554; touch-action: none;
  }
  .stick.active { border-color: #5b93e8; box-shadow: 0 0 14px rgba(91,147,232,.35); }
  .stick .knob {
    position: absolute; left: 50%; top: 50%;
    width: 64px; height: 64px; margin: -32px 0 0 -32px;
    border-radius: 50%; background: radial-gradient(circle at 40% 35%, #5b93e8, #2f6fd0 70%);
    box-shadow: 0 3px 8px rgba(0,0,0,.5);
    pointer-events: none;
  }
  .label { font-size: 12px; color: #8a97a8; }
  #sliderwrap {
    display: flex; align-items: center; gap: 10px;
    border: 1px solid #2c3542; background: #10141b; border-radius: 8px; padding: 8px 14px;
  }
  #rot { width: 200px; accent-color: #2f6fd0; }
  #speedwrap {
    display: flex; align-items: center; gap: 10px;
    border: 1px solid #2c3542; background: #10141b; border-radius: 8px; padding: 8px 14px;
  }
  #spd { width: 200px; accent-color: #2f6fd0; }
  #spdVal { min-width: 34px; text-align: right; }
  #stopBtn {
    border: 1px solid #b34; background: #7a1f1f; color: #fff;
    padding: 10px 26px; border-radius: 10px; font-size: 14px; font-weight: bold;
    cursor: pointer;
  }
  .note { font-size: 11px; color: #6d7989; text-align: center; }
</style>
</head>
<body>
  <h1>Pico 2 W RC Controller</h1>
  <div id="conn">接続待ち...</div>
  <div id="status">Vx: 0.00 | Vy: 0.00 | 旋回: 0.00</div>

  <div id="modes">
    <button class="modebtn" data-mode="1">並進のみ</button>
    <button class="modebtn" data-mode="2">ツインスティック</button>
    <button class="modebtn" data-mode="3">スライダー</button>
    <button class="modebtn" data-mode="4">ゲームパッド</button>
  </div>

  <div id="stage">
    <div class="stickwrap">
      <div id="Lstick" class="stick"><div class="knob"></div></div>
      <div class="label">並進</div>
    </div>
    <div class="stickwrap" id="Rwrap">
      <div id="Rstick" class="stick"><div class="knob"></div></div>
      <div class="label">旋回</div>
    </div>
  </div>

  <div id="sliderwrap">
    <span class="label">旋回</span>
    <input type="range" id="rot" min="-1" max="1" step="0.01" value="0">
  </div>

  <div id="speedwrap">
    <span class="label">速度</span>
    <input type="range" id="spd" min="0" max="1" step="0.01" value="0.5">
    <span class="label" id="spdVal">50%</span>
  </div>

  <button id="stopBtn">全停止</button>
  <p class="note">指/マウスでドラッグして操作。ゲームパッドは自動認識。速度は全モード共通。</p>

<script>
const MODE_TRANS = 1, MODE_DUAL = 2, MODE_SLIDER = 3, MODE_GP = 4;
const THROTTLE = 16;
const FALLBACK_MS = 100;
const DEAD = 0.08;

let mode = MODE_DUAL;
let lastTime = 0;
let lastFetch = 0;
let ws = null;
let gp = null;

const L = { active:false, x:0, y:0 };
const R = { active:false, x:0, y:0 };

const $ = id => document.getElementById(id);

function clamp(v, m, M){ return Math.max(m, Math.min(M, v)); }
function dead(v){ return Math.abs(v) < DEAD ? 0 : v; }

function setupStick(el, st){
  el.addEventListener('pointerdown', e => {
    st.active = true;
    el.classList.add('active');
    try { el.setPointerCapture(e.pointerId); } catch(_){}
    onDrag(e, el, st);
  });
  el.addEventListener('pointermove', e => { if (st.active) onDrag(e, el, st); });
  const end = () => {
    if (!st.active) return;
    st.active = false; st.x = 0; st.y = 0;
    el.classList.remove('active');
    el.querySelector('.knob').style.transform = 'translate(0,0)';
    send(0, 0, 0, true);
  };
  el.addEventListener('pointerup', end);
  el.addEventListener('pointercancel', end);
}

function onDrag(e, el, st){
  const r = el.getBoundingClientRect();
  const rad = r.width / 2;
  let dx = e.clientX - (r.left + rad);
  let dy = e.clientY - (r.top + rad);
  const d = Math.hypot(dx, dy);
  if (d > rad){ dx = dx * rad / d; dy = dy * rad / d; }
  el.querySelector('.knob').style.transform = 'translate(' + dx + 'px,' + dy + 'px)';
  st.x = clamp(dx / rad, -1, 1);
  st.y = clamp(-dy / rad, -1, 1);
}

function setMode(m){
  mode = m;
  document.querySelectorAll('.modebtn').forEach(b => b.classList.toggle('on', +b.dataset.mode === m));
  const dual = m === MODE_DUAL || m === MODE_GP;
  $('Rwrap').style.display = dual ? 'flex' : 'none';
  $('sliderwrap').style.display = m === MODE_SLIDER ? 'flex' : 'none';
  if (m === MODE_TRANS || m === MODE_SLIDER){
    R.active = false; R.x = 0; R.y = 0;
    $('Rstick').querySelector('.knob').style.transform = 'translate(0,0)';
  }
}
document.querySelectorAll('.modebtn').forEach(b =>
  b.addEventListener('click', () => setMode(+b.dataset.mode)));

const spdEl = $('spd'), spdValEl = $('spdVal');
spdEl.addEventListener('input', () => {
  spdValEl.textContent = Math.round(spdEl.value * 100) + '%';
});

function send(vx, vy, om, force){
  const now = performance.now();
  if (!force && now - lastTime < THROTTLE) return;
  lastTime = now;
  const spd = parseFloat(spdEl.value);
  const s = vx.toFixed(2) + ',' + vy.toFixed(2) + ',' + om.toFixed(2) + ',' + spd.toFixed(2);
  $('status').textContent =
    'Vx: ' + vx.toFixed(2) + ' | Vy: ' + vy.toFixed(2) + ' | 旋回: ' + om.toFixed(2) + ' | 速度: ' + spd.toFixed(2);
  const viaWs = ws && ws.readyState === WebSocket.OPEN;
  if (viaWs){
    ws.send(s);
  }
  if (location.protocol.indexOf('http') === 0){
    if (force || (!viaWs && now - lastFetch >= FALLBACK_MS)){
      lastFetch = now;
      fetch('/cmd?vx=' + vx.toFixed(2) + '&vy=' + vy.toFixed(2) + '&om=' + om.toFixed(2) + '&spd=' + spd.toFixed(2))
        .catch(()=>{});
    }
  }
}

function loop(){
  let vx = 0, vy = 0, om = 0;
  const pads = navigator.getGamepads ? navigator.getGamepads() : null;
  if (pads && pads[0]) gp = pads[0];

  if (mode === MODE_GP && gp && gp.axes){
    vx = dead(gp.axes[0] || 0);
    vy = -dead(gp.axes[1] || 0);
    om = dead(gp.axes[2] || 0);
  } else {
    vx = L.active ? L.x : 0;
    vy = L.active ? L.y : 0;
    if (mode === MODE_DUAL) om = R.active ? R.x : 0;
    else if (mode === MODE_SLIDER) om = parseFloat($('rot').value);
  }
  send(vx, vy, om);
  requestAnimationFrame(loop);
}

function setConn(cls, text){
  $('conn').className = cls;
  $('conn').textContent = text;
}

function connect(){
  if (location.protocol.indexOf('http') !== 0){
    setConn('err', 'オフライン(プレビュー)');
    return;
  }
  try {
    ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws');
  } catch(_){ setConn('err', '接続不可'); return; }
  ws.onopen = () => setConn('ok', '接続OK');
  ws.onclose = () => { setConn('err', '接続断・再接続中'); setTimeout(connect, 500); };
  ws.onerror = () => { try{ ws.close(); }catch(_){} };
}

$('stopBtn').addEventListener('click', () => {
  L.active = false; R.active = false; L.x = 0; L.y = 0; R.x = 0; R.y = 0;
  $('Lstick').classList.remove('active'); $('Rstick').classList.remove('active');
  document.querySelectorAll('.stick .knob').forEach(k => k.style.transform = 'translate(0,0)');
  $('rot').value = 0;
  send(0, 0, 0, true);
});
window.addEventListener('blur', () => send(0, 0, 0, true));

setupStick($('Lstick'), L);
setupStick($('Rstick'), R);
setMode(mode);
connect();
requestAnimationFrame(loop);
</script>
</body>
</html>
"""

# ----------------------------------------------------
# 3. コマンド処理（vx,vy,om,速度）
# ----------------------------------------------------
def do_command(vals):
    global _last_cmd_ms
    try:
        vx, vy, om, spd = vals
        _last_cmd_ms = time.ticks_ms()
    except Exception:
        return
    if (vx == 0 and vy == 0 and om == 0) or spd <= 0:
        o.stop()
    else:
        o.move(vx, vy, int(MAX_POWER * spd), om)

def do_query(qs):
    d = {}
    for kv in qs.split("&"):
        if "=" in kv:
            k, v = kv.split("=", 1)
            d[k] = v
    def getf(k, default=0.0):
        try:
            return min(1.0, max(-1.0, float(d.get(k, default))))
        except ValueError:
            return 0.0
    vx = getf("vx")
    vy = getf("vy")
    om = getf("om")
    spd = min(1.0, max(0.0, getf("spd", 0.5)))
    do_command([vx, vy, om, spd])

# ----------------------------------------------------
# 4. WebSocket フレーム解析
# ----------------------------------------------------
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

def parse_ws(buf):
    if len(buf) < 2:
        return None
    opcode = buf[0] & 0x0F
    length = buf[1] & 0x7F
    masked = (buf[1] & 0x80) != 0
    idx = 2
    if length == 126:
        if len(buf) < 4:
            return None
        length = (buf[2] << 8) | buf[3]
        idx = 4
    elif length == 127:
        if len(buf) < 10:
            return None
        length = int.from_bytes(buf[2:10], "big")
        idx = 10
    if masked:
        if len(buf) < idx + 4:
            return None
        mask = buf[idx:idx + 4]
        idx += 4
    if len(buf) < idx + length:
        return None
    payload = bytearray(buf[idx:idx + length])
    if masked:
        for i in range(length):
            payload[i] ^= mask[i % 4]
    return (idx + length, bytes(payload), opcode)

def send_all(sock, data):
    view = memoryview(data)
    while len(view):
        try:
            n = sock.send(view)
        except OSError as e:
            if e.args[0] in (_EAGAIN, _EWOULDBLOCK):
                time.sleep_ms(1)
                continue
            return False
        if not n:
            time.sleep_ms(1)
        else:
            view = view[n:]
    return True

def handle_ws(sock, buf):
    while True:
        r = parse_ws(buf)
        if r is None:
            break
        consumed, payload, opcode = r
        buf = buf[consumed:]
        if opcode == 0x8:
            return None
        elif opcode == 0x9:
            try:
                send_all(sock, b"\x8a" + bytes([len(payload)]) + payload)
            except OSError:
                pass
        elif opcode == 0x1:
            try:
                do_command([float(x) for x in payload.decode("utf-8", "ignore").split(",")])
            except Exception:
                pass
    return buf

# ----------------------------------------------------
# 5. HTTP 処理（ページ配信 / /cmd フォールバック / WSハンドシェイク）
# ----------------------------------------------------
def handle_http(sock, buf):
    while b"\r\n\r\n" in buf:
        head, buf = buf.split(b"\r\n\r\n", 1)
        lines = head.split(b"\r\n")
        if not lines:
            continue
        req = lines[0].decode("utf-8", "ignore")
        parts = req.split(" ")
        if len(parts) < 2:
            return None
        method, path = parts[0], parts[1]
        headers = {}
        for ln in lines[1:]:
            if b":" in ln:
                k, v = ln.split(b":", 1)
                headers[k.strip().lower()] = v.strip()

        if method == "GET" and path.startswith("/ws"):
            key = headers.get(b"sec-websocket-key", b"")
            if not key:
                return None
            accept = ubinascii.b2a_base64(hashlib.sha1(key + WS_GUID.encode()).digest()).strip()
            send_all(sock, ("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                            "Connection: Upgrade\r\nSec-WebSocket-Accept: " + accept.decode() + "\r\n\r\n").encode())
            return (buf, "ws")

        elif method == "GET":
            keep_close = headers.get(b"connection", b"").lower() == b"close"
            if path.startswith("/cmd"):
                qs = path.split("?", 1)[1] if "?" in path else ""
                do_query(qs)
                send_all(sock, b"HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n")
            elif path in ("/", "/index.html"):
                body = html.encode("utf-8")
                send_all(sock, b"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
                               b"Content-Length: " + str(len(body)).encode() + b"\r\n\r\n" + body)
            else:
                send_all(sock, b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n")
            if keep_close:
                return None
        else:
            return None
    return (buf, "http")

# ----------------------------------------------------
# 6. ノンブロッキング多重サーバー（keep-alive + WebSocket）
# ----------------------------------------------------
addr = socket.getaddrinfo("0.0.0.0", 80)[0][-1]
server = socket.socket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(addr)
server.listen(5)
server.setblocking(False)

poll = select.poll()
poll.register(server, select.POLLIN)
clients = {}

def close_client(sock):
    try:
        poll.unregister(sock)
    except Exception:
        pass
    try:
        sock.close()
    except Exception:
        pass
    clients.pop(sock, None)

print("Webサーバー待機中... ブラウザから http://" + ip + " にアクセスしてください。")

last_sweep = time.ticks_ms()

while True:
    try:
        events = poll.poll(5)
    except OSError:
        continue

    try:
        for sock, ev in events:
            if sock is server:
                try:
                    conn, addr = server.accept()
                    conn.setblocking(False)
                    clients[conn] = {"buf": b"", "mode": "http", "last": time.ticks_ms()}
                    poll.register(conn, select.POLLIN)
                except OSError:
                    pass
                continue

            info = clients.get(sock)
            if info is None:
                continue

            try:
                data = sock.recv(512)
            except OSError as e:
                if e.args[0] in (_EAGAIN, _EWOULDBLOCK):
                    continue
                data = b""

            if data:
                info["last"] = time.ticks_ms()
                buf = info["buf"] + data
                if info["mode"] == "ws":
                    if len(buf) > 2048:
                        close_client(sock)
                        continue
                    buf = handle_ws(sock, buf)
                    if buf is None:
                        close_client(sock)
                        continue
                else:
                    if len(buf) > 8192 and b"\r\n\r\n" not in buf:
                        close_client(sock)
                        continue
                    res = handle_http(sock, buf)
                    if res is None:
                        close_client(sock)
                        continue
                    buf, info["mode"] = res
                info["buf"] = buf
            else:
                close_client(sock)

        now = time.ticks_ms()
        if now - _last_cmd_ms > WATCHDOG_MS:
            if not _cmd_stopped:
                o.stop()
                _cmd_stopped = True
        else:
            _cmd_stopped = False

        if now - last_sweep > 5000:
            last_sweep = now
            for sock in list(clients):
                if now - clients[sock]["last"] > 15000:
                    close_client(sock)

    except Exception as e:
        o.stop()
        print("通信エラー:", e)
        time.sleep_ms(50)
