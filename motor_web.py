#!/usr/bin/env python3
"""Minimal web UI for ArduSub motor test on RK3588 / Pixhawk1.

Run on the vehicle computer:
    python3 motor_web.py
Open:
    http://<vehicle-ip>:8088/
"""
import json
import queue
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from pymavlink import mavutil

SERIAL = '/dev/ttyACM0'
BAUD = 115200
HTTP_PORT = 8088
MAX_PERCENT = 20.0
MAX_SECONDS = 10.0
ARM_TIMEOUT = 6.0
DISARM_TIMEOUT = 3.0

INDEX_HTML = r'''<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OU Motor Test</title>
<style>
:root{color-scheme:dark;--bg:#0f1115;--panel:#181b22;--line:#2b303b;--fg:#e8ecf3;--muted:#8b93a3;--ok:#39d98a;--warn:#ffb020;--bad:#ff5c5c;--blue:#4d9fff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:980px;margin:0 auto;padding:18px}
h1{font-size:20px;margin:0 0 4px}.sub{color:var(--muted);margin-bottom:16px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:14px}
.warn{border-color:#6b4b12;background:#221a0c;color:#ffd58a}
.row{display:flex;gap:12px;flex-wrap:wrap;align-items:center}
.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}
button{font:inherit;border:1px solid var(--line);border-radius:10px;background:#232833;color:var(--fg);padding:12px 14px;cursor:pointer}
button:hover{border-color:#4a5568}button:disabled{opacity:.4;cursor:not-allowed}
button.motor{font-size:18px;font-weight:700;padding:18px 10px}
button.motor small{display:block;font-size:11px;font-weight:400;color:var(--muted);margin-top:4px}
button.danger{background:#3a1717;border-color:#6b2626;color:#ffb4b4}
button.ok{background:#123024;border-color:#245c43;color:#a8f5cd}
.status{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:10px}
.kv{background:#12151b;border:1px solid var(--line);border-radius:8px;padding:8px 10px}
.kv b{display:block;font-size:11px;color:var(--muted);font-weight:500;margin-bottom:2px}
.badge{display:inline-block;padding:2px 8px;border-radius:999px;font-size:12px;background:#2a2f3a}
.badge.ok{background:#123d2b;color:#8df0bd}.badge.bad{background:#3d1717;color:#ffaaaa}.badge.warn{background:#3d3217;color:#ffd58a}
label.check{display:flex;gap:8px;align-items:center;cursor:pointer}
input[type=range]{width:220px}
table{width:100%;border-collapse:collapse}th,td{text-align:left;padding:6px 4px;border-bottom:1px solid var(--line);font-size:13px}
select{background:#232833;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:4px}
.mono{font-family:ui-monospace,Consolas,monospace;font-size:12px}
@media(max-width:700px){.grid{grid-template-columns:repeat(2,1fr)}}
</style>
</head>
<body><main>
<h1>OU Motor Test</h1>
<div class="sub">RK3588 → Pixhawk1 <span class="mono">/dev/ttyACM0</span> · 每次测试自动 arm → 测试 → disarm</div>

<div class="card warn">
  <b>安全前提：螺旋桨必须已拆，或推进器动力已断开。</b><br>
  测试会自动 arm。点击电机按钮前，请先勾选下面的确认框。
</div>

<div class="card">
  <label class="check"><input type="checkbox" id="propsOff"> 我确认：螺旋桨已拆或推进器已断电，周围无人</label>
  <div class="row" style="margin-top:12px">
    <label>油门 <b id="pctLabel">5%</b><br><input type="range" id="pct" min="1" max="20" value="5" step="1"></label>
    <label>时长 <b id="durLabel">3s</b><br><input type="range" id="dur" min="1" max="10" value="3" step="1"></label>
    <button class="danger" id="stop" disabled>紧急停止 / Disarm</button>
  </div>
</div>

<div class="card">
  <div class="grid" id="motors"></div>
</div>

<div class="card">
  <div class="status">
    <div class="kv"><b>串口</b><span id="serial">-</span></div>
    <div class="kv"><b>模式</b><span id="mode">-</span></div>
    <div class="kv"><b>Armed</b><span id="armed">-</span></div>
    <div class="kv"><b>状态</b><span id="phase">-</span></div>
    <div class="kv"><b>当前电机</b><span id="motor">-</span></div>
    <div class="kv"><b>剩余</b><span id="remaining">-</span></div>
  </div>
  <div class="mono" id="msg" style="margin-top:10px;color:var(--muted)"></div>
</div>

<div class="card">
  <div class="row" style="justify-content:space-between">
    <b>实测记录（只存在本机浏览器）</b>
    <button id="export">导出 JSON</button>
  </div>
  <table>
    <thead><tr><th>电机</th><th>实际位置</th><th>转向</th></tr></thead>
    <tbody id="mapBody"></tbody>
  </table>
</div>
</main>
<script>
const $ = id => document.getElementById(id);
const state = {busy:false};
const POS = ['未记录','前左','前右','后左','后右','前左垂','前右垂','后左垂','后右垂','其他'];
const DIR = ['未记录','正转','反转'];
const MAP_KEY = 'ou_motor_map_v1';
let map = {};
try { map = JSON.parse(localStorage.getItem(MAP_KEY) || '{}'); } catch(e) { map = {}; }

function renderMotors(){
  $('motors').innerHTML = '';
  for (let i=1;i<=8;i++){
    const b=document.createElement('button');
    b.className='motor'; b.id='m'+i;
    b.innerHTML='M'+i+'<small>测试</small>';
    b.onclick=()=>testMotor(i);
    $('motors').appendChild(b);
  }
}
function renderMap(){
  $('mapBody').innerHTML='';
  for (let i=1;i<=8;i++){
    const rec=map[i]||{pos:'未记录',dir:'未记录'};
    const tr=document.createElement('tr');
    tr.innerHTML='<td>M'+i+'</td><td><select data-i="'+i+'" data-k="pos"></select></td><td><select data-i="'+i+'" data-k="dir"></select></td>';
    $('mapBody').appendChild(tr);
    const sp=tr.querySelector('select[data-k=pos]'), sd=tr.querySelector('select[data-k=dir]');
    POS.forEach(v=>{const o=document.createElement('option');o.value=v;o.textContent=v;sp.appendChild(o)});
    DIR.forEach(v=>{const o=document.createElement('option');o.value=v;o.textContent=v;sd.appendChild(o)});
    sp.value=rec.pos; sd.value=rec.dir;
    sp.onchange=sd.onchange=()=>{map[i]={pos:sp.value,dir:sd.value};localStorage.setItem(MAP_KEY,JSON.stringify(map));};
  }
}
function setBusy(b){
  state.busy=b;
  for(let i=1;i<=8;i++) $('m'+i).disabled=b || !$('propsOff').checked;
  $('stop').disabled=false;
}
function updateStatus(s){
  $('serial').innerHTML = s.serial_ok ? '<span class="badge ok">OK</span>' : '<span class="badge bad">FAIL</span>';
  $('mode').textContent = s.mode || '-';
  $('armed').innerHTML = s.armed ? '<span class="badge warn">ARMED</span>' : '<span class="badge">DISARMED</span>';
  $('phase').textContent = s.phase || 'idle';
  $('motor').textContent = s.motor ? ('M'+s.motor+' '+s.percent+'%') : '-';
  $('remaining').textContent = s.remaining != null ? (s.remaining.toFixed(1)+' s') : '-';
  let msg=[];
  if(s.last_error) msg.push('ERROR: '+s.last_error);
  if(s.last_statustext) msg.push('FC: '+s.last_statustext);
  $('msg').textContent = msg.join('  |  ');
  setBusy(!!s.testing || ['queued','arming','stopping','disarming'].includes(s.phase));
}
async function poll(){
  try{
    const r=await fetch('/api/status',{cache:'no-store'});
    updateStatus(await r.json());
  }catch(e){ $('msg').textContent='status error: '+e; }
  setTimeout(poll,500);
}
async function post(url,body){
  const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});
  const j=await r.json().catch(()=>({}));
  if(!r.ok){ $('msg').textContent='ERROR: '+(j.error||r.status); }
  return j;
}
async function testMotor(i){
  if(!$('propsOff').checked){ alert('先勾选“螺旋桨已拆/推进器已断电”'); return; }
  const percent=+$('pct').value, seconds=+$('dur').value;
  setBusy(true);
  await post('/api/test',{motor:i,percent,seconds,props_off:true});
}
$('pct').oninput=()=>$('pctLabel').textContent=$('pct').value+'%';
$('dur').oninput=()=>$('durLabel').textContent=$('dur').value+'s';
$('propsOff').onchange=()=>setBusy(state.busy);
$('stop').onclick=()=>post('/api/stop');
$('export').onclick=()=>{
  const blob=new Blob([JSON.stringify(map,null,2)],{type:'application/json'});
  const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='motor_map.json';a.click();
};
renderMotors(); renderMap(); setBusy(false); poll();
</script>
</body></html>'''


class Tester:
    def __init__(self):
        self.q = queue.Queue()
        self.lock = threading.Lock()
        self.state = {
            'serial_ok': False,
            'mode': None,
            'armed': False,
            'testing': False,
            'phase': 'idle',
            'motor': None,
            'percent': None,
            'remaining': None,
            'last_error': None,
            'last_statustext': None,
        }
        self.mav = None
        self.pending = None
        self.testing = False
        self.end_time = 0.0
        self.next_cmd = 0.0
        self.stop_requested = False
        self.disarm_requested = False
        self.last_hb = 0.0
        self.last_hb_send = 0.0
        threading.Thread(target=self._run, daemon=True).start()

    def status(self):
        with self.lock:
            s = dict(self.state)
        if s['testing'] and s['remaining'] is not None:
            s['remaining'] = max(0.0, self.end_time - time.time())
        return s

    def _set(self, **kw):
        with self.lock:
            self.state.update(kw)
        if 'testing' in kw:
            self.testing = kw['testing']

    def submit(self, action, **kw):
        self.q.put((action, kw))

    def _run(self):
        while True:
            try:
                if self.mav is None:
                    self._connect()
                self._step()
            except Exception as e:
                self._set(serial_ok=False, last_error='%s: %s' % (type(e).__name__, e))
                if self.mav is not None:
                    try:
                        self.mav.close()
                    except Exception:
                        pass
                self.mav = None
                self.pending = None
                self._set(testing=False, phase='idle', motor=None, remaining=None, armed=False)
                time.sleep(1.0)

    def _connect(self):
        self._set(phase='connecting', last_error=None)
        m = mavutil.mavlink_connection(SERIAL, baud=BAUD)
        m.mav.srcSystem = 255
        m.mav.srcComponent = 190
        hb = m.wait_heartbeat(timeout=10)
        if hb is None:
            raise RuntimeError('no heartbeat on %s' % SERIAL)
        self.mav = m
        self._set(serial_ok=True, mode=mavutil.mode_string_v10(hb),
                  armed=bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                  phase='idle')
        self._set(last_statustext='connected %s' % SERIAL)

    def _hb(self, force=False):
        now = time.time()
        if force or now - self.last_hb_send > 0.5:
            self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                        0, 0, 0)
            self.last_hb_send = now

    def _read(self):
        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            if t == 'HEARTBEAT' and msg.get_srcSystem() == 1:
                self.last_hb = time.time()
                self._set(armed=bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                          mode=mavutil.mode_string_v10(msg))
            elif t == 'STATUSTEXT':
                text = msg.text
                if isinstance(text, bytes):
                    text = text.decode('ascii', 'ignore')
                self._set(last_statustext=text.rstrip('\x00'))

    def _command(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        self.mav.mav.command_long_send(1, 1, cmd, 0, p1, p2, p3, p4, p5, p6, p7)

    def _arm(self):
        self._set(phase='arming', last_error=None)
        self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        deadline = time.time() + ARM_TIMEOUT
        while time.time() < deadline:
            self._hb(force=True)
            self._read()
            msg = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.2)
            if msg is not None and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    raise RuntimeError('arm rejected result=%s' % msg.result)
            with self.lock:
                armed = self.state['armed']
            if armed:
                return
            time.sleep(0.05)
        raise RuntimeError('arm timeout')

    def _disarm(self):
        self._set(phase='disarming')
        deadline = time.time() + DISARM_TIMEOUT
        while time.time() < deadline:
            self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
            self._hb(force=True)
            self._read()
            with self.lock:
                armed = self.state['armed']
            if not armed:
                break
            time.sleep(0.1)
        with self.lock:
            armed = self.state['armed']
        self._set(phase='idle', armed=armed)

    def _motor_cmd(self, motor, percent):
        # ArduSub handle_do_motor_test() passes the 1-based motor number to
        # AP_MotorsMatrix::output_test_num(), which indexes motors from 0.
        # Subtract one here so the UI's M1..M8 map to physical outputs 1..8.
        # shortcut: remove this -1 if the firmware is fixed to use 1-based.
        self._command(mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                      motor - 1, mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,
                      percent, 1, 1, mavutil.mavlink.MOTOR_TEST_ORDER_BOARD, 0)

    def _begin_test(self):
        motor, percent, seconds = self.pending
        self.pending = None
        self._set(last_error=None, last_statustext=None)
        try:
            with self.lock:
                armed = self.state['armed']
            if self.stop_requested:
                self.stop_requested = False
                self._safe_stop()
                self._set(phase='idle')
                return
            if not armed:
                self._arm()
            if self.stop_requested:
                self.stop_requested = False
                self._safe_stop()
                self._set(phase='idle')
                return
            self._set(testing=True, phase='testing', motor=motor, percent=percent,
                      remaining=seconds, last_error=None)
            self.end_time = time.time() + seconds
            self.next_cmd = 0.0
            self.stop_requested = False
        except Exception as e:
            self._set(testing=False, phase='idle', motor=None, percent=None,
                      remaining=None, last_error=str(e))
            self._safe_stop()

    def _safe_stop(self):
        try:
            with self.lock:
                motor = self.state.get('motor') or 1
            for _ in range(3):
                self._hb(force=True)
                self._motor_cmd(motor, 0)
                time.sleep(0.05)
            self._disarm()
        except Exception as e:
            self._set(last_error='stop failed: %s' % e)

    def _finish_test(self, reason):
        self._set(phase='stopping')
        try:
            with self.lock:
                motor = self.state.get('motor') or 1
            for _ in range(3):
                self._hb(force=True)
                self._motor_cmd(motor, 0)
                time.sleep(0.05)
            self._disarm()
            self._set(last_statustext='test %s; disarmed' % reason)
        except Exception as e:
            self._set(last_error='finish failed: %s' % e)
        finally:
            self._set(testing=False, phase='idle', motor=None, percent=None,
                      remaining=None)
            self.stop_requested = False
            self.disarm_requested = False

    def _handle_queue(self):
        while True:
            try:
                action, kw = self.q.get_nowait()
            except queue.Empty:
                return
            if action == 'test':
                if self.pending is not None or self.testing:
                    self._set(last_error='busy: test already running')
                    continue
                self.pending = (kw['motor'], kw['percent'], kw['seconds'])
                self._set(last_error=None, last_statustext=None, phase='queued')
            elif action == 'stop':
                self.stop_requested = True
                if not self.testing and self.pending is None:
                    self._safe_stop()
            elif action == 'disarm':
                self.stop_requested = True
                self.disarm_requested = True
                if not self.testing and self.pending is None:
                    self._safe_stop()

    def _step(self):
        self._hb()
        self._read()
        self._handle_queue()

        if self.pending is not None:
            self._begin_test()
            return

        if self.testing:
            now = time.time()
            if self.stop_requested or self.disarm_requested or now >= self.end_time:
                self._finish_test('stopped' if self.stop_requested else 'done')
                return
            if now >= self.next_cmd:
                self._hb(force=True)
                self._motor_cmd(self.state['motor'], self.state['percent'])
                self.next_cmd = now + 0.2
            with self.lock:
                self.state['remaining'] = max(0.0, self.end_time - now)
        else:
            time.sleep(0.05)


tester = Tester()


class Handler(BaseHTTPRequestHandler):
    server_version = 'OU-MotorTest/1.0'

    def log_message(self, fmt, *args):
        print('%s - %s' % (self.address_string(), fmt % args), flush=True)

    def _json(self, code, obj):
        body = json.dumps(obj).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            body = INDEX_HTML.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
        elif self.path == '/api/status':
            self._json(200, tester.status())
        else:
            self._json(404, {'error': 'not found'})

    def do_POST(self):
        try:
            length = int(self.headers.get('Content-Length', '0'))
            raw = self.rfile.read(length) if length else b'{}'
            data = json.loads(raw.decode('utf-8') or '{}')
        except Exception as e:
            self._json(400, {'error': 'bad json: %s' % e})
            return

        if self.path == '/api/test':
            if data.get('props_off') is not True:
                self._json(400, {'error': 'props_off must be true'})
                return
            try:
                motor = int(data['motor'])
                percent = float(data['percent'])
                seconds = float(data['seconds'])
            except Exception:
                self._json(400, {'error': 'motor/percent/seconds required'})
                return
            if not 1 <= motor <= 8:
                self._json(400, {'error': 'motor must be 1..8'})
                return
            if not 0 < percent <= MAX_PERCENT:
                self._json(400, {'error': 'percent must be 0..%s' % MAX_PERCENT})
                return
            if not 0 < seconds <= MAX_SECONDS:
                self._json(400, {'error': 'seconds must be 0..%s' % MAX_SECONDS})
                return
            s = tester.status()
            if s['testing'] or s['phase'] in ('queued', 'arming'):
                self._json(409, {'error': 'busy'})
                return
            tester.submit('test', motor=motor, percent=percent, seconds=seconds)
            self._json(202, {'ok': True})
            return

        if self.path == '/api/stop':
            tester.submit('stop')
            self._json(202, {'ok': True})
            return

        if self.path == '/api/disarm':
            tester.submit('disarm')
            self._json(202, {'ok': True})
            return

        self._json(404, {'error': 'not found'})


def main():
    srv = ThreadingHTTPServer(('0.0.0.0', HTTP_PORT), Handler)
    print('OU motor test UI on http://0.0.0.0:%d/  serial=%s' % (HTTP_PORT, SERIAL), flush=True)
    srv.serve_forever()


if __name__ == '__main__':
    main()
