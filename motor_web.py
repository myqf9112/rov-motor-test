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
# Signed thrust in percent of full scale: 0 = stop (PWM 1500), +40 = PWM 1660.
# ArduPilot's MOTOR_TEST_THROTTLE_PERCENT maps linearly onto RC3_MIN..MAX
# (1100..1900), so percent 5 really means PWM 1140 -- almost full reverse.
MAX_THRUST = 40.0
MAX_SECONDS = 10.0
VERIFY_TIMEOUT = 1.5
MOUNT_FUNC = 7          # SERVO9_FUNCTION value that makes AUX1 the mount tilt servo
BTN_LIGHTS = {32: 'brighter', 33: 'dimmer', 31: 'cycle'}  # JSButton lights functions
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
    <label>推力 <b id="thrustLabel">+20%</b><br><input type="range" id="thrust" min="-40" max="40" value="20" step="5"></label>
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
    <b>实时输出 SERVO_OUTPUT_RAW</b>
    <span class="mono" id="expected"></span>
  </div>
  <div class="status" id="outs" style="margin-top:8px"></div>
  <div class="mono" style="margin-top:8px;color:var(--muted)">
    测试期间混控被旁路：非被测通道冻结在测试前的值（通常 1500）。被测通道没偏离目标 → 自动中止并 disarm。
  </div>
</div>

<div class="card">
  <div class="row" style="justify-content:space-between">
    <b>云台舵机 AUX1（mount tilt）</b>
    <span class="mono">当前 PWM <b id="aux1">-</b></span>
  </div>
  <div class="row" style="margin-top:10px">
    <label>俯仰 <b id="pitchLabel">0°</b><br><input type="range" id="pitch" min="-90" max="20" value="0" step="1"></label>
    <button id="pitchMin">最下</button>
    <button id="pitchZero">0°</button>
    <button id="pitchMax">最上</button>
  </div>
  <div class="mono" style="margin-top:8px;color:var(--muted)">
    走 <span class="mono">DO_MOUNT_CONTROL</span>。注意 0° 对应 ≈1754 而非 1500：`MNT1_PITCH_MIN=-90 / MAX=+20` 不对称。
  </div>
</div>

<div class="card">
  <div class="row" style="justify-content:space-between">
    <b>灯 AUX2（lights1）</b>
    <span class="mono">当前 PWM <b id="aux2">-</b></span>
  </div>
  <div class="row" style="margin-top:10px">
    <button id="lightOff">全灭</button>
    <button id="lightDown">暗一档</button>
    <button id="lightUp">亮一档</button>
    <button id="lightOn">全亮</button>
  </div>
  <div class="mono" style="margin-top:8px;color:var(--muted)">
    走 <span class="mono">MANUAL_CONTROL.buttons</span> → <span class="mono">BTNx_FUNCTION</span> 的分档调光。
    AUX2 被 <span class="mono">lights1</span> 占用，<span class="mono">DO_SET_SERVO</span> 会被拒。
  </div>
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
const state = {busy:false, mountInit:false};
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
function renderOuts(s){
  const box=$('outs');
  if(!box.children.length){
    for(let i=1;i<=8;i++){
      const d=document.createElement('div'); d.className='kv'; d.id='o'+i;
      d.innerHTML='<b>M'+i+'</b><span>-</span>';
      box.appendChild(d);
    }
  }
  const out=s.out||[];
  for(let i=1;i<=8;i++){
    const span=$('o'+i).querySelector('span');
    if(out.length>=i){ span.textContent=out[i-1]; span.style.color=''; }
    else { span.textContent='-'; }
  }
  if(s.testing && s.motor && s.thrust!=null){
    const target=Math.round(1500+s.thrust*4);
    $('expected').textContent='M'+s.motor+' 目标 PWM '+target;
    if(out.length>=s.motor){
      const ok=Math.abs(out[s.motor-1]-target)<=25;
      $('o'+s.motor).querySelector('span').style.color = ok ? 'var(--ok)' : 'var(--bad)';
    }
  } else {
    $('expected').textContent='';
  }
}
function renderAux(s){
  $('aux1').textContent = s.aux ? s.aux[0] : '-';
  $('aux2').textContent = s.aux ? s.aux[1] : '-';
  if(s.mount_min!=null && !state.mountInit){
    $('pitch').min=s.mount_min; $('pitch').max=s.mount_max;
    state.mountInit=true;
  }
}
function updateStatus(s){
  $('serial').innerHTML = s.serial_ok ? '<span class="badge ok">OK</span>' : '<span class="badge bad">FAIL</span>';
  $('mode').textContent = s.mode || '-';
  $('armed').innerHTML = s.armed ? '<span class="badge warn">ARMED</span>' : '<span class="badge">DISARMED</span>';
  $('phase').textContent = s.phase || 'idle';
  $('motor').textContent = s.motor ? ('M'+s.motor+' '+(s.thrust>0?'+':'')+s.thrust+'%') : '-';
  $('remaining').textContent = s.remaining != null ? (s.remaining.toFixed(1)+' s') : '-';
  renderOuts(s);
  renderAux(s);
  let msg=[];
  if(s.warn) msg.push('WARN: '+s.warn);
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
  const thrust=+$('thrust').value, seconds=+$('dur').value;
  if(!thrust){ alert('推力不能为 0（0 = 停转）'); return; }
  setBusy(true);
  await post('/api/test',{motor:i,thrust,seconds,props_off:true});
}
$('thrust').oninput=()=>{
  const v=+$('thrust').value;
  $('thrustLabel').textContent=(v>0?'+':'')+v+'% → PWM '+Math.round(1500+v*4);
};
$('dur').oninput=()=>$('durLabel').textContent=$('dur').value+'s';
$('propsOff').onchange=()=>setBusy(state.busy);
$('stop').onclick=()=>post('/api/stop');
function setPitch(v){
  v=Math.round(v);
  $('pitch').value=v; $('pitchLabel').textContent=v+'°';
  post('/api/mount',{pitch:v});
}
$('pitch').oninput=()=>$('pitchLabel').textContent=$('pitch').value+'°';
$('pitch').onchange=()=>post('/api/mount',{pitch:+$('pitch').value});
$('pitchMin').onclick=()=>setPitch(+$('pitch').min);
$('pitchZero').onclick=()=>setPitch(0);
$('pitchMax').onclick=()=>setPitch(+$('pitch').max);
$('lightOn').onclick=()=>post('/api/light',{name:'brighter',repeat:12});
$('lightOff').onclick=()=>post('/api/light',{name:'dimmer',repeat:12});
$('lightUp').onclick=()=>post('/api/light',{name:'brighter',repeat:1});
$('lightDown').onclick=()=>post('/api/light',{name:'dimmer',repeat:1});
$('export').onclick=()=>{
  const blob=new Blob([JSON.stringify(map,null,2)],{type:'application/json'});
  const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='motor_map.json';a.click();
};
renderMotors(); renderMap(); setBusy(false); $('thrust').oninput(); poll();
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
            'thrust': None,
            'out': None,
            'aux': None,
            'mount_min': -90,
            'mount_max': 20,
            'mount_pitch': None,
            'light_bits': {},
            'warn': None,
            'remaining': None,
            'last_error': None,
            'last_statustext': None,
        }
        self.out_ts = 0.0
        self.verified = False
        self.verify_deadline = 0.0
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
        # SERVO_OUTPUT_RAW (36) is the only honest view of what the ESCs get.
        m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0, 36, 100000, 0, 0, 0, 0, 0)
        self._init_aux()
        self._set(last_statustext='connected %s' % SERIAL)

    def _fetch(self, names, timeout=4.0):
        for n in names:
            self.mav.param_fetch_one(n)
        got = {}
        end = time.time() + timeout
        while time.time() < end and len(got) < len(names):
            msg = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
            if msg is None:
                break
            got[msg.param_id] = msg.param_value
        return got

    def _init_aux(self):
        """Read the AUX wiring so the mount/lights panels know what they can drive."""
        names = ['SERVO9_FUNCTION', 'MNT1_PITCH_MIN', 'MNT1_PITCH_MAX'] + \
                ['BTN%d_FUNCTION' % i for i in range(16)]
        got = self._fetch(names)
        bits = {}
        for i in range(16):
            name = BTN_LIGHTS.get(int(got.get('BTN%d_FUNCTION' % i) or 0))
            if name is not None:
                bits[name] = i
        warn = None
        if int(got.get('SERVO9_FUNCTION') or 0) != MOUNT_FUNC:
            warn = 'AUX1 未分配 mount_tilt（SERVO9_FUNCTION=%s，应为 %d），舵机滑块不会动' % (
                got.get('SERVO9_FUNCTION'), MOUNT_FUNC)
        elif not bits:
            warn = '没有 BTNx_FUNCTION 映射到 lights1，灯无法控制'
        self._set(mount_min=got.get('MNT1_PITCH_MIN', -90),
                  mount_max=got.get('MNT1_PITCH_MAX', 20),
                  light_bits=bits, warn=warn)

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
            elif t == 'SERVO_OUTPUT_RAW' and getattr(msg, 'port', 0) == 0:
                self.out_ts = time.time()
                self._set(out=[getattr(msg, 'servo%d_raw' % i) for i in range(1, 9)],
                          aux=[getattr(msg, 'servo9_raw'), getattr(msg, 'servo10_raw')])
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
                # armed flag flips in the heartbeat at the same moment the
                # motors library latches; init_motor_test() rejects anything
                # sent in that window and then locks out for 2 s.
                time.sleep(0.8)
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

    def _manual(self, buttons):
        # MANUAL_CONTROL is the only MAVLink path into ArduSub's joystick button
        # handlers (BTNx_FUNCTION); values are 0 so the sticks stay neutral.
        try:
            self.mav.mav.manual_control_send(1, 0, 0, 0, 0, buttons, 0, 0)
        except TypeError:
            self.mav.mav.manual_control_send(1, 0, 0, 0, 0, buttons)

    def _light(self, name, repeat=1):
        # Buttons fire on the rising edge only, so every step needs a release.
        bit = self.state['light_bits'].get(name)
        if bit is None:
            self._set(last_error='没有映射到 lights1 的按键（需要 BTNx_FUNCTION=31/32/33）')
            return
        for _ in range(repeat):
            self._manual(1 << bit)
            time.sleep(0.12)
            self._manual(0)
            time.sleep(0.12)
        self._set(last_statustext='lights %s x%d' % (name, repeat))

    def _mount(self, pitch):
        self._command(mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL, pitch, 0, 0, 0, 0, 0,
                      mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
        self._set(mount_pitch=pitch, last_statustext='mount pitch %.0f deg' % pitch)

    def _motor_cmd(self, motor, thrust):
        # Signed thrust -> ArduPilot motor-test percent. handle_do_motor_test()
        # maps percent linearly onto RC3_MIN..MAX, so percent 50 = PWM 1500 and
        # percent 0 = PWM 1100 (full reverse) -- never use 0 as "stop".
        percent = 50.0 + thrust / 2.0
        # ArduSub handle_do_motor_test() passes the 1-based motor number to
        # AP_MotorsMatrix::output_test_num(), which indexes motors from 0.
        # Subtract one here so the UI's M1..M8 map to physical outputs 1..8.
        # shortcut: remove this -1 if the firmware is fixed to use 1-based.
        self._command(mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                      motor - 1, mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,
                      percent, 1, 1, mavutil.mavlink.MOTOR_TEST_ORDER_BOARD, 0)

    def _begin_test(self):
        motor, thrust, seconds = self.pending
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
            self._set(testing=True, phase='testing', motor=motor, thrust=thrust,
                      remaining=seconds, last_error=None)
            self.end_time = time.time() + seconds
            self.next_cmd = 0.0
            self.stop_requested = False
            self.verified = False
            self.verify_deadline = time.time() + VERIFY_TIMEOUT
        except Exception as e:
            self._set(testing=False, phase='idle', motor=None, thrust=None,
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
            self._set(testing=False, phase='idle', motor=None, thrust=None,
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
                self.pending = (kw['motor'], kw['thrust'], kw['seconds'])
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
            elif action == 'light':
                self._light(kw['name'], kw.get('repeat', 1))
            elif action == 'mount':
                self._mount(kw['pitch'])

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
                self._motor_cmd(self.state['motor'], self.state['thrust'])
                self.next_cmd = now + 0.2
            if not self.verified and now >= self.verify_deadline:
                self._verify_output()
                if not self.testing:
                    return
            with self.lock:
                self.state['remaining'] = max(0.0, self.end_time - now)
        else:
            time.sleep(0.05)

    def _verify_output(self):
        """Abort if the commanded channel did not actually move.

        On the bench the FC has been seen to accept arm and ignore the motor
        test entirely (all outputs pinned at 1500); without this check the run
        looks successful and teaches nothing.
        """
        with self.lock:
            motor = self.state['motor']
            thrust = self.state['thrust']
            out = self.state['out']
        target = int(round(1500 + thrust * 4))
        if out is None or time.time() - self.out_ts > 1.0:
            self._set(last_error='M%d 无 SERVO_OUTPUT_RAW，未确认输出' % motor)
            self._finish_test('no telemetry')
            return
        actual = out[motor - 1]
        if abs(actual - target) > 25:
            self._set(last_error='M%d 无响应：目标 %d，实测 %d' % (motor, target, actual))
            self._finish_test('no response')
            return
        self.verified = True


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
                thrust = float(data['thrust'])
                seconds = float(data['seconds'])
            except Exception:
                self._json(400, {'error': 'motor/thrust/seconds required'})
                return
            if not 1 <= motor <= 8:
                self._json(400, {'error': 'motor must be 1..8'})
                return
            if thrust == 0:
                self._json(400, {'error': 'thrust 0 means stop; use -%s..%s' % (MAX_THRUST, MAX_THRUST)})
                return
            if not abs(thrust) <= MAX_THRUST:
                self._json(400, {'error': 'thrust must be -%s..%s' % (MAX_THRUST, MAX_THRUST)})
                return
            if not 0 < seconds <= MAX_SECONDS:
                self._json(400, {'error': 'seconds must be 0..%s' % MAX_SECONDS})
                return
            s = tester.status()
            if s['testing'] or s['phase'] in ('queued', 'arming'):
                self._json(409, {'error': 'busy'})
                return
            tester.submit('test', motor=motor, thrust=thrust, seconds=seconds)
            self._json(202, {'ok': True})
            return

        if self.path == '/api/light':
            name = data.get('name')
            if name not in ('brighter', 'dimmer', 'cycle'):
                self._json(400, {'error': 'name must be brighter/dimmer/cycle'})
                return
            try:
                repeat = int(data.get('repeat', 1))
            except Exception:
                repeat = 1
            if not 1 <= repeat <= 12:
                self._json(400, {'error': 'repeat must be 1..12'})
                return
            if tester.status()['testing']:
                self._json(409, {'error': 'busy'})
                return
            tester.submit('light', name=name, repeat=repeat)
            self._json(202, {'ok': True})
            return

        if self.path == '/api/mount':
            try:
                pitch = float(data['pitch'])
            except Exception:
                self._json(400, {'error': 'pitch required'})
                return
            if not -90 <= pitch <= 90:
                self._json(400, {'error': 'pitch must be -90..90'})
                return
            if tester.status()['testing']:
                self._json(409, {'error': 'busy'})
                return
            tester.submit('mount', pitch=pitch)
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
