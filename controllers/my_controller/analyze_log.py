"""Summarize the latest run or an explicit CSV path. Standard library only."""
import csv, math, sys
from pathlib import Path
base = Path(__file__).resolve().parent
files = sorted((base/'logs').glob('run_*.csv'), key=lambda p:p.stat().st_mtime)
if len(sys.argv)>1:
    path=Path(sys.argv[1])
elif files:
    path=files[-1]
else:
    raise SystemExit('No run logs yet. Reset and run Webots first.')
with path.open(encoding='utf-8') as f:
    rows=list(csv.DictReader(line for line in f if not line.startswith('#')))
if not rows:
    raise SystemExit('Log has no samples yet.')
def num(row,key):
    try:return float(row[key])
    except (ValueError,TypeError):return math.nan
def rms(vals):
    vals=[v for v in vals if math.isfinite(v)]
    return math.sqrt(sum(v*v for v in vals)/len(vals)) if vals else math.nan
valid=[r for r in rows if r['sensor_ok']=='1']
print('Log:',path)
print('Samples:',len(rows),'invalid sensors:',len(rows)-len(valid))
print('Last: time=%ss index=%s state=%s position=(%s,%s)m goal_distance=%sm'%tuple(rows[-1][k] for k in ['time_s','index','state','x_m','y_m','distance_m']))
print('State: 11=tracking, 12=final approach, 10=done; index is zero-based.')
for name,group in [('first half',[r for r in valid if int(r['index'])<=30]),('second half',[r for r in valid if int(r['index'])>30])]:
    if not group:continue
    # Active segment lateral error, not distance to globally nearest segment.
    print(name,'active-segment CTE RMS=%.4fm max=%.4fm'%(rms([num(r,'cross_track_m') for r in group]), max(abs(num(r,'cross_track_m')) for r in group)))
print('Rotation samples:',sum(r['rotating']=='1' for r in valid),'wheel saturation samples:',sum(num(r,'wheel_scale')<.999 for r in valid),'angular saturation samples:',sum(abs(num(r,'w_raw_rad_s'))>1 for r in valid))
# Feedback at t corresponds to commands at t-dt, not commands on the same row.
pairs=[(a,b) for a,b in zip(rows,rows[1:]) if a['sensor_ok']==b['sensor_ok']=='1']
for actual,command in [('left_actual_rad_s','left_cmd_rad_s'),('right_actual_rad_s','right_cmd_rad_s'),('w_actual_rad_s','w_cmd_rad_s')]:
    print(actual,'previous-command residual RMS=%.5f'%rms([num(b,actual)-num(a,command) for a,b in pairs]))
print('Worst lateral errors (time,index,cte_m,bearing_error_deg,v_cmd,w_cmd):')
for r in sorted(valid,key=lambda r:abs(num(r,'cross_track_m')),reverse=True)[:10]:
    print(r['time_s'],r['index'],r['cross_track_m'],round(math.degrees(num(r,'bearing_error_rad')),2),r['v_cmd_m_s'],r['w_cmd_rad_s'])
# Dwell per target exposes a missed waypoint or repeated circling.
dwell={}
for r in valid:dwell[r['index']]=dwell.get(r['index'],0)+num(r,'dt_s')
print('Longest target dwell (index,seconds):',sorted(dwell.items(),key=lambda item:item[1],reverse=True)[:8])
