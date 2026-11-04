import socket,json,secrets,time,collections
s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);s.settimeout(4)
s.connect('/root/.ros/wall_console/shared.sock')
s.sendall((json.dumps({'client':secrets.token_hex(24),'label':'read-only diagnostic','push':True})+'\n').encode())
reader=s.makefile('r');start=time.monotonic();last_ping=start;last=None;last_flags=None;gaps=[];ages=collections.defaultdict(list);stales=collections.Counter();frames=0
while time.monotonic()-start<15:
    now=time.monotonic()
    if now-last_ping>2:s.sendall(b'{"action":"ping"}\n');last_ping=now
    line=reader.readline()
    if not line:break
    msg=json.loads(line)
    if msg.get('type')!='snapshot':continue
    d=msg['data'];frames+=1
    if last is not None:gaps.append(now-last)
    last=now
    for k,v in d['status'].items():
        if v.get('age_s') is not None:ages[k].append(v['age_s'])
        if not v.get('fresh'):stales[k]+=1
    flags=tuple(k for k in ['runtime','state','odom','estimator','kill','owner','operator','ceiling_status'] if not d['status'][k]['fresh'])
    if flags!=last_flags:
        print('CHANGE',round(now-start,2),'stale',flags,'ages',{k:round(d['status'][k]['age_s'] or 0,3) for k in ['runtime','state','odom','estimator','kill']},flush=True)
        last_flags=flags
    final=d
print('FRAMES',frames,'MAX_STREAM_GAP',max(gaps) if gaps else None)
print('STALE_COUNTS',dict(stales))
for k in ['runtime','state','odom','estimator','kill','owner','operator','ceiling_status']:
    a=ages[k];print('AGE',k,'min',min(a) if a else None,'max',max(a) if a else None)
print('FLIGHT',final['status']['state']['value'])
print('RUNTIME',final['status']['runtime']['value'])
reader.close();s.close()
