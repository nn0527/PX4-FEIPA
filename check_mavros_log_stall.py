import re,rosbag,glob,os
text=open('/root/.ros/wall_console/mavros.log','rb').read().decode('utf8','replace')
console=[float(x) for x in re.findall(r'\[ WARN\] \[(179311\d+\.\d+)\]: CMD: Unexpected command 176, result 0',text)]
for directory in ['20261027T143654Z_570ec0','20261027T143859Z_61f8f9','20261027T144059Z_52445a']:
    seen=set()
    for p in glob.glob('/root/ceiling_test/'+directory+'/*.bag'):
        if not os.path.getsize(p):continue
        with rosbag.Bag(p) as b:
            for topic,m,t in b.read_messages(topics=['/rosout']):
                if m.name=='/mavros' and m.msg=='CMD: Unexpected command 176, result 0':
                    stamp=m.header.stamp.to_sec()
                    if stamp in seen:continue
                    seen.add(stamp)
                    before=[s for s in console if s<=stamp+.001]
                    if before:
                        s=max(before)
                        print(directory,'console',s,'rosout_created',stamp,'delta_s',round(stamp-s,6))
for name in ('1','2'):
    try:
        pid=next(int(p.split()[0]) for p in os.popen('pgrep -x mavros_node').read().splitlines())
        print('CURRENT_FD',name,os.readlink('/proc/%s/fd/%s'%(pid,name)))
    except Exception as e:print(type(e).__name__,str(e))
