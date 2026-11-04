import rosbag,collections,json
path='/root/ceiling_test/20261027T144059Z_52445a/flight_1.bag'
topics=['/mavros/rc/in','/mavros/local_position/odom','/mavros/imu/data','/mavros/state',
        '/mavros/px4flow/raw/optical_flow_rad','/mavros/setpoint_raw/target_attitude',
        '/sensor/ceiling/raw','/sensor/front/raw','/attachment/kill_switch','/operator/command',
        '/ceiling_attachment/status','/rosout']
samples=collections.defaultdict(list)
with rosbag.Bag(path) as bag:
    for topic,m,t in bag.read_messages(topics=topics):
        if not 1793112103<=t.to_sec()<=1793112112:continue
        source=m.header.stamp.to_sec() if hasattr(m,'header') else None
        samples[topic].append((t.to_sec(),source))
        if topic=='/rosout' and (m.name=='/mavros' or m.level>=8):
            print('LOG',t.to_sec(),m.header.stamp.to_sec(),m.name,m.msg)
for topic,values in samples.items():
    if len(values)<2:continue
    a,b=max(zip(values,values[1:]),key=lambda x:x[1][0]-x[0][0])
    inside=sum(1793112105.85<t<1793112110.10 for t,_ in values)
    print('GAP',topic,'count_in_gap',inside,'max',round(b[0]-a[0],4),'before',a,'after',b)
