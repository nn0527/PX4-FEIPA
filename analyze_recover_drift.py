import rosbag,math,json,collections,os
from px4_wall_ceiling_control.adapters.odometry import linear_velocity_parent_frame
from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion,rotate_vector
root='/root/ceiling_test/20261028T032128Z_9787a9/'
topics=['/wall_perch/status','/mavros/local_position/odom','/mavros/setpoint_raw/attitude','/mavros/setpoint_raw/target_attitude','/attachment/output_health','/attachment/output_owner','/attachment/output_mode','/mavros/state','/operator/command','/mavros/rc/in']
latest={};rows=[];last=-1;origin=None;heading=None;healthprev=None
for i in range(25,32):
    path=root+'flight_%d.bag'%i
    if not os.path.exists(path):continue
    with rosbag.Bag(path) as bag:
        print('BAG',i,bag.get_start_time(),bag.get_end_time())
        for topic,m,t in bag.read_messages(topics=topics):
            ts=t.to_sec()
            if not 1793158557<=ts<=1793158571:continue
            latest[topic]=(m,ts)
            if topic=='/attachment/output_health':
                d=json.loads(m.data);key=(d.get('owner'),d.get('output_mode'),d.get('pause_reason'))
                if key!=healthprev:print('OUTPUT',ts,m.data);healthprev=key
            if topic!='/wall_perch/status':continue
            odom=latest.get('/mavros/local_position/odom')
            if not odom:continue
            odom=odom[0];q=tuple(getattr(odom.pose.pose.orientation,k) for k in 'xyzw');r,p,y=euler_from_quaternion(q)
            vel=linear_velocity_parent_frame(odom,True);pos=odom.pose.pose.position
            if origin is None and m.state_name=='RECOVER':origin=(pos.x,pos.y,pos.z);heading=y
            yaw=heading if heading is not None else y
            def attitude(topic):
                if topic not in latest:return None
                x,received=latest[topic];qq=tuple(getattr(x.orientation,k) for k in 'xyzw')
                e=euler_from_quaternion(qq)
                return dict(age=ts-received,thrust=x.thrust,rpy=[math.degrees(v) for v in e],type_mask=x.type_mask)
            row=dict(time=ts,state=m.state_name,xyz=[pos.x,pos.y,pos.z],vel=vel,
                forward=math.cos(yaw)*vel[0]+math.sin(yaw)*vel[1],left=-math.sin(yaw)*vel[0]+math.cos(yaw)*vel[1],
                rpy=[math.degrees(v) for v in (r,p,y)],candidate=m.candidate_valid,
                sent=attitude('/mavros/setpoint_raw/attitude'),fcu=attitude('/mavros/setpoint_raw/target_attitude'))
            keys=['enabled','flight_state_fresh','odom_fresh','attitude_setpoint_valid','thrust_normalized','recover_attitude_locked',
                'detach_vertical_reason','detach_vertical_active','detach_vertical_base_thrust','detach_vertical_requested_thrust',
                'detach_vertical_integral','detach_vertical_saturated','detach_vertical_slew_limited','detach_range_valid',
                'detach_range_velocity_valid','detach_range_block_reason','detach_range_height_m','detach_range_velocity_mps',
                'detach_range_target_m','horizontal_velocity_active','horizontal_velocity_reason']
            for k in keys:row[k]=getattr(m,k,None)
            if origin is not None:
                dx,dy,dz=pos.x-origin[0],pos.y-origin[1],pos.z-origin[2]
                row['displacement_from_recover']=[math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy,dz]
            rows.append(row)
            if ts-last>=.4 or (m.state_name=='RECOVER' and (last_state!='RECOVER' or row['candidate']!=last_valid)):
                print('SAMPLE',json.dumps(row,ensure_ascii=False));last=ts
            last_state=m.state_name;last_valid=row['candidate']
print('RECOVER_SUMMARY')
recover=[r for r in rows if r['state']=='RECOVER']
for key in ['left','forward']:
    v=[r[key] for r in recover];print(key,min(v),max(v))
print('FIRST',json.dumps(recover[0],ensure_ascii=False));print('LAST',json.dumps(recover[-1],ensure_ascii=False))
