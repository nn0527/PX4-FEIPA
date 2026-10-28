const assert=require('node:assert/strict');
const {rangeView,wallView}=require('../../web/ceiling_wall_console/app.js');
function snapshot(){
 const values={runtime:{enabled:true,wall_pilot_mode:'POSCTL'},state:{connected:true,armed:true,mode:'OFFBOARD'},wall:{state_name:'RECOVER',detach_range_height_enabled:true,detach_task_phase_name:'RANGE_HEIGHT_CONTROL',detach_range_target_locked:true},kill:{valid:true,active:false},owner:2,setpoint:{thrust:.5},phase:'ACTIVE',operator:{mode:3},odom:{},estimator:{}};
 return {status:Object.fromEntries(Object.entries(values).map(([k,v])=>[k,{value:v,fresh:true}])),ssh:{connected:true},session:{feedback_live:true,push:true,control:true}};
}
assert.ok(rangeView(snapshot()).active);
for(const change of [s=>s.ssh.connected=false,s=>s.session.feedback_live=false,s=>s.session.push=false,s=>s.status.wall.fresh=false]){
 const s=snapshot();change(s);assert.ok(!rangeView(s).active);
}
for(const [phase,title] of Object.entries({ROTATING_TO_RANGE_ENTRY:'阶段 1',RANGE_HEIGHT_CONTROL:'阶段 2',HANDOVER_READY:'阶段 3',FAULT:'离墙异常'})){
 const s=snapshot();s.status.wall.value.detach_task_phase_name=phase;assert.ok(wallView(s).title.startsWith(title));
}
const returned=snapshot();returned.status.wall.value.detach_task_phase_name='COMPLETED';returned.status.state.value.mode='POSCTL';returned.status.owner.value=0;returned.status.phase.value='IDLE';
assert.ok(wallView(returned).returned);assert.match(wallView(returned).title,/已交回 POSCTL/);
returned.status.state.fresh=false;assert.ok(!wallView(returned).returned);assert.match(wallView(returned).title,/等待 POSCTL/);
console.log('Range diagnostics and actual handover confirmation checks passed');
