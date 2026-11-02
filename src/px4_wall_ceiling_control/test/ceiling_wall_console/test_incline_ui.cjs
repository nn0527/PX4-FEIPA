const assert=require('node:assert/strict');
const {inclineView}=require('../../web/ceiling_wall_console/app.js');
function snapshot(){const values={runtime:{enabled:true},state:{armed:true,connected:true,mode:'POSCTL'},wall:{incline_ready:true},operator:{mode:1},phase:'IDLE',owner:0,kill:{valid:true,active:false},odom:{},estimator:{}};return {status:Object.fromEntries(Object.entries(values).map(([k,value])=>[k,{fresh:true,value}])),ssh:{connected:true},session:{control:true,feedback_live:true,push:true}};}
assert(inclineView(snapshot()).canStart);
for(const key of Object.keys(snapshot().status)){const s=snapshot();s.status[key].fresh=false;assert(!inclineView(s).canStart);assert(!inclineView(s).canDetach);}
for(const mutate of [s=>s.session.control=false,s=>s.session.push=false,s=>s.ssh.connected=false,s=>s.status.wall.value.incline_ready=false,s=>s.status.kill.value.active=true,s=>s.status.runtime.value.enabled=false]){const s=snapshot();mutate(s);assert(!inclineView(s).canStart);}
const hold=snapshot();hold.status.wall.value={incline_active:true,incline_phase:'HOLD',detach_ready:true};hold.status.state.value.mode='OFFBOARD';assert(inclineView(hold).canDetach);assert(!inclineView(hold).canStart);hold.status.wall.value.incline_active=false;assert(!inclineView(hold).canDetach);
console.log('incline admission and stale-feedback assertions passed');

const {inclineDetails}=require('../../web/ceiling_wall_console/app.js');
for(const [phase,step] of ['WAIT','APPROACH','SLOW_FLIP','CONTACT_SETTLE','HOLD','RETURN','RECOVER','EXIT'].map((v,i)=>[v,i])){const s=snapshot();s.status.wall.value={incline_active:true,incline_phase:phase};const d=inclineDetails(s);assert.equal(d.step,step);assert(!d.returned);}
const exiting=snapshot();exiting.status.wall.value={incline_active:true,incline_phase:'EXIT',detach_completed:true};exiting.status.state.value.mode='OFFBOARD';assert(!inclineDetails(exiting).returned);exiting.status.state.value.mode='POSCTL';exiting.status.owner.value=2;assert(!inclineDetails(exiting).returned);exiting.status.owner.value=0;assert(inclineDetails(exiting).returned);
exiting.status.wall.fresh=false;assert.equal(inclineDetails(exiting).step,-1);assert(!inclineDetails(exiting).returned);
const fault=snapshot();fault.status.wall.value={incline_active:true,incline_phase:'LAND',incline_fault_reason:'control lost'};assert.equal(inclineDetails(fault).step,-1);assert.equal(inclineDetails(fault).reason,'control lost');
