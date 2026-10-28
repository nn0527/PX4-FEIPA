const assert=require('node:assert/strict');
const {view,ceilingReceipt}=require('../../web/ceiling_wall_console/app.js');
function snapshot(){
  const values={runtime:{enabled:true},state:{armed:true,connected:true,mode:'POSCTL'},odom:{},estimator:{},operator:{mode:1,enabled:false},phase:'IDLE',owner:0,kill:{valid:true,active:false},ceiling_status:{state_name:'NORMAL_FLIGHT'},setpoint:{thrust:.45}};
  return {status:Object.fromEntries(Object.entries(values).map(([key,value])=>[key,{fresh:true,value}])),ssh:{connected:true},session:{control:true,feedback_live:true},can_ceiling:true,can_ceiling_detach:false,ceiling_blockers:[]};
}
let checks=0;
const check=(condition)=>{assert.ok(condition);checks++;};
check(view(snapshot()).canAttach);
check(!view(snapshot(),true).canAttach);
for(const key of Object.keys(snapshot().status).filter(k=>k!=='setpoint')){
  const s=snapshot();s.status[key].fresh=false;s.can_ceiling_detach=true;
  check(!view(s).canAttach);check(!view(s).canDetach);
}
for(const mutate of [s=>s.ssh.connected=false,s=>s.session.feedback_live=false,s=>s.session.control=false]){
  const s=snapshot();mutate(s);s.can_ceiling_detach=true;
  check(!view(s).canAttach);check(!view(s).canDetach);check(!view(s).canStart);
}
const hold=snapshot();hold.status.ceiling_status.value={state_name:'SURFACE_HOLD'};hold.can_ceiling=false;hold.can_ceiling_detach=true;
hold.status.state.value.mode='OFFBOARD';hold.status.owner.value=1;hold.status.phase.value='ACTIVE';hold.status.operator.value={mode:2,enabled:true};
check(view(hold).canDetach);check(!view(hold).canAttach);check(view(hold).stage==='顶吸保持');
hold.status.setpoint.fresh=false;check(!view(hold).canDetach);
const initial=snapshot();initial.status.runtime.value.enabled=false;initial.status.state.value.armed=false;
check(view(initial).canStart);initial.status.state.value.armed=true;check(!view(initial).canStart);
initial.status.state.value.armed=false;initial.starting=true;check(!view(initial).canStart);
check(!view(snapshot()).editable);
const bootstrap=snapshot();bootstrap.status.runtime={fresh:false,value:null};bootstrap.status.state={fresh:false,value:null};check(view(bootstrap).canStart);
const returning=snapshot();returning.status.phase.value='RETURNING';returning.status.operator.value={mode:2};check(view(returning).stage==='等待遥控接管');
const abort=snapshot();abort.status.ceiling_status.value={state_name:'ABORT',fault_detected:true,abort_land_requested:false,reason:'ordinary fault'};
check(view(abort).stage==='顶吸中止');
abort.status.ceiling_status.value.abort_land_requested=true;
check(view(abort).stage==='顶吸中止 · 自动降落');
const oldReceipt=snapshot();
oldReceipt.ceiling_request={state:'NOT_SENT',reason:'基础链路已停止；等待新的吸附请求'};
check(ceilingReceipt(oldReceipt).includes('基础链路已启动'));
check(!ceilingReceipt(oldReceipt).includes('已停止'));
oldReceipt.status.operator.value={mode:2,enabled:true};
oldReceipt.status.phase.value='IDLE';
oldReceipt.status.reason={fresh:true,value:'顶吸接管被阻止：PX4内部失联自动降落参数尚未核验'};
check(ceilingReceipt(oldReceipt).includes('已检测到顶吸请求，尚未接管'));
check(ceilingReceipt(oldReceipt).includes('尚未核验'));
oldReceipt.status.phase.value='ACTIVE';oldReceipt.status.owner.value=1;oldReceipt.status.state.value.mode='OFFBOARD';
check(ceilingReceipt(oldReceipt).includes('顶吸控制已接管'));
oldReceipt.status.owner.value=0;
check(!ceilingReceipt(oldReceipt).includes('顶吸控制已接管'));
oldReceipt.status.operator.value={mode:1,enabled:false};oldReceipt.status.phase.value='CANCELING_ENTRY';
oldReceipt.status.reason.value='急停反馈过期，取消接管';
check(ceilingReceipt(oldReceipt).includes('控制节点正在取消接管'));
check(ceilingReceipt(oldReceipt).includes('急停反馈过期'));
oldReceipt.status.runtime.fresh=false;
check(ceilingReceipt(oldReceipt).includes('基础链路反馈缺失或过期'));
oldReceipt.status.runtime.fresh=true;oldReceipt.status.runtime.value.enabled=false;
check(ceilingReceipt(oldReceipt).includes('基础链路未运行'));
oldReceipt.session.feedback_live=false;
check(ceilingReceipt(oldReceipt).includes('历史回执'));
console.log(`${checks} UI state assertions passed`);
