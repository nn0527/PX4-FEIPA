const assert=require('node:assert/strict');
const {ceilingFailsafeView,ceilingDetailView}=require('../../web/ceiling_wall_console/app.js');
function snapshot(){return {ssh:{connected:true},session:{feedback_live:true},status:{state:{fresh:true,value:{connected:true}},ceiling_failsafe:{fresh:true,value:{verified:true,connected:true,reason:'已核验',parameters:{COM_OBL_RC_ACT:{actual:4,expected:4},COM_OF_LOSS_T:{actual:2.0,expected:2.0}}}},ceiling_status:{fresh:true,value:{state_name:'SURFACE_HOLD',hold_elapsed_s:120,hold_timeout_s:0}}}};}
let s=snapshot();assert.equal(ceilingFailsafeView(s).state,'已核验');assert.match(ceilingFailsafeView(s).loss,/2.00/);
assert.match(ceilingDetailView(s).hold,/120.0 s.*等待遥控脱离/);assert.doesNotMatch(ceilingDetailView(s).hold,/剩余|请及时脱离/);
for(const mutate of [s=>s.status.ceiling_failsafe.fresh=false,s=>s.status.state.value.connected=false,s=>s.session.feedback_live=false,s=>s.session.push=false,s=>s.ssh.connected=false,s=>s.status.ceiling_failsafe.value.verified=false,s=>s.status.ceiling_failsafe.value.parameters.COM_OF_LOSS_T.actual=.2,s=>s.status.ceiling_failsafe.value.parameters.COM_OF_LOSS_T.actual=2.5,s=>s.status.ceiling_failsafe.value.parameters.COM_OBL_RC_ACT.actual=0]){s=snapshot();mutate(s);assert.notEqual(ceilingFailsafeView(s).state,'已核验');}
s=snapshot();delete s.status.ceiling_failsafe.value.parameters;assert.notEqual(ceilingFailsafeView(s).state,'已核验');
s=snapshot();s.status.ceiling_failsafe.value.verified=false;s.status.ceiling_failsafe.value.unverified_warning_only=true;
assert.match(ceilingFailsafeView(s).state,/仅告警/);assert.notEqual(ceilingFailsafeView(s).state,'已核验');assert.match(ceilingFailsafeView(s).reason,/实际参数尚未确认/);
s.status.ceiling_failsafe.value.verified=true;assert.equal(ceilingFailsafeView(s).state,'已核验');
console.log('Failsafe actual readback/staleness and indefinite HOLD checks passed');
