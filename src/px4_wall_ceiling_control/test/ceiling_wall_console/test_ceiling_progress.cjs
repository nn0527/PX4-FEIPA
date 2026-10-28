const assert=require('node:assert/strict');
const {view,ceilingDetailView}=require('../../web/ceiling_wall_console/app.js');
function state(c={}){return {ssh:{connected:true},session:{control:true,feedback_live:true},status:{runtime:{fresh:true,value:{enabled:true}},state:{fresh:true,value:{mode:'OFFBOARD',connected:true}},ceiling_status:{fresh:true,value:{state_name:'SURFACE_HOLD',...c}},phase:{fresh:true,value:'ACTIVE'},owner:{fresh:true,value:1},operator:{fresh:true,value:{mode:2}}}};}
let s=state({hold_elapsed_s:3.2,hold_timeout_s:5});
assert.match(ceilingDetailView(s).hold,/剩余 1.8 s/);
s.status.ceiling_status.value.hold_elapsed_s=5.5;
assert.match(ceilingDetailView(s).hold,/剩余 0.0 s/);
assert.doesNotMatch(ceilingDetailView(s).hold,/已确认 AUTO.LAND/);
s=state({state_name:'ABORT',abort_land_requested:true});
assert.match(ceilingDetailView(s).hold,/尚未确认/);
s.status.state.value.mode='AUTO.LAND';assert.match(ceilingDetailView(s).hold,/已确认 AUTO.LAND/);
s.status.state.value.connected=false;assert.match(ceilingDetailView(s).hold,/通信不可用/);assert.doesNotMatch(ceilingDetailView(s).hold,/已确认 AUTO.LAND/);
s=state({rc_manual_coordination_enabled:true,rc_manual_feedback_ready:true,rc_manual_active:true,rc_position_weight:.25});
assert.match(ceilingDetailView(s).manual,/人工修正中/);assert.match(ceilingDetailView(s).manual,/0.25/);
s.status.ceiling_status.value.rc_manual_active=false;assert.match(ceilingDetailView(s).manual,/恢复中/);
s.status.ceiling_status.value.rc_position_weight=1;assert.match(ceilingDetailView(s).manual,/已消退/);
s.status.ceiling_status.value.rc_manual_feedback_ready=false;assert.match(ceilingDetailView(s).manual,/禁止恢复完成/);
s=state();assert.match(ceilingDetailView(s).manual,/等待协调字段/);
s=state({state_name:'DETACH',baro_fresh:true,baro_reference_ready:true,descended_height_m:.49,detach_height_ready:false});
assert.match(ceilingDetailView(s).descent,/0.490/);assert.match(ceilingDetailView(s).descent,/等待高度及速度稳定/);
s.status.ceiling_status.value.baro_fresh=false;assert.match(ceilingDetailView(s).descent,/无法确认/);
s=state({state_name:'RECOVERY_HOVER',recovery_ready:true,recovery_attitude_locked:true});assert.match(ceilingDetailView(s).recovery,/CH12/);assert.match(ceilingDetailView(s).manual,/允许遥控修正/);assert.match(view(s).stage,/等待 CH12/);
s.status.ceiling_status.value.recovery_ready=false;assert.match(view(s).stage,/等待稳定/);assert.match(ceilingDetailView(s).recovery,/回中/);
s.status.phase.value='RETURNING';assert.equal(view(s).stage,'等待遥控接管');
for(const mutate of [s=>s.ssh.connected=false,s=>s.session.feedback_live=false,s=>s.session.push=false,s=>s.status.ceiling_status.fresh=false]){
 s=state({rc_manual_coordination_enabled:true,rc_manual_feedback_ready:true,hold_elapsed_s:2,hold_timeout_s:5});mutate(s);
 for(const text of Object.values(ceilingDetailView(s)))assert.match(text,/等待新鲜顶吸反馈/);
}
s=state({state_name:'NORMAL_FLIGHT'});s.status.owner.value=2;s.status.operator.value.mode=3;
for(const phase of ['PRESTREAM','ENTERING','RETURNING','LOCKED']){s.status.phase.value=phase;assert.equal(view(s).stage,'顶吸待命');}
console.log('Ceiling progress, stale feedback and task isolation checks passed');

s=state({state_name:'DETACH',height_source:'px4_fused_hold_relative',height_feedback_valid:true,height_reference_ready:true,descended_height_m:.51,detach_height_ready:true,baro_fresh:false,baro_reference_ready:false});
assert.match(ceilingDetailView(s).descent,/PX4融合下降 0.510/);assert.match(ceilingDetailView(s).descent,/下降准入已达标/);assert.match(ceilingDetailView(s).descent,/气压对照 未就绪/);
s.status.ceiling_status.value.height_feedback_valid=false;assert.match(ceilingDetailView(s).descent,/不回退气压/);
