const assert=require('node:assert/strict');
const {ceilingDetailView}=require('../../web/ceiling_wall_console/app.js');
const c={state_name:'DETACH',height_source:'px4_fused_hold_relative',height_feedback_valid:true,height_reference_ready:true,
  descended_height_m:.12,detach_target_drop_m:.4,detach_max_drop_m:1.5,vertical_integral_thrust:-.03};
const s={ssh:{connected:true},session:{feedback_live:true},status:{ceiling_status:{fresh:true,value:c},state:{fresh:true,value:{connected:true,mode:'OFFBOARD'}}}};
let detail=ceilingDetailView(s);
assert.ok(detail.descent.includes('0.40 m'));
assert.ok(detail.descent.includes('1.50 m'));
assert.ok(detail.descent.includes('-0.030'));
c.state_name='RECOVERY_HOVER';detail=ceilingDetailView(s);
assert.ok(!detail.descent.includes('1.50 m'));
c.detach_target_drop_m=undefined;
assert.ok(ceilingDetailView(s).descent.includes('目标待反馈'));
console.log('5 descent PI UI assertions passed');
