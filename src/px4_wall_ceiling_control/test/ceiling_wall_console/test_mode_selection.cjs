const assert=require('node:assert/strict');
const {modeSelection}=require('../../web/ceiling_wall_console/app.js');
function snapshot(target='unavailable'){const values={phase:'IDLE',owner:0,operator:{mode:1},wall:{incline_active:false},rc_selected_mode:target};return {status:Object.fromEntries(Object.entries(values).map(([k,value])=>[k,{fresh:true,value}])),ssh:{connected:true},session:{control:true,feedback_live:true,push:true}};}
assert.equal(modeSelection(snapshot()).selected,'');
for(const mode of ['ceiling','wall','incline']){const result=modeSelection(snapshot(mode));assert.equal(result.selected,mode);assert.equal(result.locked,false);}
for(const [mode,name] of [[2,'ceiling'],[3,'wall'],[5,'incline']]){const s=snapshot('wall');s.status.operator.value.mode=mode;const result=modeSelection(s);assert.equal(result.selected,name);assert(result.locked);}
for(const key of ['phase','owner','operator']){const s=snapshot();s.status[key].fresh=false;assert(modeSelection(s).locked);}
const hold=snapshot();hold.status.wall.value={incline_active:true,incline_phase:'HOLD'};hold.status.owner.value=2;assert.equal(modeSelection(hold).selected,'incline');assert(modeSelection(hold).locked);
for(const mutate of [s=>s.ssh.connected=false,s=>s.session.push=false,s=>s.session.feedback_live=false,s=>s.session.control=false]){const s=snapshot();mutate(s);assert(modeSelection(s).locked);}
console.log('mode selection and active task locking passed');
