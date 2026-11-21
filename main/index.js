
const $=id=>document.getElementById(id);let current={endpoint:1,enabled:true,device_type:9,cluster_id:0,attributes:[]};
const functions=[[1,'01 • Read Coils'],[2,'02 • Read Discrete Inputs'],[3,'03 • Read Holding Registers'],[4,'04 • Read Input Registers'],[5,'05 • Write Single Coil'],[6,'06 • Write Single Register'],[15,'15 • Write Multiple Coils'],[16,'16 • Write Multiple Registers']];
const types=[[0,'BOOL (1 byte)'],[1,'UINT16 (2 bytes)'],[2,'INT16 (2 bytes)'],[3,'UINT32 (4 bytes)'],[4,'INT32 (4 bytes)'],[5,'FLOAT32 (4 bytes)'],[6,'UINT64 (8 bytes)'],[7,'INT64 (8 bytes)'],[8,'STRING (variable)'],[9,'UINT8 (1 byte)'],[10,'INT8 (1 byte)'],[11,'UINT24_t (3 bytes)'],[12,'UINT48_t (6 bytes)']];
const typeByName=Object.fromEntries(types.map(x=>[x[1].split(' ')[0].toUpperCase(),x[0]]));
function api(url,opt={}){const ctrl=new AbortController();const timer=setTimeout(()=>ctrl.abort(),5000);return fetch(url,{headers:{'Content-Type':'application/json'},...opt,signal:ctrl.signal}).then(async r=>{if(!r.ok)throw Error(await r.text());return r.json()}).finally(()=>clearTimeout(timer))}
async function init(){
 try{let names=await api('/api/device-types');names.forEach((n,i)=>$('deviceType').add(new Option(n,i)))}catch(e){['LT Panel / Panel MFM','DG','UPS','Lift','Escalator','Fire Hydrant','Anemometer','AC','Ventilation Fan','Custom'].forEach((n,i)=>$('deviceType').add(new Option(n,i)))}
 for(let i=1;i<=50;i++)$('endpoint').add(new Option(i,i));
 $('deviceType').value=9;$('endpoint').value=1;
 document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>{document.querySelectorAll('.tab,.tabcontent').forEach(x=>x.classList.remove('active'));b.classList.add('active');$(b.dataset.tab).classList.add('active')});
 try{await loadSerial()}catch(e){console.warn('Serial load failed',e)}
 try{await loadEndpoint()}catch(e){current={endpoint:1,enabled:false,device_type:9,cluster_id:0,attributes:[]};renderAttrs();console.warn('Endpoint load failed',e)}
 try{await refreshEndpoints()}catch(e){$('endpointList').innerHTML='<div class="empty">Unable to load saved endpoints.</div>';console.warn('Endpoint list failed',e)}
}

async function loadSerial(){let c=await api('/api/serial');$('baud').value=c.baud_rate;$('dataBits').value=c.data_bits;$('stopBits').value=c.stop_bits;$('parity').value=c.parity}
async function saveSerial(){await api('/api/serial',{method:'POST',body:JSON.stringify({baud_rate:+$('baud').value,data_bits:+$('dataBits').value,stop_bits:+$('stopBits').value,parity:+$('parity').value})});alert('Serial settings saved and applied.')}
async function loadEndpoint(){current=await api('/api/endpoint?ep='+$('endpoint').value);$('deviceType').value=current.device_type;$('clusterId').value=current.cluster_id||'';current.attributes.forEach(normalizeRegCount);renderAttrs()}
async function loadTemplate(){let x=await api('/api/template',{method:'POST',body:JSON.stringify({endpoint:+$('endpoint').value,device_type:+$('deviceType').value})});current=x;current.attributes.forEach(normalizeRegCount);renderAttrs();await refreshEndpoints();alert('Default device mapping loaded into this endpoint. Review and save if required.')}
function referenceForFunction(fn,address){fn=+fn;address=+address||0;if([3,6,16].includes(fn))return 40001+address;if(fn===4)return 30001+address;if(fn===2)return 10001+address;return 1+address}
function pduFromReference(fn,ref){fn=+fn;ref=+ref||0;if([3,6,16].includes(fn)&&ref>=40001&&ref<=49999)return ref-40001;if(fn===4&&ref>=30001&&ref<=39999)return ref-30001;if(fn===2&&ref>=10001&&ref<=19999)return ref-10001;if(ref>=1)return ref-1;return 0}
function functionReferenceHint(fn){fn=+fn;if([3,6,16].includes(fn))return '40001–49999';if(fn===4)return '30001–39999';if(fn===2)return '10001–19999';return '1–9999'}
function attrObj(){return {enabled:true,attribute_id:0,function:4,address:0,address2:0,address3:0,reference:30001,reg_count:1,data_type:1,word_order:0,packed_count:0,name:'Attribute'}}
function normalizeRegCount(a){if(!a.reg_count||a.reg_count<1)a.reg_count=1}
function addAttr(){current.attributes.push(attrObj());renumber();renderAttrs()}
function renumber(){current.attributes.forEach((a,i)=>a.attribute_id=i)}
function renderAttrs(){let box=$('attrs');box.innerHTML='';if(!current.attributes.length){box.innerHTML='<div class="empty">No attributes configured. Load a device template, import a CSV file or add one manually.</div>';return}current.attributes.forEach((a,i)=>{if(a.reference==null)a.reference=referenceForFunction(a.function,a.address);let d=document.createElement('div');d.className='attr';d.innerHTML=`<div class="attrgrid"><div><label>ID</label><input data-k="attribute_id" value="${a.attribute_id}" type="number" min="0" max="255"></div><div><label>Name</label><input data-k="name" value="${esc(a.name||'Attribute')}"></div><div><label>Function</label><select data-k="function">${functions.map(x=>`<option value="${x[0]}" ${x[0]==a.function?'selected':''}>${x[1]}</option>`).join('')}</select></div><div><label>Modbus Reference (datasheet)</label><input data-k="reference" value="${a.reference}" type="number" min="1" max="49999"><div class="muted">Expected: ${functionReferenceHint(a.function)}</div></div><div><label>Register / Bit Count</label><div class="muted">For phase parameters, use one row per register (R/Y/B); save each row with count = 1.</div><input data-k="reg_count" value="${a.reg_count||1}" type="number" min="1" max="125"></div><div><label>Data Type</label><select data-k="data_type">${types.map(x=>`<option value="${x[0]}" ${x[0]==a.data_type?'selected':''}>${x[1]}</option>`).join('')}</select></div><div><label>Word Order</label><select data-k="word_order"><option value="0" ${!a.word_order?'selected':''}>High word first</option><option value="1" ${a.word_order?'selected':''}>Low word first</option></select></div><div><label>Packed Count</label><input data-k="packed_count" value="${a.packed_count||0}" type="number" min="0" max="3"></div><div><label>Address 2</label><input data-k="address2" value="${a.address2||''}" type="number"></div><div><label>Address 3</label><input data-k="address3" value="${a.address3||''}" type="number"></div></div><div class="actions"><label style="display:flex;gap:8px;align-items:center;margin:0"><input data-k="enabled" type="checkbox" ${a.enabled?'checked':''} style="width:auto"> Enabled</label><button class="danger" onclick="removeAttr(${i})">Delete Attribute</button></div>`;d.querySelectorAll('[data-k]').forEach(el=>{el.oninput=el.onchange=()=>{let k=el.dataset.k;if(k==='function'){let oldPdu=+a.address||pduFromReference(a.function,a.reference);a.function=+el.value;a.address=oldPdu;a.reference=referenceForFunction(a.function,oldPdu);let refEl=d.querySelector('[data-k="reference"]');if(refEl)refEl.value=a.reference;let hint=refEl?.parentElement?.querySelector('.muted');if(hint)hint.textContent='Expected: '+functionReferenceHint(a.function);return}if(k==='reference'){current.attributes[i].reference=+el.value||0;current.attributes[i].address=pduFromReference(a.function,a.reference);return}current.attributes[i][k]=el.type==='checkbox'?el.checked:(k==='name'?el.value:+el.value)}});box.appendChild(d)})}
function removeAttr(i){current.attributes.splice(i,1);renumber();renderAttrs()}
async function saveEndpoint(){renumber();current.endpoint=+$('endpoint').value;current.slave_id=current.endpoint;current.device_type=+$('deviceType').value;current.cluster_id=parseInt($('clusterId').value||'0',0)||0;current.enabled=true;for(const a of current.attributes){a.reference=+a.reference||0;a.address=pduFromReference(a.function,a.reference);if(!a.reference){alert('Every attribute needs a Modbus reference.');return}}try{await api('/api/endpoint',{method:'POST',body:JSON.stringify(current)});$('saveMsg').textContent='Saved successfully.';await refreshEndpoints()}catch(e){$('saveMsg').textContent='Save failed: '+e.message}}
async function deleteEndpoint(){if(!confirm('Delete endpoint '+$('endpoint').value+'?'))return;await api('/api/endpoint?ep='+$('endpoint').value,{method:'DELETE'});current={endpoint:+$('endpoint').value,enabled:false,device_type:9,cluster_id:0,attributes:[]};renderAttrs();await refreshEndpoints()}
async function refreshEndpoints(){let a=await api('/api/endpoints');$('endpointList').innerHTML=a.length?a.map(x=>`<div class="ep-card" onclick="selectEp(${x.endpoint})"><span class="badge">EP ${x.endpoint} • Slave ${x.endpoint}</span><h4 style="margin:8px 0 4px">${esc(x.device_name)}</h4><span class="muted">${x.attribute_count} attributes</span><button class="danger" style="float:right" onclick="event.stopPropagation();deleteSpecific(${x.endpoint})">Delete</button></div>`).join(''):'<div class="empty">No saved endpoints.</div>'}
function selectEp(ep){$('endpoint').value=ep;loadEndpoint()}
async function deleteSpecific(ep){if(!confirm('Delete endpoint '+ep+'?'))return;await api('/api/endpoint?ep='+ep,{method:'DELETE'});if(+$('endpoint').value===ep){current={endpoint:ep,enabled:false,device_type:9,cluster_id:0,attributes:[]};renderAttrs()}await refreshEndpoints()}
async function status(){try{let s=await api('/api/status');$('h2status').textContent='H2: '+(s.h2_joined?'joined':'not joined');$('h2status').className='status '+(s.h2_joined?'ok':'')}catch(e){$('h2status').textContent='H2: offline'}}
function esc(s){return String(s).replace(/[&<>"']/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[m]))}

/* ---------------- CSV / Excel-compatible configuration ---------------- */
const CSV_HEADERS=['format','version','endpoint','slave_id','enabled','device_type','device_name','cluster_id','attribute_enabled','attribute_id','function','address','reference','reg_count','data_type','word_order','packed_count','address2','address3','name'];
function csvEscape(v){v=v==null?'':String(v);return /[",\n\r]/.test(v)?'"'+v.replace(/"/g,'""')+'"':v}
function csvLine(row){return CSV_HEADERS.map(h=>csvEscape(row[h])).join(',')}
function makeRows(cfg){let rows=[];let attrs=cfg.attributes||[];if(!attrs.length){rows.push({format:'NUOS-BMS-Modbus-Configuration',version:1,endpoint:cfg.endpoint,slave_id:cfg.slave_id||cfg.endpoint,enabled:cfg.enabled!==false,device_type:cfg.device_type,device_name:cfg.device_name||'',cluster_id:cfg.cluster_id||'0xFCE0',attribute_enabled:'',attribute_id:'',function:'',address:'',reference:'',reg_count:'',data_type:'',word_order:'',name:''});return rows}attrs.forEach((a,i)=>rows.push({format:'NUOS-BMS-Modbus-Configuration',version:1,endpoint:cfg.endpoint,slave_id:cfg.slave_id||cfg.endpoint,enabled:cfg.enabled!==false,device_type:cfg.device_type,device_name:cfg.device_name||'',cluster_id:cfg.cluster_id||'0xFCE0',attribute_enabled:a.enabled!==false,attribute_id:a.attribute_id!=null?a.attribute_id:i,function:a.function,address:a.address!=null?a.address:pduFromReference(a.function,a.reference),reference:a.reference||referenceForFunction(a.function,a.address),reg_count:1,data_type:a.data_type,word_order:a.word_order||0,packed_count:a.packed_count||0,address2:a.address2||'',address3:a.address3||'',name:a.name||''}));return rows}
function downloadBlob(name,text){let blob=new Blob([text],{type:'text/csv;charset=utf-8;'}),a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=name;document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(a.href),1000)}
function downloadCurrentCSV(){let rows=makeRows(current);downloadBlob(`nuos_ep${current.endpoint||1}_modbus_config.csv`,CSV_HEADERS.join(',')+'\n'+rows.map(csvLine).join('\n')+'\n')}
async function downloadAllCSV(){try{let list=await api('/api/endpoints');let rows=[];for(const x of list){try{let cfg=await api('/api/endpoint?ep='+x.endpoint);rows.push(...makeRows(cfg))}catch(e){}}if(!rows.length){alert('No saved endpoint configurations found.');return}downloadBlob('NUOS_ALL_DEVICE_CONFIGURATIONS.csv',CSV_HEADERS.join(',')+'\n'+rows.map(csvLine).join('\n')+'\n')}catch(e){alert('CSV export failed: '+e.message)}}
function parseCSV(text){let rows=[],row=[],cell='',quoted=false;for(let i=0;i<text.length;i++){let c=text[i],n=text[i+1];if(quoted){if(c==='"'&&n==='"'){cell+='"';i++}else if(c==='"'){quoted=false}else cell+=c}else{if(c==='"')quoted=true;else if(c===','){row.push(cell);cell=''}else if(c==='\n'){row.push(cell);rows.push(row);row=[];cell=''}else if(c==='\r'){if(n==='\n'){}else{row.push(cell);rows.push(row);row=[];cell=''}}else cell+=c}}if(cell!==''||row.length){row.push(cell);rows.push(row)}return rows.filter(r=>r.some(v=>String(v).trim()!==''))}
function normHeader(s){return String(s||'').trim().toLowerCase().replace(/\s+/g,'_')}
function boolVal(v,def=true){let s=String(v??'').trim().toLowerCase();if(!s)return def;return !['0','false','no','off','disabled'].includes(s)}
function parseType(v){let s=String(v??'').trim().toUpperCase();if(s==='')return 1;if(/^\d+$/.test(s))return +s;return typeByName[s.replace(/\s*\(.*/, '')] ?? typeByName[s] ?? 1}
function parseFunction(v){let s=String(v??'').trim().toUpperCase();let m=s.match(/(?:FC|FUNCTION|FUNC|0*?)(0*\d+)/);if(m)return +m[1];if(/^\d+$/.test(s))return +s;return 4}
function num(v,d=0){let n=Number(String(v??'').trim());return Number.isFinite(n)?n:d}
function parseImportedRows(text){let matrix=parseCSV(text);if(!matrix.length)throw Error('CSV file is empty.');let h=matrix[0].map(normHeader),idx={};h.forEach((x,i)=>idx[x]=i);if(idx.endpoint==null)throw Error('CSV must contain an endpoint column.');let groups=new Map();for(let r=1;r<matrix.length;r++){let row=matrix[r],get=k=>idx[k]==null?'':row[idx[k]]??'';let ep=num(get('endpoint'),0);if(ep<1||ep>50)continue;if(!groups.has(ep))groups.set(ep,{endpoint:ep,slave_id:num(get('slave_id'),ep),enabled:boolVal(get('enabled'),true),device_type:num(get('device_type'),9),device_name:get('device_name'),cluster_id:get('cluster_id')||'0xFCE0',attributes:[]});let g=groups.get(ep);let attrId=get('attribute_id');if(attrId==='')continue;let fn=parseFunction(get('function'));let ref=num(get('reference'),0);let address=num(get('address'),pduFromReference(fn,ref));if(!ref)ref=referenceForFunction(fn,address);let rc=Math.max(1,num(get('reg_count'),1));let dt=parseType(get('data_type'));let packedCount=Math.max(0,num(get('packed_count'),0));let address2=num(get('address2'),0);let address3=num(get('address3'),0);let baseName=get('name')||('Attribute '+attrId);if(packedCount>=2){if(packedCount>3)throw Error(`Endpoint ${ep} attribute ${attrId}: packed_count must be 2 or 3.`);if(dt!==12)throw Error(`Endpoint ${ep} attribute ${attrId}: packed groups must use UINT48 (12).`);g.attributes.push({enabled:boolVal(get('attribute_enabled'),true),attribute_id:g.attributes.length,function:fn,address:address,address2:address2,address3:address3,reference:ref,reg_count:1,data_type:dt,word_order:num(get('word_order'),0),packed_count:packedCount,name:baseName});continue;}if(rc>1 && [9,10,11,12].includes(dt)){let suffix=rc===3?['R','Y','B']:Array.from({length:rc},(_,k)=>String(k+1));for(let k=0;k<rc;k++){let ar=address+k;g.attributes.push({enabled:boolVal(get('attribute_enabled'),true),attribute_id:g.attributes.length,function:fn,address:ar,reference:referenceForFunction(fn,ar),reg_count:1,data_type:dt,word_order:num(get('word_order'),0),packed_count:0,address2:0,address3:0,name:baseName+' '+suffix[k]});}}else{g.attributes.push({enabled:boolVal(get('attribute_enabled'),true),attribute_id:num(attrId,g.attributes.length),function:fn,address:address,reference:ref,reg_count:1,data_type:dt,word_order:num(get('word_order'),0),packed_count:0,address2:0,address3:0,name:baseName});}}return [...groups.values()]}
async function importCSVFile(ev){
 let file=ev.target.files?.[0];
 ev.target.value='';
 if(!file)return;
 try{
  let text=await file.text();
  let groups=parseImportedRows(text);
  if(!groups.length)throw Error('No valid endpoint rows found in CSV.');

  let count=0;
  for(const cfg of groups){
   const totalCount=cfg.attributes.length;
   const enabledCount=cfg.attributes.filter(a=>a.enabled!==false).length;


   const postCfg={
    endpoint:cfg.endpoint,
    slave_id:cfg.slave_id,
    enabled:cfg.enabled,
    device_type:cfg.device_type,
    cluster_id:num(cfg.cluster_id,0),
    attributes:cfg.attributes.map((a,i)=>({
     enabled:a.enabled!==false,
     attribute_id:i,
     function:+a.function,
     address:+a.address,
     reference:+a.reference,
     reg_count:+(a.reg_count||1),
     data_type:+a.data_type,
     word_order:+(a.word_order||0),
     packed_count:+(a.packed_count||0),
     address2:+(a.address2||0),
     address3:+(a.address3||0),
     name:a.name||('Attribute '+i)
    }))
   };

   await api('/api/endpoint',{
    method:'POST',
    body:JSON.stringify(postCfg)
   });
   count++;
  }

  let selected=groups.find(g=>g.endpoint===+$('endpoint').value)||groups[0];
  $('endpoint').value=selected.endpoint;
  await loadEndpoint();
  await refreshEndpoints();
  alert(`CSV import complete. ${count} endpoint(s) saved to flash and applied to the C6/H2 configuration.`);
 }catch(e){
  alert('CSV import failed: '+e.message);
 }
}
$('endpoint').onchange=()=>loadEndpoint().catch(e=>console.warn('Endpoint load failed',e));$('deviceType').onchange=()=>{};status();setInterval(status,3000);init();
