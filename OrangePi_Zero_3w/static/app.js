const $=q=>document.querySelector(q);
let joy= {
  x:0,y:0
},turn=0,settings= {
},activeTab='vision',lastStatus=null,samplingColor=false;
const post=(url,data= {
})=>fetch(url, {
  method:'POST',headers: {
    'Content-Type':'application/json'
  },body:JSON.stringify(data)
});
const uiStyle=document.createElement('style');
uiStyle.textContent=`
.detectorSegment{grid-column:2/4;display:grid;grid-template-columns:1fr 1fr;gap:3px;padding:3px;border-radius:13px;background:#e7edf5}
.detectorSegment button{height:36px;border-radius:10px;background:transparent;color:#60728b;font-weight:800}
.detectorSegment button.active{background:#fff;color:var(--blue);box-shadow:0 3px 11px rgba(34,68,108,.12)}
.segmentRow{grid-template-columns:132px 1fr}
.statusChip.batteryUnknown .icon{color:#94a3b8}
.statusChip.batteryLow .icon{color:var(--orange)}
@media(max-width:380px){.segmentRow{grid-template-columns:112px 1fr}}
`;
document.head.appendChild(uiStyle);
function send() {
  post('/api/control', {
    x:joy.x,y:joy.y,turn,throttle:+$('#gas').value/100
  }).catch(()=> {
  })
}
setInterval(send,100);
const joyArea=$('#joy'),knob=$('#knob');
function moveJoy(e) {
  const r=joyArea.getBoundingClientRect();
  let x=(e.clientX-r.left-r.width/2)/(r.width/2),y=(e.clientY-r.top-r.height/2)/(r.height/2),m=Math.hypot(x,y);
  if(m>1) {
    x/=m;
    y/=m
  }
  joy= {
    x,y
  };
  knob.style.transform=`translate(${x*r.width*.31}px,${y*r.height*.31}px)`
}
joyArea.onpointerdown=e=> {
  joyArea.setPointerCapture(e.pointerId);
  moveJoy(e)
};
joyArea.onpointermove=e=> {
  if(e.buttons)moveJoy(e)
};
joyArea.onpointerup=joyArea.onpointercancel=()=> {
  joy= {
    x:0,y:0
  };
  knob.style.transform=''
};
function hold(selector,value) {
  const b=$(selector);
  b.onpointerdown=e=> {
    b.setPointerCapture(e.pointerId);
    turn=value
  };
  b.onpointerup=b.onpointercancel=()=>turn=0
}
hold('#left',-1);
hold('#right',1);
function setThrottle(value) {
  const v=Math.max(0,Math.min(100,Math.round(+value/10)*10));
  $('#gas').value=v;
  $('#gp').textContent=v+'%';
  $('#gas').style.setProperty('--fill',v+'%')
}
$('#gas').oninput=e=>setThrottle(e.target.value);
setThrottle(40);
$('#stateBtn').onclick=()=>post('/api/arm').then(async r=> {
  const d=await r.json();
  if(!d.ok)alert(d.message)
}).catch(()=> {
});
$('#stop').onclick=()=> {
  joy= {
    x:0,y:0
  };
  turn=0;
  knob.style.transform='';
  setThrottle(0);
  post('/api/stop')
};
$('#shutdown').onclick=()=> {
  if(!confirm('关闭 Orange Pi？小车将立即 STOP。'))return;
  if(!confirm('再次确认关机。'))return;
  post('/api/shutdown').then(r=>r.json()).then(d=> {
    if(!d.ok)alert(d.message)
  })
};
$('#networkBtn').onclick=()=>$('#networkModal').classList.remove('hidden');
$('#lidarBtn').onclick=()=> {
  if(!lastStatus?.lidar_online)return;
  const disabling=lastStatus.lidar_control_enabled!==false;
  const message=disabling?'禁用雷达保护？雷达仍会接收数据，但不再触发BLOCKED。':'重新启用雷达保护？';
  if(!confirm(message))return;
  post('/api/lidar/toggle').then(async r=> {
    const d=await r.json();
    if(!d.ok)alert(d.message)
  }).catch(()=> {
  });
};
document.querySelectorAll('[data-net]').forEach(b=>b.onclick=()=> {
  const target=b.dataset.net,name=target==='ap'?'3W 热点':'家庭 Wi-Fi';
  if(confirm(`切换到${name}？当前页面可能断开。`))post('/api/network', {
    target
  }).then(r=>r.json()).then(d=> {
    if(!d.ok)alert(d.message)
  })
});
document.querySelectorAll('.close').forEach(b=>b.onclick=()=>b.closest('.modal').classList.add('hidden'));
const definitions= {
  vision: {
    title:'视觉识别',rows:[
    ['detector','检测器','segment',[{value:'yolo',label:'YOLO'},{value:'hsv',label:'HSV'}]],
    ['yolo_model','YOLO 模型','segment',[{value:'yolo/models/yolov5n_ball_uint8.nb',label:'YOLOv5n'},{value:'yolo/models/yolov5s_ball_uint8.nb',label:'YOLOv5s'}]],
    ['h_center','H 中心',0,179,1],['s_center','S 中心',0,255,1],['v_center','V 中心',0,255,1],
    ['h_range','H 范围',0,60,1],['s_range','S 范围',0,255,1],['v_range','V 范围',0,255,1],
    ['min_area','最小面积',0,500,5],['max_area','最大面积',100,50000,100],
    ['min_circularity','最小圆度',0,1,.01],['morphology_kernel','形态核',1,31,2],
    ['draw_mask_preview','掩膜预览','bool']
    ]
  },
  lateral_tracking: {
    title:'X 横向控制',rows:[
    ['enabled','启用 X 控制','bool'],['kp','比例增益 P',0,1.5,.01],['kd','微分增益 D',0,.5,.005],
    ['center_deadband_norm','中心死区',0,.15,.005],['error_filter_alpha','误差滤波',.05,1,.05],
    ['min_vy','最小速度',0,1.4,.01],['max_vy','最大速度',.05,1.4,.01],
    ['fast_error_norm','高速触发误差',.02,1,.01],['fast_vy','高速区速度',.05,1.4,.01],
    ['max_accel_vy_per_s','最大加速度',.1,10,.1],['acquire_confirm_frames','目标确认帧数',1,10,1],
    ['lost_timeout_ms','丢球超时',50,1000,10],['image_error_to_body_y_sign','横移方向',-1,1,2]
    ]
  },
  oac: {
    title:'Y 预测式 OAC',rows:[
    ['enabled','启用 Y / OAC','bool'],['kp','位置反馈 P',0,5,.01],['kd','速度前馈',0,3,.01],
    ['catch_y_norm','纵向接球线',.05,.95,.01],['lookahead_s','预测前视秒',0,.6,.01],
    ['position_deadband_norm','位置死区',0,.3,.005],['fast_error_norm','高速触发误差',.01,.8,.01],
    ['min_vx','最小速度',0,1.4,.01],['max_vx','最大速度',0,1.4,.01],
    ['rate_filter_alpha','视线速度滤波',.01,1,.01],['min_upward_rate','向上判定速率',0,1,.005],
    ['fast_vx','高速区速度',0,1.4,.01],['max_accel_vx_per_s','最大加速度',.1,20,.1],
    ['lost_timeout_ms','丢球超时',50,1000,10],['dropout_hold_ms','漏检保持ms',0,500,10],
    ['overtop_enabled','顶部续冲','bool'],['overtop_y_norm','顶部区域',.02,.5,.01],
    ['overtop_hold_ms','续冲时间ms',0,1000,10],['overtop_vx','续冲速度',0,1.4,.01],
    ['output_sign','前后方向',-1,1,2]
    ]
  },
  imu: {
    title:'W 航向控制',rows:[
    ['yaw_hold_enabled','启用 W 保持','bool'],['required_for_arm','ARM 需要 IMU','bool'],
    ['yaw_kp_radps_per_deg','比例增益 P',0,.08,.001],['yaw_kd_radps_per_dps','微分增益 D',0,.03,.001],
    ['yaw_deadband_deg','航向死区',0,10,.1],['yaw_max_wz_radps','最大角速度',.05,1,.01],
    ['yaw_sign','纠偏方向',-1,1,2],['timeout_ms','IMU 超时',100,1500,50]
    ]
  }
};
function clone(v) {
  return JSON.parse(JSON.stringify(v))
}
function updateRangeFill(input) {
  const min=Number(input.min||0),max=Number(input.max||1),value=Number(input.value||0);
  const pct=max>min?Math.max(0,Math.min(100,(value-min)*100/(max-min))):0;
  input.style.setProperty('--fill',pct+'%');
}
function renderSettings() {
  const def=definitions[activeTab],data=settings[activeTab]|| {
  };
  const rows=def.rows.map(d=> {
    const [k,label,typeOrMin,max,step]=d;
    if(typeOrMin==='bool')return `<label class="settingRow toggleRow"><span>${label}</span><input type="checkbox" data-section="${activeTab}" data-key="${k}" ${data[k]?'checked':''}></label>`;
    if(typeOrMin==='segment') {
      const options=max.map(option=>typeof option==='string'?{value:option,label:option.toUpperCase()}:option);
      const fallback=options[0]?.value??'';
      const value=String(data[k]??fallback);
      return `<label class="settingRow segmentRow"><span>${label}</span><div class="detectorSegment" data-section="${activeTab}" data-key="${k}">${options.map(option=>`<button type="button" data-value="${option.value}" class="${option.value===value?'active':''}">${option.label}</button>`).join('')}</div></label>`;
    }
    const value=data[k]??0;
    return `<label class="settingRow"><span>${label}</span><input type="range" data-section="${activeTab}" data-key="${k}" min="${typeOrMin}" max="${max}" step="${step}" value="${value}"><output>${value}</output></label>`;
  }).join('');
  $('#settingsRows').innerHTML=`<section class="settingGroup"><div class="settingGroupTitle">${def.title}</div>${rows}</section>`;
  document.querySelectorAll('.settingRow input').forEach(i=> {
    if(i.type==='range')updateRangeFill(i);
    i.oninput=()=> {
      const v=i.type==='checkbox'?i.checked:+i.value;
      settings[i.dataset.section][i.dataset.key]=v;
      if(i.type==='range') {
        updateRangeFill(i);
        if(i.nextElementSibling)i.nextElementSibling.value=i.value;
      }
    };
  });
  document.querySelectorAll('.detectorSegment').forEach(group=>group.querySelectorAll('button').forEach(button=>button.onclick=()=> {
    settings[group.dataset.section][group.dataset.key]=button.dataset.value;
    group.querySelectorAll('button').forEach(item=>item.classList.toggle('active',item===button));
  }));
}
document.querySelectorAll('[data-tab]').forEach(b=>b.onclick=()=> {
  activeTab=b.dataset.tab;
  document.querySelectorAll('[data-tab]').forEach(x=>x.classList.toggle('active',x===b));
  renderSettings()
});
$('#settingsBtn').onclick=()=>post('/api/settings/open').then(async r=> {
  const d=await r.json();
  if(!d.ok) {
    alert(d.message);
    return
  }
  settings=clone(d.settings);
  activeTab='vision';
  document.querySelectorAll('[data-tab]').forEach(x=>x.classList.toggle('active',x.dataset.tab===activeTab));
  renderSettings();
  $('#settingsModal').classList.remove('hidden')
});
$('#applySettings').onclick=()=>post('/api/settings/apply',settings).then(r=>r.json()).then(d=> {
  if(d.ok) {
    settings=d.settings
  } else alert(d.message)
});
$('#saveSettings').onclick=()=>post('/api/settings/save',settings).then(r=>r.json()).then(d=> {
  if(d.ok) {
    settings=d.settings;
    alert('已保存')
  } else alert(d.message)
});
function setSampling(on) {
  samplingColor=on;
  $('#sampleColorBtn').classList.toggle('active',on);
  $('.cameraCard').classList.toggle('sampling',on);
  $('#sampleHint').classList.toggle('hidden',!on)
}
$('#sampleColorBtn').onclick=()=>setSampling(!samplingColor);
$('#video').onclick=e=> {
  if(!samplingColor)return;
  const r=e.currentTarget.getBoundingClientRect(),fw=lastStatus?.frame_width,fh=lastStatus?.frame_height;
  if(!fw||!fh)return;
  const ir=fw/fh,br=r.width/r.height;
  let dw,dh,ox,oy;
  if(ir>br) {
    dw=r.width;
    dh=dw/ir;
    ox=0;
    oy=(r.height-dh)/2
  } else {
    dh=r.height;
    dw=dh*ir;
    oy=0;
    ox=(r.width-dw)/2
  }
  const x=e.clientX-r.left-ox,y=e.clientY-r.top-oy;
  if(x<0||y<0||x>dw||y>dh)return;
  post('/api/sample_hsv', {
    x:x/dw,y:y/dh
  }).then(r=>r.json()).then(d=> {
    if(d.ok) {
      settings.vision=settings.vision|| {
      };
      Object.assign(settings.vision, {
        h_center:d.value.h,s_center:d.value.s,v_center:d.value.v
      });
      setSampling(false);
      renderSettings();
      $('#settingsModal').classList.remove('hidden')
    }
  })
};
const modeClass= {
  SAFE:'safe',MANUAL:'manual',ARMED:'armed',TRACKING:'armed',AUTO_MOVE:'armed',BLOCKED:'blocked',SUCCESS:'armed',FAULT:'fault',BOOT:'manual',SHUTTING_DOWN:'fault'
};
function chip(id,state) {
  const el=$(id).closest('.statusChip');
  el.classList.remove('ok','warn','bad','hotspot','bypass');
  if(state)el.classList.add(state)
}
function stat() {
  fetch('/api/status').then(r=>r.json()).then(s=> {
    lastStatus=s;
    $('#mode').textContent=s.mode;
    const stateBtn=$('#stateBtn');
    stateBtn.className=`actionButton state ${modeClass[s.mode]||'fault'}`;
    $('#network').textContent=s.network==='ap'?'热点':s.network==='wifi'?'Wi-Fi':'--';
    chip('#network',s.network==='wifi'?'ok':s.network==='ap'?'hotspot':'bad');
    $('#esp').textContent=s.transport_ok?'ESP32':'--';
    chip('#esp',s.transport_ok?'ok':'bad');
    $('#cam').textContent=s.camera_ok?Math.round(Number(s.fps||0))+' FPS':'--';
    chip('#cam',s.camera_ok?'ok':'bad');
    $('#yaw').textContent=s.imu_ok?Number(s.yaw_rel||0).toFixed(1)+'°':'--°';
    chip('#yaw',s.imu_ok?'ok':'bad');
    const lidarBtn=$('#lidarBtn');
    const lidarBypassed=s.lidar_online&&s.lidar_control_enabled===false;
    $('#lidar').textContent=!s.lidar_online?'OFF':lidarBypassed?'BYPASS':s.lidar_blocked?'BLOCK':s.lidar_nearest!=null?Number(s.lidar_nearest).toFixed(2)+'m':s.lidar_clear?'CLEAR':'--';
    lidarBtn.disabled=!s.lidar_online;
    chip('#lidar',!s.lidar_online?'bad':lidarBypassed?'bypass':s.lidar_blocked?'warn':'ok');
    const batteryPct=s.battery_pct==null?null:Math.max(0,Math.min(100,Number(s.battery_pct)));
    $('#battery').textContent=batteryPct==null?'--%':Math.round(batteryPct)+'%';
    const batteryFill=$('#batteryFill');
    if(batteryFill)batteryFill.setAttribute('width',batteryPct==null?0:(18*batteryPct/100).toFixed(2));
    chip('#battery',batteryPct==null?'bad':batteryPct<20?'warn':'ok');
    $('#cameraInfo').textContent=`${s.frame_width||'--'} × ${s.frame_height||'--'}`;
    $('#streamFps').textContent=`预览上限 ${Number(s.preview_fps||24).toFixed(0)} FPS`;
    $('#ballInfo').textContent=s.ball?`球 ${Math.round(s.ball.cx)}, ${Math.round(s.ball.cy)}`:'球 --'
  }).catch(()=> {
  })
}
setInterval(stat,350);
stat();
