# -*- coding: utf-8 -*-
import math, tkinter as tk
from tkinter import ttk

class LD14Tab:
    """M100风格LD14页面：无辉光、无扫描线、无CSV导出。"""
    def __init__(self,parent,service,guard,config):
        self.parent=parent; self.service=service; self.guard=guard; self.config=config
        self.online=tk.StringVar(value='LD14: waiting'); self.stats=tk.StringVar(value='Frames 0 | Bad 0')
        self.nearest=tk.StringVar(value='Nearest: --'); self.protection=tk.BooleanVar(value=config.collision_stop_enabled)
        self._build()
    def _build(self):
        self.parent.columnconfigure(0,weight=1); self.parent.columnconfigure(1,weight=0); self.parent.rowconfigure(0,weight=1)
        box=ttk.LabelFrame(self.parent,text='LD14 Radar'); box.grid(row=0,column=0,sticky='nsew',padx=10,pady=10)
        self.canvas=tk.Canvas(box,bg='#101820',highlightthickness=0); self.canvas.pack(fill='both',expand=True,padx=6,pady=6)
        side=ttk.Frame(self.parent,width=280); side.grid(row=0,column=1,sticky='ns',padx=(0,10),pady=10)
        st=ttk.LabelFrame(side,text='Radar Status'); st.pack(fill='x',pady=(0,8))
        for var in (self.online,self.stats,self.nearest): ttk.Label(st,textvariable=var,font=('Segoe UI',10)).pack(anchor='w',padx=8,pady=4)
        pr=ttk.LabelFrame(side,text='Protective Stop'); pr.pack(fill='x')
        ttk.Checkbutton(pr,text='Enable collision stop',variable=self.protection,command=self._apply).pack(anchor='w',padx=8,pady=6)
        self.stop_var=tk.DoubleVar(value=self.config.stop_distance_m); self.release_var=tk.DoubleVar(value=self.config.release_distance_m); self.angle_var=tk.DoubleVar(value=self.config.front_half_angle_deg)
        for label,var in [('Stop m',self.stop_var),('Release m',self.release_var),('Half angle',self.angle_var)]:
            row=ttk.Frame(pr); row.pack(fill='x',padx=8,pady=3); ttk.Label(row,text=label,width=12).pack(side='left'); ttk.Entry(row,textvariable=var,width=8).pack(side='left')
        ttk.Button(pr,text='Apply',command=self._apply).pack(fill='x',padx=8,pady=8)
        ttk.Label(pr,text='LD14失联默认不停车；可在m100_config.py启用严格模式。',wraplength=245,foreground='#64748b').pack(anchor='w',padx=8,pady=(0,8))
    def _apply(self):
        self.config.collision_stop_enabled=bool(self.protection.get())
        try:
            self.config.stop_distance_m=max(0.05,float(self.stop_var.get())); self.config.release_distance_m=max(self.config.stop_distance_m+0.01,float(self.release_var.get())); self.config.front_half_angle_deg=max(5.0,min(180.0,float(self.angle_var.get())))
        except Exception: pass
    def refresh(self):
        snap=self.service.snapshot(); self.online.set('LD14: ONLINE' if snap.online else 'LD14: OFFLINE')
        self.stats.set(f'Frames {snap.good_frames} | Bad {snap.bad_packets} | {snap.speed_dps/360.0:.2f} Hz')
        near=self.guard.nearest_m; self.nearest.set('Nearest: --' if near is None else f'Nearest: {near:.3f} m' + ('  STOP' if self.guard.latched else ''))
        cv=self.canvas; cv.delete('all'); w=max(100,cv.winfo_width()); h=max(100,cv.winfo_height()); cx=w/2; cy=h/2; r=max(20,min(w,h)*0.44); max_m=3.0
        for frac in (0.25,0.5,0.75,1.0):
            rr=r*frac; cv.create_oval(cx-rr,cy-rr,cx+rr,cy+rr,outline='#334155'); cv.create_text(cx+4,cy-rr+3,text=f'{max_m*frac:.1f}m',fill='#94a3b8',anchor='nw',font=('Segoe UI',8))
        cv.create_line(cx-r,cy,cx+r,cy,fill='#334155'); cv.create_line(cx,cy-r,cx,cy+r,fill='#334155'); cv.create_text(cx,cy-r-12,text='FRONT 0°',fill='#e2e8f0',font=('Segoe UI',9,'bold'))
        a=math.radians(self.config.front_half_angle_deg)
        for s in (-1,1): cv.create_line(cx,cy,cx+math.sin(s*a)*r,cy-math.cos(s*a)*r,fill='#f59e0b',dash=(4,3))
        for p in snap.points:
            if p.distance_m<=0 or p.distance_m>max_m: continue
            # LD14 0度绘制为车头向上，顺时针为屏幕右侧
            rad=math.radians(p.angle_deg); rr=min(r,p.distance_m/max_m*r); x=cx+math.sin(rad)*rr; y=cy-math.cos(rad)*rr
            color='#ef4444' if p.distance_m<=self.config.stop_distance_m else '#38bdf8'; cv.create_oval(x-2,y-2,x+2,y+2,fill=color,outline='')
        cv.create_polygon(cx,cy-10,cx-7,cy+8,cx+7,cy+8,fill='#22c55e',outline='#bbf7d0')
