# -*- coding: utf-8 -*-
import tkinter as tk
from tkinter import ttk

def build_position_command_panel(app,parent,field_width_m,field_height_m,max_speed):
    """构建Tab1 Position Command。变量仍挂在app上，保持既有回调/API兼容。"""
    app.test_x=tk.DoubleVar(value=field_width_m/2); app.test_y=tk.DoubleVar(value=field_height_m/2)
    app.kp_var=tk.DoubleVar(value=5.0); app.max_speed_var=tk.DoubleVar(value=max_speed); app.stop_radius_var=tk.DoubleVar(value=0.05); app.send_hz_var=tk.DoubleVar(value=20)
    ttk.Label(parent,text='X').grid(row=0,column=0,padx=2,pady=2); ttk.Entry(parent,textvariable=app.test_x,width=5).grid(row=0,column=1,padx=2,pady=2)
    ttk.Label(parent,text='Y').grid(row=0,column=2,padx=2,pady=2); ttk.Entry(parent,textvariable=app.test_y,width=5).grid(row=0,column=3,padx=2,pady=2)
    ttk.Label(parent,text='Hz').grid(row=0,column=4,padx=2,pady=2); ttk.Entry(parent,textvariable=app.send_hz_var,width=5).grid(row=0,column=5,padx=2,pady=2)
    ttk.Button(parent,text='Go',command=app.start_closed_loop,width=6).grid(row=0,column=6,padx=2,pady=2); ttk.Button(parent,text='Return',command=app.start_centering,width=7).grid(row=0,column=7,padx=2,pady=2); ttk.Button(parent,text='Clear',command=app.clear_screen_trace,width=6).grid(row=0,column=8,padx=2,pady=2)
    ttk.Label(parent,text='Kp').grid(row=1,column=0,padx=2,pady=2); ttk.Entry(parent,textvariable=app.kp_var,width=5).grid(row=1,column=1,padx=2,pady=2)
    ttk.Label(parent,text='Max').grid(row=1,column=2,padx=2,pady=2); ttk.Entry(parent,textvariable=app.max_speed_var,width=5).grid(row=1,column=3,padx=2,pady=2)
    ttk.Label(parent,text='Stop').grid(row=1,column=4,padx=2,pady=2); ttk.Entry(parent,textvariable=app.stop_radius_var,width=5).grid(row=1,column=5,padx=2,pady=2)
    ttk.Label(parent,text='Strategy').grid(row=1,column=6,padx=2,pady=2)
    app.goto_strategy_box=ttk.Combobox(parent,textvariable=app.goto_strategy_var,values=['rcpd','p','max_then_slow','pd_damping','brake_distance','bang_bang'],state='readonly',width=14); app.goto_strategy_box.grid(row=1,column=7,columnspan=2,sticky='ew',padx=2,pady=2); app.goto_strategy_box.bind('<<ComboboxSelected>>',app.on_goto_strategy_changed)
    entries=[]
    for i,(lv,col) in enumerate(zip((app.goto_param1_label_var,app.goto_param2_label_var,app.goto_param3_label_var,app.goto_param4_label_var,app.goto_param5_label_var),(0,2,4,6,8))):
        ttk.Label(parent,textvariable=lv).grid(row=2,column=col,padx=2,pady=2); e=ttk.Entry(parent,width=6); e.grid(row=2,column=col+1,padx=2,pady=2); entries.append(e)
    app.goto_param1_entry,app.goto_param2_entry,app.goto_param3_entry,app.goto_param4_entry,app.goto_param5_entry=entries
    ttk.Label(parent,textvariable=app.goto_timer_var,foreground='#16a34a',font=('Segoe UI',10,'bold')).grid(row=3,column=0,columnspan=10,sticky='w',padx=3,pady=(0,2))
    app.update_strategy_param_ui()
