#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse,csv,math,os,signal,statistics,threading,time,sys
from collections import defaultdict,deque
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import cv2,numpy as np
from app.ld14 import Parser

class Map:
    def __init__(self,args):
        self.a=args; self.n=max(1,round(360/args.angle_bin)); self.h=defaultdict(lambda:deque(maxlen=args.history)); self.latest={}; self.lock=threading.Lock()
    def add(self,points):
        with self.lock:
            for p in points:
                if self.a.min_distance<=p.distance_m<=self.a.max_range and p.confidence>=self.a.min_confidence:
                    ang=(p.angle_deg+self.a.angle_offset)%360; b=round(ang/self.a.angle_bin)%self.n; self.h[b].append((p.distance_m,p.confidence)); self.latest[b]=(p.distance_m,p.confidence,time.monotonic())
    def stable(self):
        with self.lock:return [(b*self.a.angle_bin,statistics.median(x[0] for x in v),round(statistics.median(x[1] for x in v))) for b,v in self.h.items() if len(v)>=self.a.min_hits]
    def recent(self):
        now=time.monotonic()
        with self.lock:return [(b*self.a.angle_bin,d,c) for b,(d,c,t) in self.latest.items() if now-t<.5]

def draw(m,stats):
    a=m.a;s=a.size;c=(s//2,s//2);r=int(s*.4);im=np.full((s,s,3),(248,250,252),np.uint8)
    for f in (.25,.5,.75,1):
        rr=int(r*f);cv2.circle(im,c,rr,(205,215,226),1);cv2.putText(im,f'{a.max_range*f:.1f}m',(c[0]+4,c[1]-rr+15),0,.43,(100,116,139),1)
    cv2.line(im,(c[0]-r,c[1]),(c[0]+r,c[1]),(210,219,230),1);cv2.line(im,(c[0],c[1]-r),(c[0],c[1]+r),(210,219,230),1)
    cv2.putText(im,'FRONT 0 deg',(c[0]-48,c[1]-r-12),0,.48,(30,41,59),1)
    def xy(p):
        ang,d,_=p;z=math.radians(ang);rr=min(r,d/a.max_range*r);return round(c[0]+math.sin(z)*rr),round(c[1]-math.cos(z)*rr)
    stable=sorted(m.stable());prev=None
    for p in stable:
        q=xy(p)
        if prev and (p[0]-prev[0][0])%360<=a.angle_bin*2.5 and abs(p[1]-prev[0][1])<=.35:cv2.line(im,prev[1],q,(68,129,235),2)
        cv2.circle(im,q,3,(37,99,235),-1);prev=(p,q)
    for p in m.recent():cv2.circle(im,xy(p),2,(0,146,255),-1)
    cv2.fillConvexPoly(im,np.array([[c[0],c[1]-12],[c[0]-9,c[1]+10],[c[0]+9,c[1]+10]],np.int32),(38,181,111))
    cv2.putText(im,'LD14 SDK-CORRECTED ROOM MAP',(20,30),0,.65,(15,35,65),2)
    cv2.putText(im,f"frames={stats['frames']} fps={stats['fps']:.1f} rotation={stats['hz']:.2f}Hz crc_bad={stats['crc']}",(20,55),0,.43,(71,85,105),1)
    cv2.putText(im,f'angle_offset={a.angle_offset:.1f} deg  rear ignore is applied in main guard, not hidden here',(20,76),0,.43,(71,85,105),1)
    return im,stable

PAGE="""<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width"><style>body{margin:0;background:#eef2f7;text-align:center;font-family:sans-serif}main{max-width:900px;margin:auto;padding:12px}img{width:100%;border-radius:18px}</style><main><h3>LD14 SDK修正房间点云</h3><p>0度朝车头，蓝色稳定轮廓，橙色最近0.5秒点。</p><img id="m" src="/map.png"><script>setInterval(()=>m.src='/map.png?t='+Date.now(),1000)</script></main>"""
def handler(path):
 class H(BaseHTTPRequestHandler):
  def do_GET(self):
   if self.path.startswith('/map.png'):
    try:data=Path(path).read_bytes();self.send_response(200);self.send_header('Content-Type','image/png')
    except FileNotFoundError:self.send_error(404);return
   else:data=PAGE.encode();self.send_response(200);self.send_header('Content-Type','text/html;charset=utf-8')
   self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
  def log_message(self,*_):pass
 return H

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--port',default='/dev/ttyS6');ap.add_argument('--baud',type=int,default=115200);ap.add_argument('--seconds',type=float,default=0);ap.add_argument('--max-range',type=float,default=5);ap.add_argument('--min-distance',type=float,default=.08);ap.add_argument('--min-confidence',type=int,default=10);ap.add_argument('--angle-bin',type=float,default=1);ap.add_argument('--history',type=int,default=9);ap.add_argument('--min-hits',type=int,default=3);ap.add_argument('--angle-offset',type=float,default=174);ap.add_argument('--size',type=int,default=900);ap.add_argument('--output',default='ld14_room_map');ap.add_argument('--web-port',type=int,default=8090);ap.add_argument('--no-sdk-correction',action='store_true');a=ap.parse_args()
 try:
  import serial
 except ImportError as exc:
  raise SystemExit('缺少pyserial，请使用项目.venv运行或安装pyserial') from exc
 out=Path(a.output).resolve();png=out.with_suffix('.png');csvp=out.with_suffix('.csv');m=Map(a);parser=Parser(not a.no_sdk_correction);run=True;stats={'frames':0,'fps':0,'hz':0,'crc':0};last=time.monotonic();count=0;start=last
 def stop(*_):nonlocal run;run=False
 signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
 server=None
 if a.web_port:server=ThreadingHTTPServer(('0.0.0.0',a.web_port),handler(png));threading.Thread(target=server.serve_forever,daemon=True).start();print(f'网页: http://3W-IP:{a.web_port}')
 with serial.Serial(a.port,a.baud,timeout=.1) as dev:
  while run and (a.seconds<=0 or time.monotonic()-start<a.seconds):
   for points in parser.feed(dev.read(2048)):m.add(points);stats['frames']+=1;count+=1
   now=time.monotonic()
   if now-last>=1:stats['fps']=count/(now-last);count=0;last=now;stats['crc']=parser.bad_crc;print(stats)
   im,pts=draw(m,stats);tmp=str(png)+'.tmp.png';cv2.imwrite(tmp,im);os.replace(tmp,png)
   with open(csvp,'w',newline='',encoding='utf-8-sig') as f:w=csv.writer(f);w.writerow(['vehicle_angle_deg','distance_m','confidence']);w.writerows(pts)
 if server:server.shutdown()
if __name__=='__main__':main()
