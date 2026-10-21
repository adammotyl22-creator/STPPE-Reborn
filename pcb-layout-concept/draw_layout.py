from pathlib import Path
import math
from html import escape
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
W, H = 1700, 1450
S, OX, OY = 7, 295, 210
BG, INK, MUTED = '#f5f7fb', '#172b40', '#53677e'
BLUE, RED, GREEN, AMBER = '#2467b3', '#bf3f50', '#187a76', '#b57819'
im = Image.new('RGB', (W,H), BG)
d = ImageDraw.Draw(im)
svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">', f'<rect width="{W}" height="{H}" fill="{BG}"/>']
FONT = Path('C:/Windows/Fonts/segoeui.ttf')
BOLD = Path('C:/Windows/Fonts/segoeuib.ttf')
def font(size, bold=False): return ImageFont.truetype(str(BOLD if bold else FONT), size)
def text(x,y,t,size=22,fill=INK,bold=False,anchor='start'):
    svg.append(f'<text x="{x}" y="{y}" fill="{fill}" font-family="Segoe UI,Arial,sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" text-anchor="{anchor}">{escape(t)}</text>')
    d.text((x,y),t,font=font(size,bold),fill=fill,anchor={'start':'ls','middle':'ms','end':'rs'}[anchor])
def line(x1,y1,x2,y2,fill=INK,width=2,dash=False):
    svg.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{fill}" stroke-width="{width}"'+(' stroke-dasharray="8 6"' if dash else '')+'/>')
    if dash:
        n=max(1,int(math.hypot(x2-x1,y2-y1)/8))
        for i in range(0,n,2):
            a,b=i/n,min(i+1,n)/n
            d.line((x1+(x2-x1)*a,y1+(y2-y1)*a,x1+(x2-x1)*b,y1+(y2-y1)*b),fill=fill,width=width)
    else: d.line((x1,y1,x2,y2),fill=fill,width=width)
def rect(x,y,w,h,fill,stroke=INK,width=2,dash=False,r=0):
    svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"'+(' stroke-dasharray="8 6"' if dash else '')+'/>')
    d.rounded_rectangle((x,y,x+w,y+h),radius=r,fill=fill,outline=None if dash else stroke,width=width)
    if dash:
        for a,b,c,e in [(x,y,x+w,y),(x+w,y,x+w,y+h),(x+w,y+h,x,y+h),(x,y+h,x,y)]:line(a,b,c,e,stroke,width,True)
def circle(x,y,r,fill,stroke=INK,width=2):
    svg.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>')
    d.ellipse((x-r,y-r,x+r,y+r),fill=fill,outline=stroke,width=width)
def arrow(x1,y1,x2,y2,fill,width=3):
    line(x1,y1,x2,y2,fill,width)
    a=math.atan2(y2-y1,x2-x1)
    for b in [a+2.65,a-2.65]:line(x2,y2,x2+13*math.cos(b),y2+13*math.sin(b),fill,width)
def pt(x,y):return OX+x*S,OY+y*S
def box(x,y,w,h,fill,stroke=INK,dash=False):rect(*pt(x,y),w*S,h*S,fill,stroke,2,dash)
def mmtext(x,y,t,size=18,fill=INK,bold=False,anchor='middle'):text(*pt(x,y),t,size,fill,bold,anchor)
def dim(x1,y1,x2,y2,label,tx,ty):
    line(x1,y1,x2,y2,MUTED,2)
    if y1==y2:
        for x in [x1,x2]:line(x,y1-7,x,y1+7,MUTED)
    else:
        for y in [y1,y2]:line(x1-7,y,x1+7,y,MUTED)
    text(tx,ty,label,21,MUTED,anchor='middle')

# Nominal body geometry in millimetres; reserved envelopes are deliberately explicit.
bodies=[('Hub8',7,7,31,18.3),('XIAO',7,38,21,17.8),('LM2596',57,7,43,21),('24V IN',83,0,16,6),('protection',57,31,43,8),('Teensy',-23,76,60.96,17.78)]
drivers=[(f'M{i+1}',x,y,15.24,20.32) for i,(x,y) in enumerate(( (x,y) for y in (52,84) for x in (44,64.32,84.64)))]
ports=[(f'PORT{i+1}',3+17*i,109,15,10) for i in range(6)]
caps=[(x,y,4) for y in (46,78) for x in (51.62,71.94,92.26)]
all_rects=bodies+drivers+ports
for i,(name,x,y,w,h) in enumerate(all_rects):
    assert y>=0 and y+h<=121 and x+w<=104.5, name
    assert x>=0 or (name=='Teensy' and x>=-24 and y>=71.2), name
    for other,xx,yy,ww,hh in all_rects[i+1:]:
        assert x+w<=xx or xx+ww<=x or y+h<=yy or yy+hh<=y, (name,other)
for x,y,r in caps:
    assert 0<=x-r and x+r<=104.5 and 0<=y-r and y+r<=121
    for name,xx,yy,w,h in all_rects:
        nearest_x=max(xx,min(x,xx+w)); nearest_y=max(yy,min(y,yy+h))
        assert math.hypot(x-nearest_x,y-nearest_y)>=r, name
for i,(x,y,r) in enumerate(caps):
    for xx,yy,rr in caps[i+1:]:assert math.hypot(x-xx,y-yy)>=r+rr
assert len(drivers)==len(ports)==len(caps)==6
assert math.isclose(109-(84+20.32),4.68)
assert math.isclose(44-(-23+60.96),6.04)
assert math.isclose(64.32-(44+15.24),5.08)
assert math.isclose(3+17*5+15,103)
assert math.isclose(52-(46+4),2)
assert math.isclose((78-4)-(52+20.32),1.68)

text(65,67,'KONCEPCJA ROZMIESZCZENIA • widok z góry',37,bold=True)
text(65,108,'Robot • 6 × TMC2209 • Teensy 4.1 • wymiary w mm',23,MUTED)
text(65,145,'Kontur występu, StepStick, LM2596 i złącza wymagają pomiaru',22,RED)
# The tongue is a sampled quadratic approximation of the supplied provisional contour.
tongue=[pt(0,59.2)]
for i in range(1,21):
    t=i/20;tongue.append(pt(-12*t*t,59.2*(1-t)**2+2*71.2*(1-t)*t+71.2*t*t))
tongue += [pt(-24,71.2),pt(-24,114)]
for i in range(1,21):
    t=i/20;tongue.append(pt(-24*(1-t)**2-48*(1-t)*t-17*t*t,114*(1-t)**2+242*(1-t)*t+121*t*t))
tongue += [pt(0,121)]
svg.append('<polygon points="'+' '.join(f'{x},{y}' for x,y in tongue)+'" fill="#e7edf4"/>')
d.polygon(tongue,fill='#e7edf4')
box(0,0,104.5,121,'#ffffff','#506b75')
for a,b in zip(tongue,tongue[1:]):line(*a,*b,MUTED,2,True)
# Only the measured main rectangle is solid; left shoulder below the tongue root is omitted.
line(*pt(0,59.2),*pt(0,121),'#d0dbe1',2,True)
dim(OX,185,OX+104.5*S,185,'104,5',OX+52.25*S,174)
dim(OX+104.5*S+35,OY,OX+104.5*S+35,OY+121*S,'121',OX+104.5*S+66,OY+62*S)
dim(OX-45,OY,OX-45,OY+71.2*S,'71,2',OX-79,OY+36*S)
text(145,990,'Występ: obrys',18,MUTED)
text(145,1015,'orientacyjny',18,MUTED)

box(7,7,31,18.3,'#d4efea',GREEN)
mmtext(22.5,14.5,'KAmod',19,GREEN,True)
mmtext(22.5,20,'I2C Hub8',19,GREEN,True)
box(7,38,21,17.8,'#d4efea',GREEN)
mmtext(17.5,44,'Seeed XIAO',16,GREEN,True)
mmtext(17.5,49,'ESP32C3',17,GREEN,True)
# Antenna cable is illustrative, not a PCB antenna keepout.
line(*pt(7,52),*pt(-9,52),GREEN,3)
line(*pt(-9,52),*pt(-14,48),GREEN,3)
circle(*pt(-14,48),5,'#ffffff',GREEN)
text(68,617,'Antena zewnętrzna',19,GREEN)
text(68,642,'poza metalową obudową',18,GREEN)
line(150,600,*pt(-14,48),GREEN,1)
box(57,7,43,21,'#fff0d4',AMBER)
mmtext(78.5,15,'LM2596 • moduł',19,AMBER,True)
mmtext(78.5,22,'24 V → 5 V',23,AMBER,True)
box(83,0,16,6,'#f5d9a9',AMBER,True)
mmtext(91,4.3,'24 V IN',14,AMBER,True)
box(57,31,43,8,'#fff9ef',AMBER,True)
mmtext(78.5,36.5,'zabezpieczenia 24 V',17,AMBER)
mmtext(21,65,'Miejsce na sygnały',17,MUTED)
mmtext(21,70,'/ złącza pomocnicze',17,MUTED)
box(-23,76,60.96,17.78,'#dde9ff',BLUE)
mmtext(7.48,82,'Teensy 4.1',22,BLUE,True)
mmtext(7.48,90,'60,96 × 17,78 mm',17,BLUE)
box(-26,81,3,7,'#c5cdd8',MUTED)
rect(65,OY+78*S,56,14*S,'#ffe4e7',RED,1,True)
arrow(130,OY+84.5*S,65,OY+84.5*S,RED,4)
text(65,OY+78*S-14,'USB ←',23,RED,True)
text(65,OY+92*S+29,'Strefa wtyku / przewodu',17,RED)
for name,x,y,w,h in drivers:
    box(x,y,w,h,'#fae3cd','#a55f26')
    mmtext(x+w/2,y+4.3,name,20,'#874410',True)
    box(x+3.12,y+7,9,9,'#c2c9cf','#6f7d88')
    for j in range(1,6):line(*pt(x+3.12+1.5*j,y+7.4),*pt(x+3.12+1.5*j,y+15.6),'#74818c',1)
    mmtext(x+w/2,y+19,'TMC2209',12,'#874410')
for x,y,r in caps:
    circle(*pt(x,y),r*S,'#fff8e9',AMBER,2)
    mmtext(x,y+1,'C_VM',12,AMBER)
for i,(_,x,y,w,h) in enumerate(ports):
    box(x,y,w,h,'#dcecff',BLUE,True)
    mmtext(x+w/2,y+3.8,f'M{i+1}',17,BLUE,True)
    for j in range(4):circle(*pt(x+3+3*j,y+7.3),2.2,'#ffffff',BLUE,1)
    arrow(*pt(x+w/2,120),*pt(x+w/2,131),BLUE,3)
text(OX+52.25*S,1169,'Wyjścia silników – w dół',25,BLUE,True,'middle')
text(OX+52.25*S,1202,'Rezerwa: 15 × 10 mm / złącze, typ do doboru',21,MUTED,anchor='middle')

LX=1140
text(LX,219,'ROZMIESZCZENIE',24,bold=True)
def note(y,title,lines,color=INK):
    text(LX,y,title,21,color,True)
    for i,s in enumerate(lines):text(LX,y+29+27*i,s,19,MUTED)
note(265,'Logika i komunikacja',['Hub8: 31 × 18,3','XIAO ESP32C3: 21 × 17,8','Teensy 4.1: 60,96 × 17,78'],GREEN)
note(400,'Zasilanie',['LM2596: ok. 43 × 21 (założenie)','24 V IN: rezerwa 16 × 6','Ochrona wejścia: rezerwa 43 × 8'],AMBER)
note(535,'Sterowniki: 2 rzędy × 3',['StepStick: 15,24 × 20,32 (założenie)','Radiator: symbol 9 × 9','C_VM: rezerwy Ø8, bez doboru BOM'],'#a55f26')
note(670,'Odstępy korpusów',['Kolumny sterowników: 5,08','Teensy → sterowniki: 6,04','Dolny rząd → złącza: 4,68','C_VM → korpus: min. 1,68'])
note(832,'Chłodzenie — idea przepływu',['Sześć radiatorów i nawiew wspólny.','Strzałki nie określają wentylatora','ani wymaganej wydajności.'])
for x in [1180,1260,1340,1420,1500,1580]:arrow(x,992,x,935,'#75aab7',3)
note(1040,'Do pomiaru i weryfikacji',['Występ PCB, wersje modułów,','złącza, wysokości, wtyki i radiator.','Brak tras i otworów montażowych.'])

line(65,1260,1635,1260,'#cbd5e0',2)
text(65,1301,'SKALA BRYŁ: 7 px / mm',20,bold=True)
text(65,1335,'Początek (0,0): lewy górny narożnik głównego prostokąta. X →  •  Y ↓',20,MUTED)
text(65,1372,'Obrysy przerywane = rezerwa lub wymiar wymagający potwierdzenia. Nawiew jest ilustracyjny.',20,MUTED)
text(65,1410,'Koncepcja mechaniczna • nie jest projektem PCB do produkcji • bez gwarancji prądu ciągłego',19,RED)
svg.append('</svg>')
(ROOT/'robot-pcb-layout.svg').write_text('\n'.join(svg),encoding='utf-8')
im.save(ROOT/'robot-pcb-layout.png')
print('PASS: six drivers, six ports, six diameter 8 cap envelopes; no body/cap overlaps; provisional tongue fit; main boundary containment; gaps 4.68 / 6.04 / 5.08 / 1.68 mm; connector edge 103 <=104.5 mm.')
