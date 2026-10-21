from pathlib import Path
from html import escape
import math
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parent
W,H=1900,1650
BG='#f5f7fb'; INK='#20354a'; MUTED='#586b7f'; KEEP='#e1f0e7'; EDGE='#428167'; WASTE='#f8dfe1'; RED='#b93f51'; BLUE='#2468b4'; GAP='#f3dfaa'
im=Image.new('RGB',(W,H),BG); d=ImageDraw.Draw(im)
sv=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',f'<rect width="{W}" height="{H}" fill="{BG}"/>']
def text(x,y,t,size=22,color=INK,bold=False,anchor='start'):
    fn='segoeuib.ttf' if bold else 'segoeui.ttf'
    f=ImageFont.truetype('C:/Windows/Fonts/'+fn,size)
    d.text((x,y),t,font=f,fill=color,anchor={'start':'ls','middle':'ms','end':'rs'}[anchor])
    sv.append(f'<text x="{x}" y="{y}" fill="{color}" font-family="Segoe UI,Arial,sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" text-anchor="{anchor}">{escape(t)}</text>')
def line(x1,y1,x2,y2,color=INK,width=2,dash=False):
    sv.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{width}"'+(' stroke-dasharray="7 5"' if dash else '')+'/>')
    if dash:
        n=max(1,int(math.hypot(x2-x1,y2-y1)/7))
        for i in range(0,n,2):
            a,b=i/n,min(i+1,n)/n
            d.line((x1+(x2-x1)*a,y1+(y2-y1)*a,x1+(x2-x1)*b,y1+(y2-y1)*b),fill=color,width=width)
    else:d.line((x1,y1,x2,y2),fill=color,width=width)
def rect(x,y,w,h,fill,stroke=INK,width=2,dash=False):
    sv.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"'+(' stroke-dasharray="7 5"' if dash else '')+'/>')
    d.rectangle((x,y,x+w,y+h),fill=fill,outline=None if dash else stroke,width=width)
    if dash:
        for a,b,c,e in [(x,y,x+w,y),(x+w,y,x+w,y+h),(x+w,y+h,x,y+h),(x,y+h,x,y)]:line(a,b,c,e,stroke,width,True)
def polygon(points,fill,stroke=INK,width=2):
    sv.append('<polygon points="'+' '.join(f'{x},{y}' for x,y in points)+f'" fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>')
    d.polygon(points,fill=fill);d.line(points+[points[0]],fill=stroke,width=width)
def circle(x,y,r,fill,stroke=INK):
    sv.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="{stroke}"/>')
    d.ellipse((x-r,y-r,x+r,y+r),fill=fill,outline=stroke)
def arrow(x1,y1,x2,y2,color=INK):
    line(x1,y1,x2,y2,color,3);a=math.atan2(y2-y1,x2-x1)
    for b in (a+2.65,a-2.65):line(x2,y2,x2+12*math.cos(b),y2+12*math.sin(b),color,3)
def dim(x1,y1,x2,y2,t,tx,ty):
    line(x1,y1,x2,y2,MUTED)
    for x,y in ((x1,y1),(x2,y2)):
        line(x-6 if x1==x2 else x,y-6 if y1==y2 else y,x+6 if x1==x2 else x,y+6 if y1==y2 else y,MUTED)
    text(tx,ty,t,20,MUTED,anchor='middle')
S,OX,OY=7,300,230
def pt(x,y):return OX+x*S,OY+y*S
def box(x,y,w,h,fill,stroke=INK,dash=False):rect(*pt(x,y),w*S,h*S,fill,stroke,2,dash)
def mt(x,y,t,size=18,color=INK,bold=False):text(*pt(x,y),t,size,color,bold,'middle')

# Parts share global concept coordinates, not an assumed physical hole origin.
def left_contains(x,y):
    if not(-23.5<=x<=40 and .5<=y<=120.5):return False
    if y<71.62 and x<1.9:return False
    if x<-16.5 and y>113.5:return (x+16.5)**2+(y-113.5)**2<=49+1e-8
    return True
def right_contains(x,y):return 42<=x<=102.96 and .5<=y<=120.5
def fit(body,predicate):
    _,x,y,w,h=body
    return all(predicate(xx,yy) for xx,yy in ((x,y),(x+w,y),(x,y+h),(x+w,y+h)))
logic=[('Teensy',-23,76,60.96,17.78),('Hub8',7,7,31,18.3),('XIAO',7,38,21,17.8)]
power=[('LM2596',57,7,43,21)]
drivers=[(f'M{i+1}',x,y,15.24,20.32) for i,(x,y) in enumerate(( (x,y) for y in (52,84) for x in (44,64.32,84.64)))]
ports=[(f'M{i+1}',x,109,15,10) for i,x in enumerate((-17,0,17,45,64,83))]
assert all(fit(b,left_contains) for b in logic+ports[:3])
assert all(fit(b,right_contains) for b in power+drivers+ports[3:])
assert len(drivers)==len(ports)==6 and len(ports[:3])==len(ports[3:])==3
assert math.isclose(42-40,2) and math.isclose(-23-(-23.5),.5)
assert 63.5<=80 and 60.96<=80 and 120<=120
assert math.isclose(80-63.5,16.5) and math.isclose(80-60.96,19.04)
assert math.isclose(25.4/2.54,10) and math.isclose(71.12/2.54,28)
all_b=logic+power+drivers+ports
for i,(name,x,y,w,h) in enumerate(all_b):
    for other,xx,yy,ww,hh in all_b[i+1:]:
        assert x+w<=xx or xx+ww<=x or y+h<=yy or yy+hh<=y,(name,other)
assert 7>=1.9 and 38+17.8<71.62

text(65,65,'Plan podziału płytek 120 × 80 mm',39,bold=True)
text(65,106,'Dwie płytki • bok 120 mm pionowo • wymiary orientacyjne',25,MUTED)
text(65,145,'To nie jest szablon do bezpośredniego cięcia: najpierw karton, pomiar i przymiarka do otworów.',23,RED)
text(65,197,'ZŁOŻENIE • moduły bez zmiany położenia',24,bold=True)

# Background available envelope, including the assumed tongue.
env=[pt(0,0),pt(104.5,0),pt(104.5,121),pt(-17,121)]
for i in range(1,13):
    t=i/12;env.append(pt(-17*(1-t)**2-48*(1-t)*t-24*t*t,121*(1-t)**2+242*(1-t)*t+114*t*t))
env +=[pt(-24,71.2),pt(-12,71.2),pt(0,59.2)]
polygon(env,'#edf0f5','#a8b5c1',1)
L=[pt(1.9,.5),pt(40,.5),pt(40,120.5),pt(-16.5,120.5)]
for i in range(1,19):
    a=math.pi/2+i/18*math.pi/2;L.append(pt(-16.5+7*math.cos(a),113.5+7*math.sin(a)))
L +=[pt(-23.5,71.62),pt(1.9,71.62)]
polygon(L,KEEP,EDGE)
box(42,.5,60.96,120,KEEP,EDGE)
box(40,.5,2,120,GAP,'#bc9a43',True)
text(1055,OY+61*S,'120',19,MUTED)
line(*pt(105,.5),*pt(105,120.5),MUTED)

box(7,7,31,18.3,'#d0eae5','#247c75');mt(22.5,14.5,'KAmod',19,'#247c75',True);mt(22.5,20,'I2C Hub8',19,'#247c75',True)
box(7,38,21,17.8,'#d0eae5','#247c75');mt(17.5,44,'XIAO',19,'#247c75',True);mt(17.5,50,'ESP32C3',17,'#247c75',True)
box(-23,76,60.96,17.78,'#dce8ff',BLUE);mt(7.48,82,'Teensy 4.1',23,BLUE,True);mt(7.48,89,'USB w lewo',19,BLUE)
box(-26,81,3,7,'#c5cdd8',MUTED)
arrow(*pt(-27,84.5),*pt(-33,84.5),RED);text(65,OY+79*S,'USB ←',24,RED,True)
text(65,OY+96*S,'0,5 mm od krawędzi:',18,RED);text(65,OY+100*S,'sprawdź wtyk / przewód',18,RED)
box(57,7,43,21,'#fff0d5','#a87722');mt(78.5,15,'LM2596',22,'#a87722',True);mt(78.5,22,'24 V → 5 V',23,'#a87722',True)
for name,x,y,w,h in drivers:
    box(x,y,w,h,'#fae2cc','#a6642d');mt(x+w/2,y+5,name,21,'#8e4918',True)
    box(x+3.12,y+7,9,9,'#c3ccd2','#77828c')
    for j in range(1,6):line(*pt(x+3.12+1.5*j,y+7.3),*pt(x+3.12+1.5*j,y+15.7),'#7c8993',1)
    mt(x+w/2,y+19,'TMC2209',12,'#8e4918')
for name,x,y,w,h in ports:
    box(x,y,w,h,'#dcecff',BLUE,True);mt(x+w/2,y+4,name,18,BLUE,True)
    for j in range(4):circle(*pt(x+3+3*j,y+7.5),2.5,'#ffffff',BLUE)
    arrow(*pt(x+w/2,120),*pt(x+w/2,130),BLUE)
mt(8,105,'A • logika + 3 wyjścia',17,EDGE,True)
text(630,1170,'Wyjścia silników w dół • złącza przełożone względem pierwszej koncepcji',21,BLUE,True,'middle')
text(630,1203,'Szczelina 2 mm; brak tras, otworów montażowych i połączeń mechanicznych na rysunku.',20,MUTED,anchor='middle')
text(*pt(47,35),'Szczelina 2 mm',18,'#a27b27')
arrow(*pt(46,36),*pt(41,36),'#a27b27')

text(1185,205,'DWA SUROWE FORMATY 80 × 120',24,bold=True)
text(1185,239,'Zachować zielone • usunąć różowe',21,MUTED)
def stock(x,y,keep,is_left):
    sc=2.65; p=lambda a,b:(x+a*sc,y+b*sc)
    rect(x,y,80*sc,120*sc,WASTE,RED)
    if is_left:
        points=[p(25.4,0),p(63.5,0),p(63.5,120),p(7,120)]
        for i in range(1,17):
            a=math.pi/2+i/16*math.pi/2;points.append(p(7+7*math.cos(a),113+7*math.sin(a)))
        points +=[p(0,71.12),p(25.4,71.12)]
        polygon(points,KEEP,EDGE)
        text(*p(12.7,30),'odpad',18,RED,anchor='middle');text(*p(12.7,40),'wcięcia',18,RED,anchor='middle')
        text(*p(34,91),'A',30,EDGE,True,'middle')
        text(x-6,y+120*sc+34,'~63,5 × 120',22,EDGE,True)
        text(x-6,y+120*sc+65,'Wcięcie: 25,4 × 71,12',18,MUTED)
        text(x-6,y+120*sc+92,'Narożnik: R7',18,MUTED)
        label='16,5'
    else:
        rect(x,y,keep*sc,120*sc,KEEP,EDGE)
        text(*p(keep/2,64),'B',30,EDGE,True,'middle')
        text(x-6,y+120*sc+34,'~61 × 120',22,EDGE,True)
        text(x-6,y+120*sc+65,'Nominalnie: 60,96 × 120',18,MUTED)
        label='19,04'
    text(*p((keep+80)/2,87),'odpad',16,RED,anchor='middle')
    text(*p((keep+80)/2,97),label,16,RED,anchor='middle')
    dim(x,y-17,x+80*sc,y-17,'80',x+40*sc,y-27)
    dim(x+80*sc+28,y,x+80*sc+28,y+120*sc,'120',x+80*sc+58,y+62*sc)
stock(1200,310,63.5,True);stock(1550,310,60.96,False)
text(1200,270,'PŁYTKA A • lewa, L',21,bold=True);text(1550,270,'PŁYTKA B • prawa',21,bold=True)

def note(x,y,title,lines,color=INK):
    text(x,y,title,22,color,True)
    for i,t in enumerate(lines):text(x,y+31+28*i,t,20,MUTED)
note(1185,782,'Raster: odniesienie nominalne',['25,4 = 10 × 2,54; 71,12 = 28 × 2,54.','Początek otworów i rzeczywisty format nieznane.','Moduły dopasować do rzeczywistych otworów.','Wymiary nie obejmują rzazu.'])
note(1185,960,'B: moc + 3 wyjścia; A: logika + 3 wyjścia',['Do trzech lewych wyjść: 12 przewodów fazowych.','Prowadzenie nad / pod płytkami, w izolacji.','Sprawdź wiązkę na sucho; kanał niezweryfikowany.'],BLUE)
note(65,1290,'1  Najpierw przymiarka',['Wytnij model kartonowy; sprawdź obrys obudowy,','złącza, kabel USB, raster i sposób podparcia.','Sprawdź: pojedyncze pady czy połączone paski.'])
note(690,1290,'2  Cięcie od strony odpadu',['Zacznij od długiego pasa, potem wytnij wcięcie L.','Nie wyłamuj narożnika L. Piłka o drobnych zębach,','zamocowana płytka, następnie dopasowanie pilnikiem.'])
note(1320,1290,'3  Montaż i zasilanie',['Każda część na własnych dystansach.','Przewody / lut nie łączą mechanicznie płytek.','Moc: osobne przewody, nie łańcuchy cienkich padów.'])
line(65,1440,1835,1440,'#cad5df')
text(65,1480,'MINIMUM: 2 PŁYTKI',23,bold=True)
text(65,1514,'Jedna płytka 120 × 80 nie pokryje pola 104,5 × 121 mm z występem. Geometria jest orientacyjna.',21,MUTED)
text(65,1550,'Jeśli laminat FR4: pył wymaga ochrony oczu, maski i wentylacji. Nacinanie / łamanie tylko jako wariant cięcia prostego.',20,RED)
text(65,1590,'Bryły modułów w skali 7 px/mm • mapy surowych płytek 2,65 px/mm • bez gwarancji elektrycznej i termicznej',20,MUTED)
sv.append('</svg>')
(ROOT/'perfboard-cutting-plan.svg').write_text('\n'.join(sv),encoding='utf-8')
im.save(ROOT/'perfboard-cutting-plan.png')
print('PASS: modules and six ports each fit one part; 3 left / 3 right ports; six drivers; no body overlaps; notch clears logic; rounded corner fits; blank bounds; 2 mm seam; 0.5 mm Teensy edge margin. Exit normal.')
