from pathlib import Path
from html import escape
import math
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parent
W,H=2300,1930
BG='#f5f7fb'; INK='#20364b'; MUTED='#586d82'; BLUE='#236ac0'; PURPLE='#8545ac'; RED='#bd4050'; GREEN='#387f68'; AMBER='#a77622'
im=Image.new('RGB',(W,H),BG);d=ImageDraw.Draw(im)
sv=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',f'<rect width="{W}" height="{H}" fill="{BG}"/>']
def text(x,y,t,size=22,color=INK,bold=False,anchor='start'):
    f=ImageFont.truetype('C:/Windows/Fonts/'+('segoeuib.ttf' if bold else 'segoeui.ttf'),size)
    d.text((x,y),t,font=f,fill=color,anchor={'start':'ls','middle':'ms','end':'rs'}[anchor])
    sv.append(f'<text x="{x}" y="{y}" fill="{color}" font-family="Segoe UI,Arial,sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" text-anchor="{anchor}">{escape(t)}</text>')
def line(x1,y1,x2,y2,color=INK,width=2,dash=False):
    sv.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{width}"'+(' stroke-dasharray="6 4"' if dash else '')+'/>')
    if dash:
        n=max(1,int(math.hypot(x2-x1,y2-y1)/6))
        for i in range(0,n,2):
            a,b=i/n,min(i+1,n)/n;d.line((x1+(x2-x1)*a,y1+(y2-y1)*a,x1+(x2-x1)*b,y1+(y2-y1)*b),fill=color,width=width)
    else:d.line((x1,y1,x2,y2),fill=color,width=width)
def rect(x,y,w,h,fill,stroke=INK,width=2):
    sv.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>')
    d.rectangle((x,y,x+w,y+h),fill=fill,outline=stroke,width=width)
def circle(x,y,r,fill,stroke=INK,width=2):
    sv.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>')
    d.ellipse((x-r,y-r,x+r,y+r),fill=fill,outline=stroke,width=width)
def polygon(points,fill,stroke=INK):
    sv.append('<polygon points="'+' '.join(f'{x},{y}' for x,y in points)+f'" fill="{fill}" stroke="{stroke}" stroke-width="2"/>')
    d.polygon(points,fill=fill);d.line(points+[points[0]],fill=stroke,width=2)
def arrow(x1,y1,x2,y2,color=INK,width=3):
    line(x1,y1,x2,y2,color,width);a=math.atan2(y2-y1,x2-x1)
    for b in (a+2.65,a-2.65):line(x2,y2,x2+12*math.cos(b),y2+12*math.sin(b),color,width)
S,OX,OY=8,350,230
def pt(x,y):return OX+x*S,OY+y*S
def box(x,y,w,h,fill,stroke=INK):rect(*pt(x,y),w*S,h*S,fill,stroke)
def mt(x,y,t,size=20,color=INK,bold=False):text(*pt(x,y),t,size,color,bold,'middle')
def note(x,y,title,lines,color=INK,size=22):
    text(x,y,title,25,color,True)
    for i,t in enumerate(lines):text(x,y+34+31*i,t,size,MUTED)

def inleft(x,y):
    if not(-23.5<=x<=40 and .5<=y<=120.5):return False
    if x<1.9 and y<71.62:return False
    if x<-16.5 and y>113.5:return (x+16.5)**2+(y-113.5)**2<=49+1e-7
    return True
def inright(x,y):return 42<=x<=102.96 and .5<=y<=120.5
def fit(x,y,w,h,part):return all(part(a,b) for a,b in [(x,y),(x+w,y),(x,y+h),(x+w,y+h)])
drivers=[dict(name=f'M{i+1}',x=x,y=92.5,w=15.24,h=20.32,rot=i in (1,2),left=i<3) for i,x in enumerate((-22.23,.63,23.49,44.54,64.86,85.18))]
ports=[dict(name=b['name'],x=b['x']+2.31+(2.54 if i==0 else 0),y=113.53,w=10.62,h=6.2,left=i<3) for i,b in enumerate(drivers)]
caps=[(-3.18,97.58),(-3.18,107.74),(19.68,107.74),(55.97,87.42),(76.29,87.42),(96.61,87.42)]
logic=[('Teensy',-22.23,72.18,60.96,17.78,True),('Hub8',7,7,31,18.3,True),('XIAO',7,38,21,17.8,True),('LM2596',57,7,43,21,False)]
rects=[(b['name'],b['x'],b['y'],b['w'],b['h'],b['left']) for b in drivers]+[(b['name']+'port',b['x'],b['y'],b['w'],b['h'],b['left']) for b in ports]+logic
for name,x,y,w,h,left in rects:assert fit(x,y,w,h,inleft if left else inright),name
for i,(name,x,y,w,h,left) in enumerate(rects):
    for other,xx,yy,ww,hh,_ in rects[i+1:]:assert x+w<=xx or xx+ww<=x or y+h<=yy or yy+hh<=y,(name,other)
for i,(x,y) in enumerate(caps):
    part=inleft if i<3 else inright
    assert all(part(x+3.15*math.cos(t*math.pi/90),y+3.15*math.sin(t*math.pi/90)) for t in range(180))
    for name,xx,yy,w,h,left in rects:assert math.hypot(x-max(xx,min(x,xx+w)),y-max(yy,min(y,yy+h)))>=3.15-1e-7,(i,name)
    for xx,yy in caps[i+1:]:assert math.hypot(x-xx,y-yy)>=6.3
phases=['A2','A1','B1','B2'];nets=[]
for b,p in zip(drivers,ports):
    sx=b['x']+(1.27 if b['rot'] else 13.97)
    row=['GND','VIO','B2','B1','A1','A2','GND','VM'] if b['rot'] else ['VM','GND','A2','A1','B1','B2','VIO','GND']
    for j,phase in enumerate(phases):
        sy=93.77+2.54*row.index(phase);ex=p['x']+1.5+2.54*j;ey=116.63
        assert (sx<=40 and ex<=40) if b['left'] else (sx>=42 and ex>=42)
        nets.append((b['name'],phase,sx,sy,ex,ey,math.hypot(sx-ex,sy-ey)))
    base=-20.96 if b['left'] else 45.81
    for xx in [b['x']+1.27,b['x']+13.97]+[p['x']+1.5+2.54*j for j in range(4)]:assert math.isclose((xx-base)/2.54,round((xx-base)/2.54),abs_tol=1e-7)
    assert math.isclose((116.63-93.77)/2.54,9)
assert len(nets)==24 and len(set((n[0],n[1]) for n in nets))==24
assert len(drivers)==len(ports)==len(caps)==6
assert math.isclose(40-38.73,1.27) and math.isclose(72.18-71.62,.56)
assert math.isclose(92.5-(72.18+17.78),2.54)
assert math.isclose(120.5-(113.53+6.2),.77) and math.isclose(113.53-(92.5+20.32),.71)
assert 63.5<=80 and 60.96<=80 and 120<=120

text(65,66,'Układ pod krótkie połączenia silnikowe',42,bold=True)
text(65,109,'Sześć sterowników przy krawędzi • 24 osobne przewody fazowe • widok z góry z projekcją połączeń od spodu',25,MUTED)
text(65,153,'Potwierdzony BIGTREETECH TMC2209 V1.3 • widok TOP — sprawdź VM, GND i fazy przed włożeniem / USB.',23,RED)
text(65,199,'Cięcie bez zmian: A 63,5 × 120 mm, L / B ~61 × 120 mm • szczelina 2 mm',24,GREEN,True)

L=[pt(1.9,.5),pt(40,.5),pt(40,120.5),pt(-16.5,120.5)]
for i in range(1,21):
    a=math.pi/2+i/20*math.pi/2;L.append(pt(-16.5+7*math.cos(a),113.5+7*math.sin(a)))
L +=[pt(-23.5,71.62),pt(1.9,71.62)]
polygon(L,'#e0eee5',GREEN);box(42,.5,60.96,120,'#e0eee5',GREEN);box(40,.5,2,120,'#f5dfab','#b79a57')
mt(21,34,'A • logika + M1–M3',21,GREEN,True);mt(74,35,'B • zasilanie + M4–M6',21,GREEN,True)
box(7,7,31,18.3,'#d3eae5','#267c73');mt(22.5,14.5,'KAmod',22,'#267c73',True);mt(22.5,20,'I2C Hub8',22,'#267c73',True)
box(7,38,21,17.8,'#d3eae5','#267c73');mt(17.5,44,'XIAO',22,'#267c73',True);mt(17.5,50,'ESP32C3',20,'#267c73',True)
box(-22.23,72.18,60.96,17.78,'#dce8ff',BLUE);mt(8.25,79,'Teensy 4.1',26,BLUE,True);mt(8.25,85.5,'przesunięte • marginesy 1,27 mm',18,BLUE)
box(-25.23,77.5,3,7,'#c6ced7',MUTED);arrow(*pt(-25.8,81),*pt(-33,81),RED);text(65,OY+75*S,'USB ←',26,RED,True)
box(57,7,43,21,'#fff0d5',AMBER);mt(78.5,15,'LM2596',26,AMBER,True);mt(78.5,22,'24 V → 5 V',26,AMBER,True)
mt(74,45,'Wolne miejsce na rozdział',21,MUTED);mt(74,50,'zasilania i połączenia sterujące',21,MUTED)
mt(74,61,'Przez szczelinę: sterowanie + zasilanie',18,AMBER,True);mt(74,66,'bez przewodów fazowych silników',18,AMBER)

# Pale bodies are projected: dashed insulated phase wires lie below the boards.
for b in drivers:
    x,y=b['x'],b['y'];box(x,y,b['w'],b['h'],'#f4eadf','#b58a61')
    mt(x+7.62,96,b['name']+(' • 180°' if b['rot'] else ' • R'),18,'#8b501d',True)
    for side in [1.27,13.97]:
        for j in range(8):circle(*pt(x+side,93.77+2.54*j),3.3,'#e4e5e7','#9a9da3',1)
    vmx=x+(1.27 if b['rot'] else 13.97);vmy=111.55 if b['rot'] else 93.77
    circle(*pt(vmx,vmy),5,RED,RED);mt(x+7.62,111.6,'VM ↓' if b['rot'] else '',13,RED,True)
for p in ports:
    box(p['x'],p['y'],p['w'],p['h'],'#dce9f9',BLUE)
    mt(p['x']+p['w']/2,121.9,p['name'],20,BLUE,True)
    arrow(*pt(p['x']+p['w']/2,123),*pt(p['x']+p['w']/2,128),BLUE)
for name,phase,sx,sy,ex,ey,length in nets:
    color=BLUE if phase.startswith('A') else PURPLE
    line(*pt(sx,sy),*pt(ex,ey),color,2,True)
    circle(*pt(sx,sy),3.5,color,color)
    circle(*pt(ex,ey),4.5,'#ffffff',color)
for i,(x,y) in enumerate(caps):
    circle(*pt(x,y),3.15*S,'#fff7dc',AMBER)
    # Leads form from 2.50 to the nominal 2.54 mm concept pitch.
    plus_y=y+(1.27 if i in (1,2) else -1.27)
    minus_y=y-(1.27 if i in (1,2) else -1.27)
    circle(*pt(x,plus_y),2.5,RED,RED);circle(*pt(x,minus_y),2.5,'#ffffff',MUTED)
    mt(x,y+.5,f'C{i+1}',13,AMBER,True)
    text(*pt(x+1.1,plus_y+.4),'+',13,RED,True)
mt(8,68,'Wszystkie TMC: radiatory i nawiew',19,MUTED)
text(660,1300,'24 przerywane linie = osobne izolowane przewody POD płytkami, nie ścieżki PCB',23,BLUE,True,'middle')
text(660,1336,'Niebieski: cewka A • fioletowy: cewka B • skrzyżowania przewodów bez połączenia',22,MUTED,anchor='middle')
text(660,1371,'około 1–3 cm przewodu po ułożeniu; długości do przymiarki',24,INK,True,'middle')

RX=1305
text(RX,220,'POŁĄCZENIA FUNKCJONALNE',28,bold=True)
def block(x,y,w,h,lines,color=GREEN,fill='#e0eee5'):
    rect(x,y,w,h,fill,color)
    for i,t in enumerate(lines):text(x+w/2,y+29+28*i,t,21,color,i==0,'middle')
block(RX,250,400,58,['24 V IN • zabezpieczenia'],AMBER,'#fff0d5')
arrow(RX+200,309,RX+200,345,AMBER)
block(RX,345,400,90,['Osobne pary VM / GND','bank lewy: M1–M3','bank prawy: M4–M6'],AMBER,'#fff0d5')
arrow(RX+400,277,RX+480,277,AMBER)
block(RX+480,250,390,87,['LM2596 → 5 V','Teensy VIN / XIAO 5 V'],BLUE,'#dce8ff')
note(RX+480,384,'USB + zewnętrzne 5 V',['Rozdzielenie VIN / VUSB Teensy.','XIAO: ochrona przed zasilaniem','zwrotnym zgodnie z Seeed.'],RED,21)
note(RX,484,'C1–C6 blisko właściwego VM / GND',['100 µF / 35 V, low ESR: przykład Rubycon ZLH.','Korpus Ø6,3 × 11 mm; raster nóżek 2,5 mm.','Na rysunku raster 2,54 wymaga uformowania nóżek.','+ kondensatora do VM, − do lokalnego GND.'],AMBER)
line(RX,644,2200,644,'#ccd6df')
note(RX,693,'Teensy → 6 sterowników',['6 niezależnych par STEP + DIR.','EN wspólne; stan nieaktywny przy resecie.','Poziomy sygnałów i VIO: 3,3 V.'],BLUE)
note(RX+480,693,'I2C / komunikacja',['Hub: pull-up do 3,3 V.','XIAO: niezależna komunikacja.','Nie łączyć wyjść regulatorów 3,3 V.'],GREEN,21)
note(RX,858,'UART V1.3 — jeśli potrzebny',['RX → PDN_UART; TX przez R10 (NC), pozostaw wolne.','Dwie osobne grupy po 3; adresy 0 / 1 / 2 w każdej.','Maks. 4 adresy; sześć na jednej magistrali odpada.','Sygnały 3,3 V i wspólna masa; szczegóły w notatkach.'])
line(RX,999,2200,999,'#ccd6df')
note(RX,1042,'Złącze: Phoenix MPT 0,5/4-2,54 • 1725672',['Korpus 10,62 × 6,2; raster 2,54; otwór 1,1 mm.','Wysokość montażowa 8,5 mm; nominalnie 6 A (dane złącza).','6 A nie określa prądu układu ani przekroju wspólnego zasilania.','Oś pinów Y=116,63 i odsunięcie 1,5 mm: do weryfikacji.'],BLUE)
note(RX,1230,'Orientacja i fazy — BIGTREETECH V1.3',['R: M1 / M4 / M5 / M6, VM na górze prawej listwy.','180° w płaszczyźnie płytki: M2 / M3, VM lewy dół.','Każde złącze od lewej: A2 | A1 | B1 | B2.','Pary cewek A2–A1 / B1–B2; kolory silnika nie są uniwersalne.'],RED)

line(65,1440,2235,1440,'#c9d5df')
note(65,1490,'Mechanika do sprawdzenia',['Teensy: 1,27 mm z boków; 0,56 nad dolnym występem.','Teensy → rząd TMC: 2,54; TMC → złącze: 0,71.','Złącza → dół: 0,77 mm; M1 przesunięte dla R7.','Rzeczywisty początek rastra, otwory 1,1 mm i pierścienie padów.'],size=21)
note(805,1490,'Przewody i podparcie',['Izolacja, przestrzeń pod płytką i dystanse do przymiarki.','Bez prowadzenia faz przez szczelinę między płytkami.','Duże prądy: dedykowane przewody i rozdział mocy,','nie łańcuch zwykłych padów / złączy sygnałowych.'],size=21)
note(1545,1490,'Termika i uruchomienie',['Instrukcja V1.3: 2 A peak; ciągły prąd do weryfikacji.','Powyżej 1,2 A: aktywne chłodzenie według instrukcji.','Nawiew na cały rząd; sprawdź polaryzację i połączenia','przed włożeniem sterowników i podłączeniem USB.'],RED,21)
text(65,1720,'REFERENCJA LISTWY SILNIKOWEJ — kolejność od góry modułu na rysunku',24,bold=True)
text(65,1760,'R: VM · GND · A2 · A1 · B1 · B2 · VIO · GND',23,INK)
text(65,1800,'180°: GND · VIO · B2 · B1 · A1 · A2 · GND · VM',23,INK)
text(65,1850,'Nie jest to uniwersalny pinout wszystkich TMC2209. Raster 2 × 8 / 2,54 mm i odstęp rzędów 12,7 mm wymagają sprawdzenia.',23,RED)
text(65,1890,'Koncepcja montażu przewodowego • bez pełnego schematu, walidacji termicznej, padów i gotowej płytki produkcyjnej',21,MUTED)
sv.append('</svg>')
(ROOT/'motor-wiring-layout.svg').write_text('\n'.join(sv),encoding='utf-8');im.save(ROOT/'motor-wiring-layout.png')

md='''# Układ pod krótkie połączenia silnikowe

Sześć potwierdzonych modułów BIGTREETECH TMC2209 V1.3 umieszczono w jednym dolnym rzędzie. Każdy ma własne czteropolowe złącze obok dolnej krawędzi; wszystkie 24 przewody fazowe pozostają na tej samej części płytki co sterownik. Ta aktualizacja dotyczy najnowszego planu połączeń silnikowych; wcześniejsze koncepcje są historyczne i pozostają bez zmian, podobnie jak podział A/B. To koncepcja montażu przewodowego, nie pełny schemat ani projekt PCB do produkcji.

## Geometria i orientacje

Wymiary i współrzędne w mm, początek jak w poprzednich koncepcjach. A: X −23,5…40; Y 0,5…120,5; wcięcie górne lewe 25,4 × 71,12; dolny lewy narożnik R7, środek (−16,5; 113,5). B: X 42…102,96; Y 0,5…120,5. Szczelina 2 mm. Obie części mieszczą się w osobnych formatach 80 × 120 mm.

Teensy: (−22,23; 72,18), 60,96 × 17,78; USB w lewo. Hub8: (7; 7), 31 × 18,3; XIAO: (7; 38), 21 × 17,8; LM2596: (57; 7), orientacyjnie 43 × 21. Antena XIAO pozostaje zewnętrzna.

Wszystkie sterowniki mają korpus 15,24 × 20,32, potwierdzony rysunkiem wymiarowym V1.3, oraz Y = 92,5. Wszystkie złącza mają Y = 113,53, korpus 10,62 × 6,2 i oś pinów Y = 116,63.

| Moduł | X sterownika | Orientacja | X złącza | Część |
|---|---:|---|---:|---|
| M1 | −22,23 | R, VM prawa góra | −17,38 | A |
| M2 | 0,63 | 180°, VM lewy dół | 2,94 | A |
| M3 | 23,49 | 180°, VM lewy dół | 25,80 | A |
| M4 | 44,54 | R, VM prawa góra | 46,85 | B |
| M5 | 64,86 | R, VM prawa góra | 67,17 | B |
| M6 | 85,18 | R, VM prawa góra | 87,49 | B |

Rozstaw modułów: lewy bank 22,86 mm (przerwa 7,62), prawy 20,32 mm (przerwa 5,08). Złącze M1 przesunięto o dodatkowe 2,54 mm w prawo, aby ominąć R7. Pozostałe złącza: X sterownika + 2,31.

Marginesy Teensy: po 1,27 mm z lewej i prawej, 0,56 mm od poziomu wcięcia; od Teensy do TMC 2,54 mm. Od TMC do korpusu złącza 0,71 mm, od złącza do dolnej krawędzi 0,77 mm. Są to marginesy samych brył; pola lutownicze, tolerancje i dostęp narzędzi pozostają do sprawdzenia.

## Pinout i 24 połączenia fazowe

**Potwierdzona wersja: BIGTREETECH TMC2209 V1.3.** Pinout zweryfikowano z oficjalnym zdjęciem opisanym TOP, przedstawiającym stronę ze złotym polem termicznym. Nie jest to uniwersalny pinout wszystkich modułów TMC2209. Przed włożeniem oraz podłączeniem USB sprawdzić VM, GND i fazy na faktycznych płytkach. Obrót 180° oznacza obrót w płaszczyźnie płytki, bez odwracania modułu spodem do góry.

Założenie geometryczne: 2 listwy po 8 pinów, podziałka 2,54 mm, odległość rzędów 12,7 mm, odsunięcie pinów 1,27 mm od krawędzi korpusu. Sam korpus 15,24 × 20,32 jest potwierdzony; podziałkę, odstęp rzędów i odsunięcia nadal trzeba zmierzyć. Wymiarów centralnego obszaru 8,40 × 12,88 na rysunku nie należy utożsamiać z rozstawem listew. Dla R lista silnikowa po prawej od góry: **VM, GND, A2, A1, B1, B2, VIO, GND**. Dla obrotu 180° lista po lewej od góry: **GND, VIO, B2, B1, A1, A2, GND, VM**.

Fizyczna listwa sterująca w widoku TOP/R po lewej od góry: **EN, MS1, MS2, RX, TX, CLK, STEP, DIR**. Po obrocie 180° znajduje się po prawej, od góry: **DIR, STEP, CLK, TX, RX, MS2, MS1, EN**. To kolejność fizyczna, nie numeracja P1A na schemacie. Rysunek nie przypisuje rzeczywistych numerów pinów Teensy.

Złącze każdego silnika od lewej: **A2, A1, B1, B2**. Cewka A to dwa pierwsze pola, B to dwa kolejne. Sprawdzić pary uzwojeń rzeczywistego silnika; jego kolory przewodów nie są uniwersalne. Przerywane linie to osobne izolowane przewody pod płytką, nie ścieżki PCB; przecięcia nie są węzłami. Zadbać o izolację, wysokość pod płytką, podparcie i odciążenie przewodów. Kolory na diagramie oznaczają cewki, nie wymagane kolory izolacji.

Poniżej współrzędne źródła i końca w mm oraz geometryczna odległość w linii prostej. To **nie są długości cięcia**: około 1–3 cm przewodu po ułożeniu; długości do przymiarki, z uwzględnieniem lutowania, gięcia i zapasu.

| Moduł | Sieć | Pin modułu X; Y | Pin złącza X; Y | Linia prosta mm |
|---|---|---|---|---:|
'''
for name,phase,sx,sy,ex,ey,length in nets:
    md+=f'| {name} | {phase} | {sx:.2f}; {sy:.2f} | {ex:.2f}; {ey:.2f} | {length:.2f} |\n'
md+='''
## Konkretne złącze i kondensatory

[Phoenix Contact MPT 0,5/4-2,54, 1725672](https://www.phoenixcontact.com/en-us/products/printed-circuit-board-terminal-mpt-05-4-254-1725672?type=pdf): korpus 10,62 × 6,2 mm, raster liniowy 4 pinów 2,54 mm, nominalny prąd 6 A, wysokość montażowa 8,5 mm, otwór montażowy 1,1 mm. Na koncepcji piny zaczynają się 1,5 mm od lewej krawędzi i leżą 3,1 mm od krawędzi korpusu w Y; **te odsunięcia trzeba potwierdzić rysunkiem konkretnego egzemplarza przed montażem**. Sprawdzić, czy otwory płytki można bezpiecznie dostosować do 1,1 mm bez zniszczenia pierścieni padów. Nominalne 6 A złącza nie wyznacza przekroju przewodów wspólnego zasilania.

Przykład C1–C6: Rubycon ZLH 100 µF / 35 V low ESR, korpus Ø6,3 × 11 mm, raster wyprowadzeń 2,5 mm. Źródło: [katalog Rubycon ZLH](https://www.rubycon.co.jp/wp-content/uploads/catalog-aluminum/ZLH.pdf). Sprawdzić konkretny numer zamówieniowy i warunki pracy; to przykład lokalnej filtracji, nie kompletny dobór całej sieci zasilania. Uwaga na wysokość 11 mm. Piny w koncepcji po Y ±1,27 mm: wymaga to ostrożnego uformowania nominalnego rastra 2,5 mm do 2,54, bez naprężania uszczelnienia. **Plus do VM, minus do lokalnego GND.**

| C | Środek X; Y | Przypisanie VM |
|---|---|---|
| C1 | −3,18; 97,58 | M1, VM u góry |
| C2 | −3,18; 107,74 | M2, VM u dołu |
| C3 | 19,68; 107,74 | M3, VM u dołu |
| C4 | 55,97; 87,42 | M4, VM u góry |
| C5 | 76,29; 87,42 | M5, VM u góry |
| C6 | 96,61; 87,42 | M6, VM u góry |

Kondensatory są nad płytką, przewody fazowe pod spodem. C1 i C4–C6: plus u góry (Y środka − 1,27); C2/C3: plus u dołu (Y środka + 1,27), do VM obróconego sterownika. Pozostałe wyprowadzenie do lokalnego GND. Rysunek nie pokazuje wszystkich lokalnych przewodów VM/GND; należy je wykonać możliwie krótko do właściwych pinów i sprawdzić polaryzację obróconych modułów. Założenia o listwach i osi pinów złącza nie są szablonem do wiercenia.

## Połączenia funkcjonalne

- Chronione 24 V IN → osobne sparowane przewody VM/GND dla lewego banku M1–M3 i prawego M4–M6, z lokalnymi kondensatorami. Rozdział mocy dedykowanymi przewodami; nie łańcuchem cienkich padów ani zwykłym połączeniem sygnałowym. Przekroje i zabezpieczenia zależą od rzeczywistych prądów wejściowych, nie od prostego przemnożenia prądu złącza.
- Z tego samego wejścia 24 V równoległa gałąź → LM2596 → 5 V do Teensy VIN i XIAO 5 V. VM silników nie przechodzi przez LM2596. Uwzględnić właściwe rozdzielenie VIN/VUSB Teensy i ochronę XIAO przed zasilaniem zwrotnym przy USB + zewnętrznym 5 V (według Seeed dioda: anoda od źródła, katoda do 5 V XIAO).
- Poziomy sygnałów, VIO oraz podciąganie I2C Hub8: 3,3 V. Nie łączyć wyjść regulatorów 3,3 V Teensy i XIAO. Ustalić źródło VIO i masę odniesienia w schemacie.
- Teensy → sześć niezależnych par STEP/DIR; EN wspólne, bezpiecznie nieaktywne podczas resetu. Hub8 na I2C, komunikacja XIAO niezależna. Nie przypisano numerów GPIO ani elementów ustalających stany.
- UART V1.3, jeśli potrzebny: dwie osobne grupy po trzy sterowniki, adresy 0/1/2 w każdej. TMC2209 ma tylko cztery adresy; sześciu unikalnych na jednej magistrali nie uzyskamy. U1 pin14 PDN_UART jest połączony z siecią UART_RX i pinem opisanym RX. Pin TX dochodzi do RX przez **R10 oznaczony NC**, zatem fabrycznie nie jest dołączony do tego węzła. W proponowanej topologii używać RX modułów, TX pozostawić niepodłączone; nie trzeba mostkować R10. R9 jest potencjometrem 20 kΩ dla VREF, nie elementem UART. Każdy moduł ma R1 = 20 kΩ podciągający EN do VIO; wspólne EN musi zachować bezpieczny stan nieaktywny przy resecie.
- Przez szczelinę przechodzą wyłącznie sterowanie i zasilanie; **żadne z 24 połączeń fazowych silników nie przecina szczeliny**. To koncepcja, nie pełny rozkład wiązek.
- Każda część wymaga własnych dystansów; przewody i lut nie zastępują połączenia mechanicznego. Posiadany wentylator musi chłodzić cały rząd na obu płytkach. Instrukcja V1.3 z 26 stycznia 2024 r., strona 4, podaje **„Peak Current Support: 2A”**, a strona 8 wymaga aktywnego chłodzenia powyżej 1,2 A. Nie nazywa tego 2 A RMS ani gwarantowanym prądem ciągłym. Ogólna strona BTT podająca 2 A RMS jest niespójna z instrukcją tego wariantu; dla V1.3 zachowujemy ostrożniejsze sformułowanie **2 A peak, prąd ciągły wymaga weryfikacji chłodzenia**. Sprawdzić radiatory, obudowę, temperatury i jednoczesne obciążenie; nie ustalono nastawy prądu firmware.

Przykład funkcjonalny jednej magistrali single-wire, zgodny z ideą ADI Fig. 4.1/4.2 (sygnały 3,3 V, wspólna masa):

```text
Teensy TXn -- 1k --+-- RX M1 (adres 0)
Teensy RXn --------+-- RX M2 (adres 1)
                   +-- RX M3 (adres 2)
```

Dla M4–M6 drugi niezależny UART, ponownie adresy 0/1/2. TX modułów pozostaje wolne; R10 pozostaje NC. To przykład topologii, nie końcowy przydział GPIO ani pełny schemat okablowania; sprawdzić stany, adresowanie i masę przed uruchomieniem.

## Źródła pinoutu i ograniczenia

- [BIGTREETECH V1.3 — instrukcja](https://raw.githubusercontent.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/master/TMC2209/V1.3/manual/BIGTREETECH%20TMC2209%20V1.3%20User%20Manual.pdf)
- [BIGTREETECH V1.3 — katalog schematów i rysunków](https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/tree/master/TMC2209/V1.3/Schematic)
- [V1.3 — zdjęcie pinoutu TOP](https://raw.githubusercontent.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/master/TMC2209/V1.3/Schematic/TMC2209-V1.3-Pin.jpg)
- [V1.3 — schemat](https://raw.githubusercontent.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/master/TMC2209/V1.3/Schematic/TMC2209%20V1.3-SCH.pdf)
- [V1.3 — wymiary](https://raw.githubusercontent.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/master/TMC2209/V1.3/Schematic/TMC2209%20V1.3-SIZE.pdf)
- [Analog Devices — TMC2209, rewizja 1.09, Fig. 4.1/4.2](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf)
- [BTT — TMC2209](https://neo.bttwiki.com/en/docs/accessories-docs/tmc-driver/tmc-2209/)
- [Watterott — warianty pinów SilentStepStick](https://learn.watterott.com/silentstepstick/pinconfig/)

Współrzędne otworów mieszczą się na lokalnej siatce 2,54 mm lewego i prawego banku, ale rzeczywisty początek siatki formatki pozostaje nieznany. Przymierzyć wszystkie części przed cięciem/wierceniem. Skrypt sprawdza bryły w obu częściach (włącznie z R7), kolizje z kondensatorami, 24 unikalne sieci fazowe i ich przypisania przy obrocie, lokalny raster, brak faz przez szczelinę i marginesy. Nie sprawdza obciążalności, termiki, izolacji, padów, wymiarów rzeczywistych części, połączeń wewnętrznych płytki ani pełnej zgodności elektrycznej.
'''
(ROOT/'plan-polaczen.md').write_text(md,encoding='utf-8')
print('PASS: six drivers, six connectors, six capacitors; 24 unique phase nets with correct rotated pin order; no phase link crosses gap; body/cap clearance and R7 fit; local 2.54 mm grids; blank bounds and margins. Normal exit.')
print(f'Straight link distance range: {min(n[6] for n in nets):.2f} to {max(n[6] for n in nets):.2f} mm; not wire cut lengths.')
