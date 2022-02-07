**<a href="https://gitlab.liu.se/cvl/tsbb18/-/blob/master/README.md">&larr; Tillbaka till huvudsidan</a>**

# Robotsimulator

**Tok-kortfattad instruktion:**

- Installera senaste **<a href="https://www.blender.org/">Blender</a>**.
- Ladda ned **<a href="https://gitlab.liu.se/cvl/tsbb18/-/blob/master/robot_sim/robot.blend">kursens robotsimulator</a>** och **<a href="https://gitlab.liu.se/cvl/tsbb18/-/blob/master/robot_sim/client.py">klientscriptet</a>**.
- Starta Blender från en terminal, för att kunna se eventuella felmeddelanden eller för att kunna se vad som skrivs ut med <code>print()</code> eller <code>capture()</code>.
- Öppna <code>robot.blend</code> i Blender, tryck på play-knappen i texteditorn. En text längst ner i mitten av Blender-fönstret kommer blinka att servern är igång. Det går inte att använda programmet till annat medan den kör. Vill man flytta till en ny vy i 3D-fönstret, så måste man stänga av den igen med escape eller vänster musknapp.
- Starta klienten separat, t.ex. från en annan terminal eller ett IDE på samma dator.

**I 3D-fönstret:**

- Rotera med mitt-musknappen + drag
- Panorera med skift + mitt-musknappen + drag
- Zooma med scrollhjulet
- Sätt kameran (för renderingen) till aktuell vy genom <code>Ctrl</code>+<code>Alt</code>+<code>Num 0</code>

Om något slutar fungera, starta om Blender. (Enklast så.)

**Updateringar**

* 2021-02-04
    - Nytt interface. Det mesta ska gå att göra via klienten.
    - Nu går det att lägga ut legobitar, slumpmässigt eller på specifik plats.
    - Servern, och inte klienten, animerar roboten  
    - Buggfix: Proben rapporterade fel position  
