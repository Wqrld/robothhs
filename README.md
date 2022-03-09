# Todo;
naar github repo moven zodat je mooi die progress kan laten zien? 

# DOEL 1: Het robotwagentje is in staat om een statisch doel te vinden en ernaar toe te
rijden.
```python
servo links
zet hoogste naar 0
loop n times:
  servo 10* naar rechts
  scan waarde en if groter dan lastgrootste zet lastgrootste = huidige
pak hoogste, 
if hoogste < threshold draai 45-90* en repeat
if hoogste > threshold draai die richting op en rijd 0.5m vooruit, then repeat. 
```

optimization: als n keer lager dan hoogste heb je hem al gevonden en hoef je niet verder te scannen.

# DOEL 2: Het robotwagentje is in staat om een statisch doel te vinden terwijl er
obstakels aanwezig zijn en het robotwagentje kan naar het doel toe rijden terwijl het
de obstakels vermijdt.

Idem als hierboven, voor je rijdt eerst afstand meten tot obstakel, if dist < threshold draai 90%*, meet again, if geblokkeerd again 180* naar links en probeer die kant. Pas als laatste weer terug naar achteren (aan 3 kanten ingesloten) 

(kan geoptimaliseerd worden als je het parkour kent)

# DOEL 3: Het robotwagentje is in staat om een bewegend doel te vinden terwijl er
obstakels aanwezig zijn en het robotwagentje kan naar het bewegende doel toe rijden
terwijl het de obstakels vermijdt.

idem als hierboven, theoretisch hoef je niks aan te passen (?)


# DOEL 4: Hetrobotwagentje isin staat om als “jager” en als “prooi” te werken. Alsjager
kan het robotwagentje een ander robotwagentje detecteren, opjagen en aantikken.
Als prooi kan het robotwagentje een ander robotwagentje detecteren en dan
proberen te ontsnappen


eigenlijk hetzelfde als nr.2 maar dan kijken waar de tegenstander is en exact de andere kant op willen, optimaliseren op zijwaards en erlangs ipv in een hoekje drijven afhankelijk van het parkour.