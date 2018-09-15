Bitte Architektur und Schnittstellendefinition der Module beachten!

scans.txt:
- Output des Lidar
- je Zeile ein vollständiger Scan des Lidar
- insgesammt 200 Zeilen -> 200 Scans
- Darstellung in Polar-Koordinaten

xyCoords:
- Output des Lidar Data Converters
- 1. Scan aus scans.txt umgerechnet in kartesischen XY-Koordinaten
- Lidar bei (0,0)
- Input für Map Generator (SLAM)
