# Navaneeth Compact 40M v2.0 – Änderungsprotokoll, Montageanleitung und Prüfbericht

## 1. Zusammenfassung der Konstruktionsänderungen gegenüber v1.0 (REV-01 bis REV-07)
- **REV-01 (Umreifungszugang & +X Entnahme):** 3 radiale Schlitze (16 × 58 mm bei 0°, 120°, 240°, R = 45..103 mm mit geschlossenem Außenring R = 103..110 mm) in Hinterflansch P-01 und Vorderflansch P-04 sowie 3 axiale Trommelkanäle (16 × 6 mm) in der konischen Trommel P-02 integriert.
- **REV-02 (Kragarm-Führungsachse & Lager 6002-2RS):** Führungsarm P-12 auf 8,0 mm Wandstärke verstärkt. Führungsachse P-13 an der Wurzel auf Ø16,0 mm und am Lagersitz auf **Ø15,00 k5** (passend zur 15,00-mm-Normbohrung der zwei SKF 6002-2RS Lager!) ausgelegt. Führungsrolle P-15 auf Ø38,0 mm (Lagersitze Ø32,00 H7) vergrößert. Sicherheitsfaktor steigt von SF = 1,05 auf SF = 6,94.
- **REV-03 (Hauptspindel, Lager 6003-2RS & Naben-Mitnahme):** Spindel P-07 auf **Ø17,00 k5** geschliffen (behebt 0,2-mm-Wurfpassung aus v1.0) mit Anlaufschulter Ø20,0 mm und Innenring-Distanzhülse P-10. Lagergehäuse P-09 mit 2× Lagersitz **Ø35,00 H7** und Innenschulter Ø31,0 mm versehen. Formschnittige Drehmomentübertragung über Querschraube M6×50 FK 8.8 (F-21) bei X = -9,0 mm.
- **REV-04 (Bohrschrauber-Drehmomentstütze):** Rahmenfeste Drehmomentstütze P-21 mit NBR-gepolsterter Griffgabel (45 × 36 mm) auf Fußplatte P-20 ergänzt; nimmt 20 Nm Reaktionsmoment (235,3 N Querlast am Griff) auf (SF = 8,16).
- **REV-05 (Bodenfreiheit ≥ 12,0 mm):** Fußplatte P-20 auf Z = 12,0..18,0 mm angehoben (auf 4× 12-mm-Elastomerpads P-23B); alle M6-Rahmenschrauben F-17..F-20 von unten in +Z-Richtung mit versenkten Köpfen montiert.
- **REV-06 (4-Spur-Bandführung):** 4-Spur-Führungsschlitten P-16 (Spurzentren X = 19, 45, 71, 97 mm) führt das einzelne durchgehende 40-m-Band über 4 Spuren à 10 m (R_coil = 96,58 mm < R_flange = 110,0 mm).
- **REV-07 (Ventilstiel & Zugentlastung):** 2-mm-Lücke am Ventilstiel (Z = 28..32 mm) geschlossen und rahmenfeste Band-Zugentlastungsklemme P-22 an rechter Bodenschiene ergänzt.

## 2. Web-Viewer & Kinematik-Überarbeitung (60-FPS Koaxial-Engine)
- **Exakte Koaxiale Rotationsachsen (0,00 mm Exzentrizität):** Alle rotierenden Baugruppen der Hauptspindel (`P-01` bis `P-07`, `P-10`, `REF-01` sowie die Mitnehmerschrauben `F-11..F-16` und `F-21`) besitzen ihren lokalen Drehpunkt exakt auf der Spindelmittellinie `(Y = 0,0 mm, Z = 150,0 mm)`. Die obere Führungsrolle `P-15` rotiert koaxial um `(Y = 0,0 mm, Z = 285,0 mm)` synchron zur Bandgeschwindigkeit `ω_rolle = ω_spindel × (R_coil / 19,0 mm)`.
- **Physikalisch exakter 4-Spur-Wickelaufbau & Bandlauf:** Der Innenradius jeder der 4 Bandspuren (`X = 6,5..109,5 mm`) bleibt fest auf dem 2°-Trommelkonus (`R_core = 51,45 → 48,72 mm`), während der Außenradius kontinuierlich bis `R_coil = 96,58 mm` anwächst. Das zulaufende 25-mm-Flachband (`REF-03`) wandert synchron mit dem Führungsschlitten `P-16` über die 4 Spuren (`X = 19, 45, 71, 97 mm`) mit animierten Tropfer-Markierungen.
- **60-FPS-Performance & Dual-Engine (WebGL + Offline-3D/2D-Canvas):** Vermeidung von DOM-Reflows in der Animationsschleife, In-Place-Vertex-Updates ohne Speicherallokation, Drag-Schwellenwert für Kamera-Orbit sowie ein synchronisierter **Live 2D-Kinematik-Querschnitt (Längsschnitt X-Z & Seitenansicht Y-Z)** in beiden HTML-Dateien.

## 3. Dateiinventar des Komplettpakets (Revision 2.0)
- `Navaneeth_Compact_40M_Interactive_CAD.html` – Interaktiver 3D-CAD- & 2D-Kinematik-Simulator (60 FPS, koaxial, inkl. STL/CSV/JSON-Export)
- `Navaneeth_Compact_40M_Project.html` – Technisches Projektportal mit integriertem 60-FPS-Kinematiksimulator, Live-Berechnungsmodell, BOM und Zeichnungen
- `Navaneeth_Compact_40M_Engineering_Report_DE.pdf` – 12-seitiger deutscher Ingenieur- und Konstruktionsbericht
- `Navaneeth_Compact_40M_Calculations_BOM_DE.xlsx` – 12-Blatt-Excel-Berechnungsmappe mit vollständig evaluierten Formeln, BOM, Fastener- und Verbindungsregister
- `Drawings/Navaneeth_Compact_40M_v2_Technical_Drawings_DE.pdf` – 6-Blatt-Fertigungs-, Baugruppen- und Explosionszeichnungen
- `CAD/GLB/Navaneeth_Compact_40M_v2_Assembled.glb` & `Navaneeth_Compact_40M_v2_Exploded.glb` – 3D-GLB-Baugruppenmodelle
- `CAD/STEP/Navaneeth_Compact_40M_v2_Assembly.step` & `CAD/STEP/parts/*.step` – 28 ISO-10303-21 AP214 STEP-Dateien
- `CAD/STL/Navaneeth_Compact_40M_v2_Complete_Assembly.stl` & `CAD/STL/*.stl` – 28 binäre STL-Dateien
- `CAD/Parametric_Scripts/` – Reproduzierbare parametrische CadQuery-, OpenSCAD- und Python-CAD-Skripte
