"""
Navaneeth Compact 40M v2.0 - Reproduzierbares Parametrisches CadQuery / Build123d CAD-Skript
Revision: 2.0 (Alle Geometrie- und Passungskonflikte REV-01 bis REV-07 aufgelöst)
Einheiten: mm | Koordinatensystem: X = Spindelachse, Y = Querachse, Z = Vertikalachse (Spindel Z=150)
"""
import math
import cadquery as cq

# Master-Parameter v2.0
ZC = 150.0
R_FLANGE = 110.0
T_FLANGE = 3.0
R_CORE_BACK = 52.0
R_CORE_FRONT = 48.15
L_CONE = 110.0
D_SPINDLE_K5 = 17.00
D_BEARING_6003_H7 = 35.00
D_GUIDE_SHAFT_K5 = 15.00
D_BEARING_6002_H7 = 32.00

def build_rear_flange_p01():
    # Scheibe Ø220x3 bei X=0..3 + Nabe Ø44x18 bei X=-18..0 + 3x Umreifungsschlitz 16x58 (REV-01)
    disc = cq.Workplane("YZ").workplane(offset=0.0).circle(R_FLANGE).extrude(T_FLANGE)
    hub = cq.Workplane("YZ").workplane(offset=-18.0).circle(22.0).extrude(18.0)
    part = disc.union(hub).faces("<X").workplane().circle(D_SPINDLE_K5/2.0 + 0.01).cutThruAll()
    for deg in [0, 120, 240]:
        part = (part.faces(">X").workplane()
                .transformed(rotate=(0, 0, deg))
                .center(74.0, 0).slot2D(58.0, 16.0, 0).cutThruAll())
    return part.translate((0, 0, ZC))

def build_conical_drum_p02():
    # Konischer Trommelmantel X=3..113, R=52,00 -> 48,15 mm, Wandstärke 2,0 mm
    outer = cq.Solid.makeCone(R_CORE_BACK, R_CORE_FRONT, L_CONE, cq.Vector(3, 0, ZC), cq.Vector(1, 0, 0))
    inner = cq.Solid.makeCone(R_CORE_BACK - 2.0, R_CORE_FRONT - 2.0, L_CONE, cq.Vector(3, 0, ZC), cq.Vector(1, 0, 0))
    return cq.Workplane(obj=outer.cut(inner))

def build_spindle_p07():
    # Hauptspindel Ø17,00 k5 (X=-84..128) + Schulter Ø20 (X=-24..-18) + 3/8" Sechskant (X=-120..-84)
    shaft = cq.Workplane("YZ").workplane(offset=-84.0).circle(D_SPINDLE_K5/2.0).extrude(212.0)
    shoulder = cq.Workplane("YZ").workplane(offset=-24.0).circle(10.0).extrude(6.0)
    hex_stub = cq.Workplane("YZ").workplane(offset=-120.0).polygon(6, 11.0).extrude(36.0)
    return shaft.union(shoulder).union(hex_stub).translate((0, 0, ZC))

if __name__ == "__main__":
    assy = cq.Assembly(name="Navaneeth_Compact_40M_v2_0")
    assy.add(build_rear_flange_p01(), name="P01_Hinterflansch")
    assy.add(build_conical_drum_p02(), name="P02_Konische_Trommel")
    assy.add(build_spindle_p07(), name="P07_Hauptspindel")
    assy.save("Navaneeth_Compact_40M_v2_Assembly.step")
