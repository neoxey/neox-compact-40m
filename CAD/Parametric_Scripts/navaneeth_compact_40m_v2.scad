// Navaneeth Compact 40M v2.0 - Parametrisches OpenSCAD-Modell (Revision 2.0)
// Einheiten: mm | Spindelachse bei Y = 0, Z = 150 mm
$fn = 72;
zc = 150.0;

module flange_220(has_bolts=true) {
    difference() {
        cylinder(h=3.0, r=110.0);
        translate([0,0,-1]) cylinder(h=5.0, r=8.51);
        for (a = [0, 120, 240])
            rotate([0, 0, a]) translate([45, -8, -1]) cube([58, 16, 5]);
        for (a = [30, 60, 150, 180, 270, 300])
            rotate([0, 0, a]) translate([82, 0, -1]) cylinder(h=5, r=9);
        if (has_bolts)
            for (a = [30, 90, 150, 210, 270, 330])
                rotate([0, 0, a]) translate([38, 0, -1]) cylinder(h=5, r=2.75);
    }
}

// Baugruppe aufbauen
translate([0, 0, zc]) rotate([0, 90, 0]) {
    color([0.8, 0.82, 0.84]) flange_220(true);
    color([0.24, 0.59, 0.51]) translate([0, 0, 3])
        difference() {
            cylinder(h=110.0, r1=52.0, r2=48.15);
            translate([0,0,-0.1]) cylinder(h=110.2, r1=50.0, r2=46.15);
        }
    color([0.8, 0.82, 0.84]) translate([0, 0, 113]) flange_220(false);
    color([0.5, 0.52, 0.54]) translate([0, 0, -84]) cylinder(h=212.0, r=8.50);
}
