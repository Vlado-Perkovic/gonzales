/* =====================================================================
   Parametric snap-fit enclosure for a 5x7 cm (50 x 70 mm) protoboard
   ---------------------------------------------------------------------
   * PCB snaps onto 4 solid corner posts: each boss carries a solid pin
     (pin_d, slightly oversize) that press-fits into the board's 2 mm
     corner holes - no screws needed. Tune pin_d for grip.
   * Snap-on lid: a bead on the lid skirt snaps under a rib ring cast
     into the box walls.  Pry notches at front/back rim for opening.
   * Rectangular opening in the bottom, rectangular opening in the
     front (50 mm / "5 cm") side.  Everything is parametric.
   * Printing: box as-is, lid as-is (plate on the bed), no supports.
     If the snap is too tight / too loose, tweak bead_p, rib_p or
     lip_clear in 0.05 mm steps.

   part = "box" | "lid" | "both" (for printing) | "preview" (assembled)
   ===================================================================== */

$fn = 64;

/* ---------------- 1. PCB ---------------- */
pcb_w          = 50;     // pcb width  (the 5 cm / front side)
pcb_l          = 70;     // pcb length (the 7 cm side)
pcb_t          = 1.6;    // pcb thickness (reference only)
pcb_hole_d     = 2.0;    // corner mounting hole diameter
pcb_hole_inset = 4.0;    // hole CENTRE distance from pcb edge -> MEASURE YOUR BOARD
pcb_side_clear = 0.4;    // pcb-edge to inner-wall gap, per side

/* ---------------- 2. Shell ---------------- */
wall    = 2.0;           // wall thickness
floor_t = 2.0;           // bottom thickness
box_h   = 30;            // TOTAL outside height with lid closed

/* pcb snap posts */
standoff_d = 5.6;        // base boss diameter (pcb rests on its top)
standoff_h = 5.0;        // pcb underside height above the floor
pin_d      = 2.1;        // snap pin diameter: pcb holes are 2.0, so this
                         // press-fits. +0.05 = tighter, -0.05 = looser
pin_above  = 0.7;        // how far the pin sticks out above the pcb top
pin_lead   = 0.8;        // lead-in cone height on the pin tip

/* ---------------- 3. Openings ---------------- */
/* bottom opening */
bh_w           = 10;     // size in x
bh_l           = 10;     // size in y
bh_cx          = 0;      // x centre offset (0 = centred across the width)
bh_y_from_back = 10;     // hole CENTRE distance from the OUTER back edge
                         // (back = the pcb_w side opposite the front opening)

/* front opening (in the pcb_w / 5 cm side) */
fh_w  = 12;              // opening width
fh_h  = 8;               // opening height
fh_x  = 8;               // opening left edge from OUTER left box edge
fh_cz = 15;              // centre height of the opening

/* ---------------- 4. Lid & snap fit ---------------- */
lid_t      = 2.0;        // lid plate thickness
lip_t      = 2.0;        // lid skirt wall thickness
lip_h      = 6.0;        // lid skirt depth
lip_clear  = 0.4;        // skirt-to-wall clearance
bead_p     = 0.5;        // snap bead protrusion on the skirt
bead_h     = 1.6;        // snap bead height
bead_top_h = 0.2;        // insertion ramp, horizontal
bead_top_v = 0.15;       // insertion ramp, vertical
bead_bot_h = 0.15;       // retention chamfer, horizontal
bead_bot_v = 0.15;       // retention chamfer, vertical
rib_p      = 0.3;        // box snap rib protrusion
rib_h      = 1.6;        // box snap rib height
entry_ch   = 1.2;        // lead-in chamfer at the box mouth
pry_notch   = true;      // finger notches in the rim for prying the lid off
pry_notch_d = 10;        // notch diameter

/* ---------------- 5. What to show ---------------- */
part    = "both";        // "box" | "lid" | "both" | "preview"
cutaway = true;          // preview only: cut the box open to see inside

/* ============ derived values (do not edit) ============ */
in_w  = pcb_w + 2*pcb_side_clear;   // cavity width
in_l  = pcb_l + 2*pcb_side_clear;   // cavity length
out_w = in_w + 2*wall;              // outside width
out_l = in_l + 2*wall;              // outside length
rim_z = box_h - lid_t;              // wall top plane (lid plate seats here)
rib_z = rim_z - lip_h + bead_h;     // snap rib bottom plane

/* ===================================================================== */

module standoff() {
    union() {
        cylinder(d = standoff_d, h = standoff_h);                  // base boss
        cylinder(d = pin_d, h = standoff_h + pcb_t + pin_above);   // solid snap pin
        translate([0, 0, standoff_h + pcb_t + pin_above])          // lead-in tip
            cylinder(d1 = pin_d, d2 = 0.01, h = pin_lead);
    }
}

/* snap bead: runs -Y from origin, protrudes +X, base at z=0 */
module snap_bead(len) {
    rotate([90, 0, 0])
        linear_extrude(len)
            polygon([
                [0, 0],
                [bead_p - bead_bot_h, 0],
                [bead_p, bead_bot_v],
                [bead_p, bead_h - bead_top_v],
                [bead_p - bead_top_h, bead_h],
                [0, bead_h]
            ]);
}

module enclosure() {
    difference() {
        union() {
            difference() {
                cube([out_w, out_l, rim_z]);
                /* cavity */
                translate([wall, wall, floor_t - 0.01])
                    cube([in_w, in_l, rim_z]);
                /* lead-in chamfer at the mouth */
                translate([wall, wall, rim_z - entry_ch])
                    hull() {
                        linear_extrude(0.01) square([in_w, in_l]);
                        translate([0, 0, entry_ch - 0.02])
                            linear_extrude(0.01)
                                offset(delta = entry_ch) square([in_w, in_l]);
                    }
            }
            /* snap rib ring, cast into all four walls */
            translate([wall, wall, rib_z])
                linear_extrude(rib_h)
                    difference() {
                        square([in_w, in_l]);
                        translate([rib_p, rib_p])
                            square([in_w - 2*rib_p, in_l - 2*rib_p]);
                    }
            /* pcb standoffs at the corner hole positions */
            sxy = wall + pcb_side_clear + pcb_hole_inset;
            for (x = [sxy, out_w - sxy], y = [sxy, out_l - sxy])
                translate([x, y, floor_t]) standoff();
        }
        /* bottom opening: centred in x, offset from the back edge */
        translate([out_w/2 + bh_cx - bh_w/2, out_l - bh_y_from_back - bh_l/2, -1])
            cube([bh_w, bh_l, floor_t + 2]);
        /* front opening (front wall is at y = 0) */
        translate([fh_x, -1, fh_cz - fh_h/2])
            cube([fh_w, wall + 2, fh_h]);
        /* pry notches in the rim */
        if (pry_notch) {
            translate([out_w/2, wall/2, rim_z])
                rotate([90, 0, 0])
                    cylinder(d = pry_notch_d, h = wall + 2, center = true);
            translate([out_w/2, out_l - wall/2, rim_z])
                rotate([90, 0, 0])
                    cylinder(d = pry_notch_d, h = wall + 2, center = true);
        }
    }
}

/* lid modelled in printing orientation: plate on the bed, skirt up */
module lid() {
    x0 = wall + lip_clear;              // skirt ring corner
    lw = in_w - 2*lip_clear;            // skirt outer width
    ll = in_l - 2*lip_clear;            // skirt outer length
    cc = lip_t + 1.5;                   // bead end clearance from skirt corners
    bx0 = x0 + cc;  bx1 = out_w - x0 - cc;
    by0 = x0 + cc;  by1 = out_l - x0 - cc;
    zb  = lid_t + lip_h - bead_h;       // bead base height
    union() {
        cube([out_w, out_l, lid_t]);    // plate, flush with box outside
        translate([x0, x0, lid_t])      // skirt
            linear_extrude(lip_h)
                difference() {
                    square([lw, ll]);
                    translate([lip_t, lip_t])
                        square([lw - 2*lip_t, ll - 2*lip_t]);
                }
        /* snap beads, one per skirt face */
        translate([x0 + lw, by1, zb])  snap_bead(by1 - by0);                    // right  (+X)
        translate([x0,      by1, zb])  mirror([1, 0, 0])
                                       snap_bead(by1 - by0);                    // left   (-X)
        translate([bx0, x0 + ll, zb])  rotate([0, 0, 90])
                                       snap_bead(bx1 - bx0);                    // back   (+Y)
        translate([bx0, x0,      zb])  mirror([0, 1, 0]) rotate([0, 0, 90])
                                       snap_bead(bx1 - bx0);                    // front  (-Y)
    }
}

/* ------------------------- output selector ------------------------- */
if (part == "box") {
    enclosure();
} else if (part == "lid") {
    lid();
} else if (part == "both") {
    enclosure();
    translate([out_w + 20, 0, 0]) lid();
} else if (part == "preview") {
    difference() {
        enclosure();
        if (cutaway)
            translate([-1, out_l/2, -1])
                cube([out_w + 2, out_l + 2, box_h + 2]);
    }
    /* lid ghost, shown assembled */
    %translate([0, out_l, rim_z + lid_t]) rotate([180, 0, 0]) lid();
}
