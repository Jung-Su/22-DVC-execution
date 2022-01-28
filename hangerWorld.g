world {}

### table

table_base (world) {
    Q:<t(0 0 .6)>
    shape:marker, size:[.03],
}
table (table_base){
    shape:ssBox, Q:<t(0 0 -.05)>, size:[2.3 1.24 .1 .02], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.1
}


### two pandas

Prefix: "L_"
Include: 'panda_convexGripper.g'
joint (table_base L_panda_link0){ joint:rigid Q:<p(-0.4 -0.3 0 0.707107 0 0 0.707107)> }#Q:<t(-.4 -.3 .0) d(90 0 0 1)> }
joint (L_panda_joint7 L_gripper){
  Q:<t(0.194 0 .0) d(90 0 1 0) d(180 0 0 1)>
}
L_gripperBody(L_panda_joint7) {joint:rigid, shape:capsule, size: [.1 .04] Q:<t(0.15 0 .0) d(90 0 1 0)>}

Prefix: "R_"
Include: 'panda_convexGripper.g'
joint (table_base R_panda_link0){ joint:rigid  Q:<p(0.403724 -0.298969 0 0.706412 0 0 0.7078)> }#Q:<t( .4 -.3 .0) d(90 0 0 1)> }
joint (R_panda_joint7 R_gripper){
  Q:<t(0.194 0 0.0) d(90 0 1 0) d(180 0 0 1)>
}
R_gripperBody(R_panda_joint7) {joint:rigid, shape:capsule, size: [.1 .04] Q:<t(0.15 0 .0) d(90 0 1 0)>}
Prefix!


## Calibaration

optitrack_base (world) { Q:<p(-0.0277512 -0.144802 0.600151 -0.709463 -0.000719419 -0.0020549 -0.704739)> }
#Edit l_panda_link0 { Q:<p(-0.4 -0.3 0 0.707107 0 0 0.707107)> }
#Edit r_panda_link0 { Q:<p(0.403724 -0.298969 0 0.706412 0 0 0.7078)> }


# cupHolder
cupHolder_base (table) { 
    Q:<t(-.1 .3 .06) d(40 0 0 1)> shape: cylinder size: [.015 .08] color: [.85 .73 .54]
}

cupHolder_pillar (cupHolder_base) {
    Q:<t(.0 .0 .165)> shape: capsule size: [.33 .0125] color: [.85 .73 .54]
}

U1_hook (cupHolder_base) {
    Q:<t(.0 .0525 .305) d(64 -1 0 0)> shape: capsule size: [.089 .005] color: [.85 .73 .54]

}

U2_hook (cupHolder_base) {
    Q:<t(.0 -.0525 .305) d(64 1 0 0)> shape: capsule size: [.089 .005] color: [.85 .73 .54]
}

U_hook_coll (cupHolder_base) {
    Q:<t(.0 .0 .288)> shape: sphere size: [.02] color: [.9 .9 .9 .1]
}

M1_hook (cupHolder_base) {
    Q:<t(.0525 0 .22) d(64 0 1 0)> shape: capsule size: [.089 .005] color: [.85 .73 .54]

}

M2_hook (cupHolder_base) {
    Q:<t(-.0525 0 .22) d(64 0 -1 0)> shape: capsule size: [.089 .005] color: [.85 .73 .54]

}

M_hook_coll (cupHolder_base) {
    Q:<t(.0 .0 .2)> shape: sphere size: [.02] color: [.9 .9 .9 .1]
}

L1_hook (cupHolder_base) {
    Q:<t(.0 .0525 .137) d(64 -1 0 0)> shape: capsule size: [.089 .005] color: [.85 .73 .54]
}
L2_hook (cupHolder_base) {
    Q:<t(.0 -.0525 .137) d(64 1 0 0)> shape: capsule size: [.089 .005] color: [.85 .73 .54]

}

L_hook_coll (cupHolder_base) {
    Q:<t(.0 .0 .12)> shape: sphere size: [.02] color: [.9 .9 .9 .1]
}
