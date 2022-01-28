Include: '../botop/rai-robotModels/scenarios/pandasTable-calibrated.g'

viewCenter(table) { shape:marker, Q:[.5,.2,.1],size:[.1] }


# cupHolder
cupHolder_base (table) {
    Q:<t(-.1 .3 .08) d(90 0 0 1)> shape: cylinder size: [.015 .08] color: [.85 .73 .54]
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

M1_hook (cupHolder_base) {
    Q:<d(20 0 0 1) t(.0525 0 .22) d(64 0 1 0)> shape: capsule size: [.089 .005] color: [.85 .73 .54]

}

M2_hook (cupHolder_base) {
    Q:<d(15 0 0 1) t(-.0525 0 .22) d(64 0 -1 0)> shape: capsule size: [.089 .005] color: [.85 .73 .54]

}


L1_hook (cupHolder_base) {
    Q:<t(.0 .0525 .137) d(64 -1 0 0)> shape: capsule size: [.089 .005] color: [.85 .73 .54]
}
L2_hook (cupHolder_base) {
    Q:<t(.0 -.0525 .137) d(64 1 0 0)> shape: capsule size: [.089 .005] color: [.85 .73 .54]

}
