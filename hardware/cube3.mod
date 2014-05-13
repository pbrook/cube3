PCBNEW-LibModule-V1  Tue 13 May 2014 01:20:50 BST
# encoding utf-8
Units mm
$INDEX
CP2V10
Crystal
HACKSPACE_LOGO
HOLE3
LOGO_SMALL
OSMW_LOGO
PIN1
SIL-36
isp
power2
rgb_cathode
to252
$EndINDEX
$MODULE CP2V10
Po 0 0 0 15 536447B3 00000000 ~~
Li CP2V10
Cd Condensateur polarise
Kw CP
Sc 0
AR /53627749
Op 0 0 0
T0 0 2.54 1.27 1.27 0 0.254 N V 21 N "C7"
T1 0 -2.54 1.27 1.27 0 0.254 N V 21 N "2200u"
T2 -4.445 0 1 1 0 0.15 N V 21 N "+"
DC 0 0 4.826 -2.794 0.3048 21
$PAD
Sh "1" R 1.778 1.778 0 0 0
Dr 1.016 0 0
At STD N 00E0FFFF
Ne 1 "/5v"
Po -2.54 0
$EndPAD
$PAD
Sh "2" C 1.778 1.778 0 0 0
Dr 1.016 0 0
At STD N 00E0FFFF
Ne 2 "/GND"
Po 2.54 0
$EndPAD
$SHAPE3D
Na "discret/c_vert_c2v10.wrl"
Sc 1 1 1
Of 0 0 0
Ro 0 0 0
$EndSHAPE3D
$EndMODULE CP2V10
$MODULE Crystal
Po 0 0 0 15 5364466B 00000000 ~~
Li Crystal
Cd Crystal
Kw R
Sc 0
AR /53626143
Op 0 A 0
T0 0 0.127 1.397 1.27 0 0.2032 N V 21 N "X1"
T1 0 1.905 1.397 1.27 0 0.2032 N I 21 N "16MHz"
DS 2.54 2.54 -2.54 2.54 0.15 21
DS -2.54 -2.54 2.54 -2.54 0.15 21
DA -2.54 0 -2.54 2.54 1800 0.15 21
DA 2.54 0 2.54 -2.54 1800 0.15 21
$PAD
Sh "1" C 1.397 1.397 0 0 0
Dr 0.635 0 0
At STD N 00E0FFFF
Ne 0 ""
Po -2.4384 0
$EndPAD
$PAD
Sh "2" C 1.397 1.397 0 0 0
Dr 0.635 0 0
At STD N 00E0FFFF
Ne 0 ""
Po 2.4384 0
$EndPAD
$SHAPE3D
Na "Device/crystal_low_profile.wrl"
Sc 1 1 1
Of 0 0 0
Ro 0 0 0
$EndSHAPE3D
$EndMODULE Crystal
$MODULE HACKSPACE_LOGO
Po 0 0 0 15 53713155 00000000 ~~
Li HACKSPACE_LOGO
Sc 0
AR 
Op 0 0 0
T0 0 0 1.524 1.524 0 0.3048 N I 21 N "HACKSPACE_LOGO"
T1 0 2.54 1.524 1.524 0 0.3048 N I 21 N "VAL**"
DC 0 0 0 16.51 1.27 21
DS 3.81 0 5.08 1.27 1.27 21
DS 5.08 1.27 3.81 2.54 1.27 21
DS 3.81 2.54 7.62 6.35 1.27 21
DS 7.62 6.35 13.97 0 1.27 21
DS 13.97 0 10.16 -3.81 1.27 21
DS 10.16 -3.81 8.89 -2.54 1.27 21
DS 8.89 -2.54 2.54 -8.89 1.27 21
DS 2.54 -8.89 3.81 -10.16 1.27 21
DS 3.81 -10.16 0 -13.97 1.27 21
DS 0 -13.97 -6.35 -7.62 1.27 21
DS -6.35 -7.62 -2.54 -3.81 1.27 21
DS -2.54 -3.81 -1.27 -5.08 1.27 21
DS -1.27 -5.08 0 -3.81 1.27 21
DS 0 -3.81 -3.81 0 1.27 21
DS -3.81 0 -5.08 -1.27 1.27 21
DS -5.08 -1.27 -3.81 -2.54 1.27 21
DS -3.81 -2.54 -7.62 -6.35 1.27 21
DS -7.62 -6.35 -13.97 0 1.27 21
DS -13.97 0 -10.16 3.81 1.27 21
DS -10.16 3.81 -8.89 2.54 1.27 21
DS -8.89 2.54 -2.54 8.89 1.27 21
DS -2.54 8.89 -3.81 10.16 1.27 21
DS -3.81 10.16 0 13.97 1.27 21
DS 0 13.97 6.35 7.62 1.27 21
DS 6.35 7.62 2.54 3.81 1.27 21
DS 2.54 3.81 1.27 5.08 1.27 21
DS 1.27 5.08 0 3.81 1.27 21
DS 3.81 0 0 3.81 1.27 21
$EndMODULE HACKSPACE_LOGO
$MODULE HOLE3
Po 0 0 0 15 53645D0C 00000000 ~~
Li HOLE3
Sc 0
AR /53548167
Op 0 0 0
T0 0 0 1.524 1.524 0 0.3048 N I 21 N "P8"
T1 0 0 1.524 1.524 0 0.3048 N I 21 N "CONN_1"
DC 0 0 3 0 0.15 21
$PAD
Sh "1" C 4.064 4.064 0 0 0
Dr 3.048 0 0
At STD N 00E0FFFF
Ne 0 ""
Po 0 0
.ZoneConnection 2
$EndPAD
$EndMODULE HOLE3
$MODULE LOGO_SMALL
Po 0 0 0 15 53713210 00000000 ~~
Li LOGO_SMALL
Sc 0
AR 
Op 0 0 0
T0 0 -5.08 1.524 1.524 0 0.3048 N I 21 N "LOGO_SMALL"
T1 0 -5.08 1.524 1.524 0 0.3048 N I 21 N "VAL**"
DS 0.762 0 0 0.762 0.2032 21
DC 0 0 0 3.302 0.2032 21
DS 1.778 -0.508 0.508 -1.778 0.2032 21
DS 0.508 -1.778 0.762 -2.032 0.2032 21
DS 0.762 -2.032 0 -2.794 0.2032 21
DS 0 -2.794 -1.27 -1.524 0.2032 21
DS -1.27 -1.524 -0.508 -0.762 0.2032 21
DS -0.508 -0.762 -0.254 -1.016 0.2032 21
DS -0.254 -1.016 0 -0.762 0.2032 21
DS 0 -0.762 -0.762 0 0.2032 21
DS -0.762 0 -1.016 -0.254 0.2032 21
DS -1.016 -0.254 -0.762 -0.508 0.2032 21
DS -0.762 -0.508 -1.524 -1.27 0.2032 21
DS -1.524 -1.27 -2.794 0 0.2032 21
DS -2.794 0 -1.778 1.016 0.2032 21
DS -1.778 1.016 -1.524 0.762 0.2032 21
DS -1.524 0.762 -0.508 1.778 0.2032 21
DS -0.508 1.778 -0.762 2.032 0.2032 21
DS -0.762 2.032 0 2.794 0.2032 21
DS 0 2.794 1.27 1.524 0.2032 21
DS 1.27 1.524 0.508 0.762 0.2032 21
DS 0.508 0.762 0.254 1.016 0.2032 21
DS 0.254 1.016 0 0.762 0.2032 21
DS 0.762 0 1.016 0.254 0.2032 21
DS 1.016 0.254 0.762 0.508 0.2032 21
DS 0.762 0.508 1.524 1.27 0.2032 21
DS 1.524 1.27 2.794 0 0.2032 21
DS 2.794 0 2.032 -0.762 0.2032 21
DS 2.032 -0.762 1.778 -0.508 0.2032 21
$EndMODULE LOGO_SMALL
$MODULE OSMW_LOGO
Po 0 0 0 15 53713168 00000000 ~~
Li OSMW_LOGO
Sc 0
AR 
Op 0 0 0
T0 0 6.73608 0.57912 0.57912 0 0.1143 N I 21 N "G***"
T1 0 -6.73608 0.57912 0.57912 0 0.1143 N I 21 N "LOGO"
DP 0 0 0 0 277 0.00254 21
Dl -3.85064 5.70484
Dl -3.78206 5.66928
Dl -3.63474 5.5753
Dl -3.42138 5.43814
Dl -3.17246 5.26796
Dl -2.921 5.09778
Dl -2.71272 4.96062
Dl -2.56794 4.86664
Dl -2.50698 4.83362
Dl -2.4765 4.84632
Dl -2.35458 4.90474
Dl -2.18186 4.99364
Dl -2.0828 5.04444
Dl -1.92278 5.11556
Dl -1.84404 5.12826
Dl -1.83134 5.1054
Dl -1.77292 4.98348
Dl -1.68148 4.77774
Dl -1.55956 4.50088
Dl -1.4224 4.1783
Dl -1.27508 3.83032
Dl -1.12776 3.47726
Dl -0.98806 3.13944
Dl -0.8636 2.83464
Dl -0.76454 2.58826
Dl -0.6985 2.41554
Dl -0.67564 2.34188
Dl -0.68326 2.32664
Dl -0.762 2.2479
Dl -0.9017 2.14376
Dl -1.20142 1.89992
Dl -1.4986 1.53162
Dl -1.67894 1.10998
Dl -1.73736 0.64262
Dl -1.68656 0.20828
Dl -1.51638 -0.20828
Dl -1.22682 -0.58166
Dl -0.87376 -0.86106
Dl -0.46228 -1.03632
Dl 0 -1.0922
Dl 0.44196 -1.04394
Dl 0.86868 -0.8763
Dl 1.24206 -0.59182
Dl 1.39954 -0.40894
Dl 1.61798 -0.02794
Dl 1.74244 0.37846
Dl 1.75514 0.4826
Dl 1.73736 0.9271
Dl 1.60528 1.35382
Dl 1.36906 1.73736
Dl 1.04394 2.04978
Dl 1.0033 2.08026
Dl 0.8509 2.19202
Dl 0.7493 2.27076
Dl 0.67056 2.3368
Dl 1.23698 3.70332
Dl 1.32842 3.92176
Dl 1.48336 4.29514
Dl 1.62052 4.61518
Dl 1.72974 4.87172
Dl 1.80594 5.04444
Dl 1.83896 5.11302
Dl 1.84404 5.11556
Dl 1.8923 5.12572
Dl 1.99898 5.08762
Dl 2.18694 4.99364
Dl 2.31394 4.93014
Dl 2.45872 4.85902
Dl 2.52476 4.83362
Dl 2.58064 4.8641
Dl 2.72034 4.95554
Dl 2.92354 5.09016
Dl 3.16738 5.25526
Dl 3.39852 5.41528
Dl 3.61442 5.55498
Dl 3.7719 5.65658
Dl 3.84556 5.69722
Dl 3.85826 5.69722
Dl 3.92684 5.65912
Dl 4.04876 5.55498
Dl 4.23418 5.37972
Dl 4.49834 5.12064
Dl 4.53898 5.08
Dl 4.75488 4.85902
Dl 4.93014 4.67614
Dl 5.04952 4.54406
Dl 5.09016 4.4831
Dl 5.09016 4.4831
Dl 5.05206 4.40944
Dl 4.953 4.2545
Dl 4.81076 4.03606
Dl 4.63804 3.78206
Dl 4.18592 3.1242
Dl 4.43484 2.50444
Dl 4.51104 2.31394
Dl 4.60756 2.0828
Dl 4.68122 1.92024
Dl 4.71678 1.84658
Dl 4.78536 1.82372
Dl 4.95554 1.78308
Dl 5.19938 1.73228
Dl 5.49402 1.6764
Dl 5.77342 1.6256
Dl 6.02742 1.57734
Dl 6.2103 1.54178
Dl 6.29158 1.52654
Dl 6.3119 1.51384
Dl 6.32714 1.4732
Dl 6.33984 1.38938
Dl 6.34492 1.23444
Dl 6.34746 0.99314
Dl 6.34746 0.64262
Dl 6.34746 0.60452
Dl 6.34492 0.26924
Dl 6.33984 0.00508
Dl 6.32968 -0.17018
Dl 6.31952 -0.23876
Dl 6.31698 -0.23876
Dl 6.23824 -0.25908
Dl 6.06044 -0.29718
Dl 5.80644 -0.34544
Dl 5.50672 -0.40386
Dl 5.4864 -0.4064
Dl 5.18668 -0.46482
Dl 4.93522 -0.51816
Dl 4.75996 -0.5588
Dl 4.68376 -0.58166
Dl 4.66852 -0.60198
Dl 4.60756 -0.71882
Dl 4.52374 -0.90678
Dl 4.42214 -1.13284
Dl 4.32562 -1.36906
Dl 4.23926 -1.57988
Dl 4.18338 -1.7399
Dl 4.1656 -1.81102
Dl 4.1656 -1.81356
Dl 4.21132 -1.88468
Dl 4.31546 -2.03962
Dl 4.46278 -2.25552
Dl 4.6355 -2.51206
Dl 4.65074 -2.53238
Dl 4.82346 -2.78638
Dl 4.96316 -2.99974
Dl 5.05714 -3.15214
Dl 5.09016 -3.22072
Dl 5.09016 -3.2258
Dl 5.03428 -3.302
Dl 4.90474 -3.44678
Dl 4.71932 -3.63982
Dl 4.4958 -3.86334
Dl 4.42468 -3.93446
Dl 4.1783 -4.17322
Dl 4.00812 -4.33324
Dl 3.90144 -4.41706
Dl 3.85064 -4.43484
Dl 3.8481 -4.4323
Dl 3.7719 -4.38658
Dl 3.61188 -4.28244
Dl 3.39344 -4.13512
Dl 3.1369 -3.95986
Dl 3.11658 -3.94716
Dl 2.86512 -3.77698
Dl 2.65176 -3.6322
Dl 2.5019 -3.53314
Dl 2.43586 -3.4925
Dl 2.4257 -3.4925
Dl 2.32156 -3.52298
Dl 2.14122 -3.58648
Dl 1.92024 -3.67284
Dl 1.68402 -3.76682
Dl 1.47066 -3.85826
Dl 1.31064 -3.93192
Dl 1.23444 -3.97256
Dl 1.23444 -3.97764
Dl 1.2065 -4.06908
Dl 1.16332 -4.25704
Dl 1.10998 -4.51866
Dl 1.04902 -4.83108
Dl 1.0414 -4.87934
Dl 0.98298 -5.18414
Dl 0.93472 -5.43306
Dl 0.89916 -5.60578
Dl 0.88138 -5.6769
Dl 0.8382 -5.68706
Dl 0.69088 -5.69722
Dl 0.46482 -5.7023
Dl 0.19304 -5.70738
Dl -0.09652 -5.70484
Dl -0.37592 -5.69722
Dl -0.61722 -5.6896
Dl -0.7874 -5.6769
Dl -0.85852 -5.6642
Dl -0.86106 -5.65912
Dl -0.889 -5.56514
Dl -0.92964 -5.37464
Dl -0.98298 -5.11302
Dl -1.04394 -4.8006
Dl -1.0541 -4.74472
Dl -1.10998 -4.445
Dl -1.16078 -4.19608
Dl -1.19888 -4.0259
Dl -1.21666 -3.95986
Dl -1.2446 -3.94462
Dl -1.36906 -3.89128
Dl -1.57226 -3.80746
Dl -1.82118 -3.70586
Dl -2.40284 -3.46964
Dl -3.11658 -3.95986
Dl -3.18008 -4.00304
Dl -3.43662 -4.1783
Dl -3.64744 -4.318
Dl -3.79476 -4.41198
Dl -3.85572 -4.44754
Dl -3.8608 -4.445
Dl -3.93192 -4.3815
Dl -4.07162 -4.24942
Dl -4.26466 -4.06146
Dl -4.49072 -3.83794
Dl -4.65582 -3.67284
Dl -4.8514 -3.47218
Dl -4.97586 -3.33756
Dl -5.04444 -3.2512
Dl -5.06984 -3.2004
Dl -5.06222 -3.16484
Dl -5.0165 -3.09118
Dl -4.91236 -2.93624
Dl -4.76504 -2.7178
Dl -4.59232 -2.46634
Dl -4.45008 -2.25552
Dl -4.29514 -2.01676
Dl -4.19354 -1.84658
Dl -4.15798 -1.76276
Dl -4.16814 -1.7272
Dl -4.2164 -1.5875
Dl -4.30276 -1.37668
Dl -4.40944 -1.12522
Dl -4.65582 -0.56134
Dl -5.02666 -0.49022
Dl -5.25018 -0.44958
Dl -5.56514 -0.38862
Dl -5.86486 -0.3302
Dl -6.33222 -0.23876
Dl -6.35 1.48336
Dl -6.27634 1.51384
Dl -6.20776 1.53162
Dl -6.03504 1.56972
Dl -5.78612 1.62052
Dl -5.49402 1.67386
Dl -5.2451 1.72212
Dl -4.99364 1.76784
Dl -4.8133 1.8034
Dl -4.73456 1.82118
Dl -4.71424 1.84658
Dl -4.65074 1.9685
Dl -4.56184 2.16154
Dl -4.46278 2.39522
Dl -4.36118 2.63652
Dl -4.27482 2.8575
Dl -4.21132 3.02768
Dl -4.18846 3.11404
Dl -4.22402 3.18008
Dl -4.318 3.3274
Dl -4.4577 3.53822
Dl -4.62788 3.78714
Dl -4.79806 4.03606
Dl -4.9403 4.24942
Dl -5.0419 4.40436
Dl -5.08254 4.47294
Dl -5.06222 4.5212
Dl -4.96316 4.64058
Dl -4.7752 4.83616
Dl -4.4958 5.11302
Dl -4.45008 5.15874
Dl -4.22656 5.3721
Dl -4.0386 5.54482
Dl -3.90652 5.66166
Dl -3.85064 5.70484
$EndMODULE OSMW_LOGO
$MODULE PIN1
Po 0 0 0 15 53645919 00000000 ~~
Li PIN1
Cd module 1 pin
Kw DEV
Sc 0
AR /5362691A
Op 0 0 0
T0 0 -1.26746 0.508 0.508 0 0.127 N I 21 N "P7"
T1 0 1.778 0.635 0.635 0 0.1524 N V 21 N "A0"
$PAD
Sh "1" C 1.778 1.778 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 1 "/H0"
Po 0 0
$EndPAD
$SHAPE3D
Na "pin_array/pin_array_1x1.wrl"
Sc 1 1 1
Of 0 0 0
Ro 0 0 0
$EndSHAPE3D
$EndMODULE PIN1
$MODULE SIL-36
Po 0 0 0 15 536466A2 00000000 ~~
Li SIL-36
Cd Connecteur 36 pins
Kw CONN DEV
Sc 0
AR /53637FCD
Op 0 0 0
T0 -13.97 -2.54 1.72974 1.08712 0 0.27178 N I 21 N "P10"
T1 5.08 -2.54 1.524 1.016 0 0.254 N V 21 N "C_IN"
DS -33.02 1.524 -33.02 -1.524 0.3048 21
DS -33.02 -1.524 58.42 -1.524 0.3048 21
DS 58.42 -1.524 58.42 1.524 0.3048 21
DS 58.42 1.524 -33.02 1.524 0.3048 21
DS -30.48 -1.524 -30.48 1.524 0.3048 21
$PAD
Sh "1" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 25 "/r0"
Po -31.75 0
$EndPAD
$PAD
Sh "2" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 26 "/r1"
Po -29.21 0
$EndPAD
$PAD
Sh "3" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 31 "/r2"
Po -26.67 0
$EndPAD
$PAD
Sh "4" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 32 "/r3"
Po -24.13 0
$EndPAD
$PAD
Sh "5" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 33 "/r4"
Po -21.59 0
$EndPAD
$PAD
Sh "6" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 34 "/r5"
Po -19.05 0
$EndPAD
$PAD
Sh "7" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 35 "/r8"
Po -16.51 0
$EndPAD
$PAD
Sh "8" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 36 "/r9"
Po -13.97 0
$EndPAD
$PAD
Sh "9" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 27 "/r10"
Po -11.43 0
$EndPAD
$PAD
Sh "10" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 28 "/r11"
Po -8.89 0
$EndPAD
$PAD
Sh "11" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 29 "/r12"
Po -6.35 0
$EndPAD
$PAD
Sh "12" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 30 "/r13"
Po -3.81 0
$EndPAD
$PAD
Sh "13" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 1 "/b0"
Po -1.27 0
$EndPAD
$PAD
Sh "14" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 2 "/b1"
Po 1.27 0
$EndPAD
$PAD
Sh "15" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 7 "/b2"
Po 3.81 0
$EndPAD
$PAD
Sh "16" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 8 "/b3"
Po 6.35 0
$EndPAD
$PAD
Sh "17" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 9 "/b4"
Po 8.89 0
$EndPAD
$PAD
Sh "18" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 10 "/b5"
Po 11.43 0
$EndPAD
$PAD
Sh "19" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 11 "/b8"
Po 13.97 0
$EndPAD
$PAD
Sh "20" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 12 "/b9"
Po 16.51 0
$EndPAD
$PAD
Sh "21" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 3 "/b10"
Po 19.05 0
$EndPAD
$PAD
Sh "22" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 4 "/b11"
Po 21.59 0
$EndPAD
$PAD
Sh "23" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 5 "/b12"
Po 24.13 0
$EndPAD
$PAD
Sh "24" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 6 "/b13"
Po 26.67 0
$EndPAD
$PAD
Sh "25" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 13 "/g0"
Po 29.21 0
$EndPAD
$PAD
Sh "26" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 14 "/g1"
Po 31.75 0
$EndPAD
$PAD
Sh "27" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 19 "/g2"
Po 34.29 0
$EndPAD
$PAD
Sh "28" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 20 "/g3"
Po 36.83 0
$EndPAD
$PAD
Sh "29" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 21 "/g4"
Po 39.37 0
$EndPAD
$PAD
Sh "30" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 22 "/g5"
Po 41.91 0
$EndPAD
$PAD
Sh "31" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 23 "/g8"
Po 44.45 0
$EndPAD
$PAD
Sh "32" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 24 "/g9"
Po 46.99 0
$EndPAD
$PAD
Sh "33" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 15 "/g10"
Po 49.53 0
$EndPAD
$PAD
Sh "34" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 16 "/g11"
Po 52.07 0
$EndPAD
$PAD
Sh "35" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 17 "/g12"
Po 54.61 0
$EndPAD
$PAD
Sh "36" O 1.5748 2.286 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 18 "/g13"
Po 57.15 0
$EndPAD
$EndMODULE SIL-36
$MODULE isp
Po 0 0 0 15 53645287 00000000 ~~
Li isp
Cd Double rangee de contacts 2 x 4 pins
Kw CONN
Sc 0
AR /53626425
Op 0 0 0
T0 0 -3.81 1.016 1.016 0 0.2032 N I 21 N "P5"
T1 0 3.81 1.016 1.016 0 0.2032 N V 21 N "ISP"
DC -3.81 3.175 -3.429 3.175 0.254 21
DS 3.81 2.54 -3.81 2.54 0.2032 21
DS -3.81 -2.54 3.81 -2.54 0.2032 21
DS 3.81 -2.54 3.81 2.54 0.2032 21
DS -3.81 2.54 -3.81 -2.54 0.2032 21
$PAD
Sh "1" R 1.524 1.524 0 0 0
Dr 1.016 0 0
At STD N 00E0FFFF
Ne 3 "/MISO"
Po -2.54 1.27
$EndPAD
$PAD
Sh "2" C 1.524 1.524 0 0 0
Dr 1.016 0 0
At STD N 00E0FFFF
Ne 1 "/5v"
Po -2.54 -1.27
$EndPAD
$PAD
Sh "3" C 1.524 1.524 0 0 0
Dr 1.016 0 0
At STD N 00E0FFFF
Ne 6 "/SCK"
Po 0 1.27
$EndPAD
$PAD
Sh "4" C 1.524 1.524 0 0 0
Dr 1.016 0 0
At STD N 00E0FFFF
Ne 4 "/MOSI"
Po 0 -1.27
$EndPAD
$PAD
Sh "5" C 1.524 1.524 0 0 0
Dr 1.016 0 0
At STD N 00E0FFFF
Ne 5 "/RESET"
Po 2.54 1.27
$EndPAD
$PAD
Sh "6" C 1.524 1.524 0 0 0
Dr 1.016 0 0
At STD N 00E0FFFF
Ne 2 "/GND"
Po 2.54 -1.27
$EndPAD
$SHAPE3D
Na "pin_array/pins_array_3x2.wrl"
Sc 1 1 1
Of 0 0 0
Ro 0 0 0
$EndSHAPE3D
$EndMODULE isp
$MODULE power2
Po 0 0 0 15 5370AB3C 00000000 ~~
Li power2
Sc 0
AR /536263B6
Op 0 0 0
At VIRTUAL
T0 -1.905 -3.81 1.016 1.016 0 0.0889 N I 21 N "P1"
T1 0 -3.81 1.524 1.016 0 0.254 N V 21 N "POWER"
T2 -2.54 -1.905 1 1 0 0.15 N V 21 N "-"
T2 2.54 -1.905 1 1 0 0.15 N V 21 N "+"
DS 0 -2.54 0 2.54 0.15 21
DS 5.08 2.54 -5.08 2.54 0.254 21
DS -5.08 -2.54 5.08 -2.54 0.254 21
DS -5.08 -2.54 -5.08 2.54 0.254 21
DS 5.08 -2.54 5.08 2.54 0.254 21
$PAD
Sh "1" O 1.651 3.01498 0 0 0
Dr 1.0668 0 0
At STD N 00A8FFFF
Ne 2 "/GND"
Po -1.27 0
$EndPAD
$PAD
Sh "2" O 1.651 3.01498 0 0 0
Dr 1.0668 0 0
At STD N 00A8FFFF
Ne 1 "/5v"
Po 1.27 0
$EndPAD
$PAD
Sh "1" O 1.651 3.01498 0 0 0
Dr 1.0668 0 0
At STD N 00A8FFFF
Ne 2 "/GND"
Po -3.81 0
$EndPAD
$PAD
Sh "2" O 1.651 3.01498 0 0 0
Dr 1.0668 0 0
At STD N 00A8FFFF
Ne 1 "/5v"
Po 3.81 0
$EndPAD
$EndMODULE power2
$MODULE rgb_cathode
Po 0 0 0 15 5371655E 00000000 ~~
Li rgb_cathode
Cd RGB Cathode Socket
Sc 0
AR /53645008
Op 0 0 0
T0 5.08 -4.445 2.794 2.54 0 0.508 N V 21 N "K0"
T1 -1.27 -5.08 1.016 1.016 0 0.2032 N I 21 N "RGB_CATHODE"
T2 -5.715 2.54 1 1 0 0.15 N V 21 N "B"
T2 5.715 2.54 1 1 0 0.15 N V 21 N "G"
T2 -1.905 -4.445 1 1 0 0.15 N V 21 N "R"
DS 0.762 0 0 0.762 0.2032 21
DC 0 0 0 3.302 0.2032 21
DS 1.778 -0.508 0.508 -1.778 0.2032 21
DS 0.508 -1.778 0.762 -2.032 0.2032 21
DS 0.762 -2.032 0 -2.794 0.2032 21
DS 0 -2.794 -1.27 -1.524 0.2032 21
DS -1.27 -1.524 -0.508 -0.762 0.2032 21
DS -0.508 -0.762 -0.254 -1.016 0.2032 21
DS -0.254 -1.016 0 -0.762 0.2032 21
DS 0 -0.762 -0.762 0 0.2032 21
DS -0.762 0 -1.016 -0.254 0.2032 21
DS -1.016 -0.254 -0.762 -0.508 0.2032 21
DS -0.762 -0.508 -1.524 -1.27 0.2032 21
DS -1.524 -1.27 -2.794 0 0.2032 21
DS -2.794 0 -1.778 1.016 0.2032 21
DS -1.778 1.016 -1.524 0.762 0.2032 21
DS -1.524 0.762 -0.508 1.778 0.2032 21
DS -0.508 1.778 -0.762 2.032 0.2032 21
DS -0.762 2.032 0 2.794 0.2032 21
DS 0 2.794 1.27 1.524 0.2032 21
DS 1.27 1.524 0.508 0.762 0.2032 21
DS 0.508 0.762 0.254 1.016 0.2032 21
DS 0.254 1.016 0 0.762 0.2032 21
DS 0.762 0 1.016 0.254 0.2032 21
DS 1.016 0.254 0.762 0.508 0.2032 21
DS 0.762 0.508 1.524 1.27 0.2032 21
DS 1.524 1.27 2.794 0 0.2032 21
DS 2.794 0 2.032 -0.762 0.2032 21
DS 2.032 -0.762 1.778 -0.508 0.2032 21
$PAD
Sh "R" C 1.778 1.778 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 3 "/r0"
Po 0 -4.445
$EndPAD
$PAD
Sh "B" C 1.778 1.778 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 1 "/b0"
Po -3.81 2.54
$EndPAD
$PAD
Sh "G" C 1.778 1.778 0 0 0
Dr 0.8128 0 0
At STD N 00E0FFFF
Ne 2 "/g0"
Po 3.81 2.54
$EndPAD
$EndMODULE rgb_cathode
$MODULE to252
Po 0 0 0 15 5362EC1B 00000000 ~~
Li to252
Cd D-Pak (TO-252)
Sc 0
AR /53626CE9
Op 0 0 0
T0 0 -5.08 1.524 1.524 0 0.3048 N V 21 N "Q1"
T1 0 9.525 1.524 1.524 0 0.3048 N I 21 N "AP4435GH"
DS 1.27 4.445 1.27 8.255 0.15 21
DS 1.27 8.255 3.175 8.255 0.15 21
DS 3.175 8.255 3.175 4.445 0.15 21
DS -3.175 4.445 -3.175 8.255 0.15 21
DS -3.175 8.255 -1.27 8.255 0.15 21
DS -1.27 8.255 -1.27 4.445 0.15 21
DS -3.81 -3.81 3.81 -3.81 0.15 21
DS 3.81 -3.81 3.81 4.445 0.15 21
DS 3.81 4.445 -3.81 4.445 0.15 21
DS -3.81 4.445 -3.81 -3.81 0.15 21
$PAD
Sh "G" R 1.5 2.5 0 0 0
Dr 0 0 0
At SMD N 00888000
Ne 2 "/A0"
Po -2.3 6.9
$EndPAD
$PAD
Sh "S" R 1.5 2.5 0 0 0
Dr 0 0 0
At SMD N 00888000
Ne 1 "/5v"
Po 2.3 6.9
$EndPAD
$PAD
Sh "D" R 7 7 0 0 0
Dr 0 0 0
At SMD N 00888000
Ne 3 "/H0"
Po 0 0
.ZoneConnection 2
$EndPAD
$SHAPE3D
Na "smd/smd_transistors/sot428.wrl"
Sc 1 1 1
Of 0 0 0
Ro 0 0 0
$EndSHAPE3D
$EndMODULE to252
$EndLIBRARY
