## LOW POWER-oriented LOW-COST MPPT controller based on PIC12F675 MCU with minimal external components.

### Apparently for the first time in my life I'm relasing a project prematurely, as I believe I'm forced to do so due to latest events in my country, in hope that this project is mature enough and can already be used in some areas to help my people.

#### Detailed description and documentation is in top of "pic12f675/main.c" file. Ukrainian version of description will be available on Ukrainian internet resources.

#### I've been working on this project intermittently during Autumn 2022 and it took longer than I wanted as I have extremely limited time/resources.

#### "LOW POWER" note: designed specifically for use in cloudy and winter weather to work properly with generated power starting from around 0.1W, but depending on used components can be used for high power as well. The prototype was designed/assembled for power range 0.1W - 100W. Even higher power can be achieved with strong inductor etc.


### Repository structure:
- pic12f675/main.c - MCU source code with description and documentation;
- pic12f675/Basic_Scheme.png - schematic of solution with some description (drawn by hand, electricity may drop-off any time anyway).
- pic12f675/compiled/... - firmware images (current version - 0.3.THR.OPTION).
- pic12f675/compiled/pic12f675_lp_mppt__0_3_198_A.hex - latest tested version.

### Versioning important note:
Version format is major.minor.THR.OPTION where:
- THR is a threshold constant, which should be chosen according to used input diode for optimal work:
    * at the moment compiled THR versions are 202, 198, 194; 202 is assumed for "no input diode" connection, 198 - for low drop diodes, 194 - for worse diodes.
    * THR choice affects efficiency and output pulsations and it's preferable to use lower THR (leave a little room until MPP) - see "PROJ_CONST_THR" notes and "Input diode Vf" notes in "main.c".
- OPTION is used pinout/topology, see "FW_VER_OPTION vaiants" in "main.c".
- version data is built into FW at the end addresses (before OSCCAL) so it can be easily checked, those bytes/commands aren't used by firmware.


### Releases
#### Release 0.3.x.A
- added input diode Vf consideration (since now several .THR options are built for optimal choice) - significantly reduces pulsations, improves efficiency (avoids solar power generation loss in MPP overshoot).
- excluded debug/configure code part - reduces pulsations;
- used constant multiplication (hardcoded autogen algorithm) - reduces pulsations;

#### Release 0.1.0.A - first version
