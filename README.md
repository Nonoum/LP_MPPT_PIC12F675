## LOW POWER-oriented LOW-COST MPPT controller based on PIC12F675 MCU with minimal external components.

### Apparently for the first time in my life I'm relasing a project prematurely (version 0.1.0.A), as I believe I'm forced to do so due to latest events in my country, in hope that this project is mature enough and can already be used in some areas to help my people.

#### Detailed description and documentation is in top of "pic12f675/main.c" file. Ukrainian version of description will be available on Ukrainian internet resources.

#### I've been working on this project intermittently during Autumn 2022 and it took longer than I wanted as I have extremely limited time/resources.

#### "LOW POWER" note: designed specifically for use in cloudy and winter weather to work properly with generated power starting from around 0.1W, but depending on used components can be used for high power as well. The prototype was designed/assembled for power range 0.1W - 100W. Even higher power can be achieved with strong inductor etc.


### Repository structure:
- pic12f675/main.c - MCU source code with description and documentation;
- pic12f675/Basic_Scheme.png - schematic of solution with some description (drawn by hand, electricity may drop-off any time anyway).
- pic12f675/compiled/... - firmware images (current version - 0.8.THR.OPTION).
- pic12f675/compiled/pic12f675_lp_mppt__0_8_198_B.hex - latest tested version.
- pic12f675/compiled/pic12f675_lp_mppt__0_8_198_F.hex - latest tested version.

### Versioning important note:
Version format is major.minor.THR.OPTION where:
- THR is a threshold constant, which should be chosen according to used input diode for optimal work:
    * at the moment compiled THR versions are 202, 198, 194; 202 is assumed for "no input diode" connection, 198 - for low drop diodes, 194 - for worse diodes.
    * THR choice affects efficiency and output pulsations and it's preferable to use lower THR (leave a little room until MPP) - see "PROJ_CONST_THR" notes and "Input diode Vf" notes in "main.c".
- OPTION is used pinout/topology, see "FW_VER_OPTION vaiants" in "main.c".
- version data is built into FW at the end addresses (before OSCCAL) so it can be easily checked, those bytes/commands aren't used by firmware.


### Releases
In development: big update for PWM modes.

#### Release 0.8.x.B, 0.8.x.F [22 jan 2023] - significantly reworked cycle and balancing, introduced one additional topology
- added topology 'F' (used in second assembled prototype) with two PWM-driving pins (both not inverted, 100% synced), more PCB-frindly pinout;
- reworked balancing logic - will work well with PWM updates, still looks better as is (middle part of PWM modes is especially poorly balanced yet);
- new balancer assumes no significant noise on ADC line (not likely to ever be an issue in any circumstances);
- improved limiting logic/accuracy (still being improved, currently there are significant output spikes at higher power range);
- significantly reduced cycle delays (less response time - faster balancing);
- MPP voltage threshold is slightly lowered due to changed logic (by 1/256), might be compensated internally in future;
- *dual PWM pin in topology 'F' is rather for better temperature stability on chip for stronger mosfets, even strong (6 amps rated) external Low-Side mosfet driver TC4420CPA has made no impact on efficiency whatsoever, so no insufficient current for driving fets was observed in tested conditions (IRLZ34N fet, IRL2203 fet);
- *overall measured efficiency looks good and matches appropriate charts of buck converters with such topology (with diode), on 18v panel with output at 5v efficiency was from 78% (below 1 watt output) raising to 86% (at around 2 watts output) which is an expected characteristic (comparing to data of XL4015 in it's datasheet), with output at 12v characteristic is similar but higher and reached around 92.5% efficiency at 2 watts output (core components of used prototype here are ILR2203, IRLZ34N (pwm), SR840 diode, CDRH127/LDNP-330MC - 33uH inductor).

#### Release 0.6.x.B [07 jan 2023] - extended functionality - upgraded topology letter (A -> B)
- added output limiting pin (GPIO4; pull to gnd when outputting power needs to be suspended; has internal pullup resistor);
- added inverted PWM output (GPIO5; strict phase matching with non-inverted PWM) - might be used in some cases (e.g. when some output mosfet driver is involved);
- compatible with 'A' versions if GPIO4 and GPIO5 weren't used (as it was recommended) - see 'FW_VER_OPTION vaiants' updated;
- this release has been more or less tested in various scenarious involving initial and second assembled prototype boards;
- plenty of testing has been done during last ~1 month, related info/assumptions are to be updated sometime later ('Releases' section will get a new tag as well).

#### Release 0.3.x.A [07 dec 2022]
- added input diode Vf consideration (since now several .THR options are built for optimal choice) - significantly reduces pulsations, improves efficiency (avoids solar power generation loss in MPP overshoot);
- excluded debug/configure code part - reduces pulsations;
- used constant multiplication (hardcoded autogen algorithm) - reduces pulsations.

#### Release 0.1.0.A [27 nov 2022] - first version
