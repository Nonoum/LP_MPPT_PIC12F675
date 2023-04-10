## LOW POWER-capable wide power range LOW-COST Solar MPPT controller based on PIC12F675 MCU with minimal external components.

### Apparently for the first time in my life I'm relasing a project prematurely (version 0.1.0.A), as I believe I'm forced to do so due to latest events in my country, in hope that this project is mature enough and can already be used in some areas to help my people.

#### Detailed description and documentation is in top of "pic12f675/main.c" file. Ukrainian version of description will be available on Ukrainian internet resources.

#### I've been working on this project intermittently since Autumn 2022 and it took longer than I wanted as I have extremely limited time/resources.

#### "LOW POWER" note: designed specifically for use in cloudy and winter weather to work properly with generated power starting from around 0.1W, but depending on used components can be used for high power as well, which won't impede proper work of very same device with low power, due to device concept and low frequency MCU. Most reasonable use-case is a small solar station / medium-size panel (e.g. for hiking) with solar panel(s) rating ranging from some 10W to 100W.


### Repository structure:
- pic12f675/main.c - MCU source code with description and documentation;
- pic12f675/Basic_Scheme.png - schematic of solution with some description (drawn by hand, electricity may drop-off any time anyway).
- pic12f675/compiled/... - firmware images (current version - 1.0.THR.OPTION).
- pic12f675/compiled/pic12f675_lp_mppt__1_0_198_B.hex - latest tested version.
- pic12f675/compiled/pic12f675_lp_mppt__1_0_198_F.hex - latest tested version.

### Versioning important note:
Version format is major.minor.THR.OPTION where:
- THR is a threshold constant, which should be chosen according to used input diode for optimal work:
    * compiled THR versions are 202, 198, 194; 202 is assumed for "no input diode" connection, 198 - for low drop diodes, 194 - for worse diodes.
    * THR choice affects efficiency and output pulsations and it's preferable to use lower THR (leave a little room until MPP) - see "PROJ_CONST_THR" notes and "Input diode Vf" notes in "main.c".
- OPTION is used pinout/topology, see "FW_VER_OPTION vaiants" in "main.c".
- version data is built into FW at the end addresses (before OSCCAL) so it can be easily checked, those bytes/commands aren't used by firmware.


### Releases
- TBD: youtube video with more details;


#### [10 april 2023] - added schematic samples, assembly example and more info
- added TLDR_DIY.md
- added HOTWO_PROGRAM_PIC12F675_USING_ARDUINO_IN_5_MINUTES.md
- added pic12f675/1.0_schematics.jpg - extended schematics info
- added pic12f675/lp_mppt_pic12f675__prototype2_top_view.png
- added pic12f675/lp_mppt_pic12f675__prototype2_bottom_view_flipped.png
- added pic12f675/lp_mppt_pic12f675__prototype2_bottom_view_flipped_commented.png

#### Release 1.0.x.B, 1.0.x.F [05 april 2023] - last planned code release, only some auxiliary info updates are planned now.
- reworked mid and high PWM modes (now 72 modes in total);
- improved limiting logic and accuraccy;
- increased range of continuous mode (more cases/more possibility of falling into continuous mode);
- reduced ripple current;
- reduced coil whine is some cases;
- eased output capacitor requirements;
- optimized OCV measuring algorithm and used EEPROM for faster multiplication (smaller no-output-feeding gaps during OCV measurings : minimal possible gap is 52us);
- updated documentation (in main.c);
- ran wide testing using different equipment, efficiency results appeared slightly smaller (measuring errors +/-) but up to 95% conversion efficiency.

#### Release 0.10.x.B, 0.10.x.F [18 feb 2023] - two game-changing updates in one, and some fixes (don't miss '47uf capacitor' notes)
- revised and drastically reworked PWM modes, now 90 modes instead of old 26 modes;
- supported inductor's continuous mode (in part of cases / some power ranges it works in continuous mode);
- improved efficiency due to reworked PWM modes (mostly for higher-power ranges);
- significantly reduced ripple current;
- significantly improved pulsations (especially when in continuous mode, middle range is now well balanced too);
- significantly eased output capacitor and inductor requirements;
- significantly reduced coil whine (is some cases fully removed);
- improved output limiting accuraccy;
- revised/fixed adc mode/timings, revised and profiled OCV (Open Circuit Voltage) measurings;
- it's necessary to add an extra 47uf (any, e.g. electrolythic) capacitor (along with existing ceramic 1-2uf cap) to output of linear voltage regulator to make it work correctly*
    - *this fixes OCV measuring on higher power ranges as otherwise the used regulator (LP2950CZ) overshoots output voltage when input voltage rapidly increases, which significantly distorts ADC readings for OCV evaluation and has very long settling time (when only recommended small ceramic cap is used);
    - *adding this capacitor BEFORE updating the FW to 0.10 is NOT recommended due to [see 'startup voltage' note below];
    - *exact value of referred capacitor might be reconsidered but 47uf overall looks okay in current testing for both assembled prototypes.
- removed a feature of measuring startup voltage (which was used as minimum allowed MPP voltage) due to too many related problems:
    - apart of secondary problem PIC starts up with voltage around 2v, while ADC requires higher voltage to provide correct results, which makes it too complicated to attempt to measure minimal stable voltage;
    - so minimum voltage is now hardcoded to be 21.8% of measured range (actual voltage range depends on PIC input voltage and voltage divider on ADC line);
    - actual THR voltages will be from around 50% (to 80%) of divider range (solar panels would output somewhat power starting from around 50% of their nominal OCV);
    - *with older FW ADC provides completely invalid starting measured level if larger capacitor is added to PIC power supply.
- updated internal documentation/notes in main.c;
- tested conversion efficiency reached 97% @ 11.7v 6w output;
- still present issue: when input power is more than output load can consume (cut by limiting logic) - there's coil whine and some extra power loss (e.g. input is capable of providing 22watts, output can consume 19 watts, actual output is 18.5 watts), planned for further improvement of limiting logic;
- now when output can be (sometimes) as flat as a battery (not just voltage, but even current in 3 digits) it's very clear how much PCB layout/design matters as second prototype with smarter assembly works much more smoothly in same conditions, so at some point PCB layout example/recommendations will be provided, minimal recommendation would be to keep power circuit (fets+inductor+diode+output capacitor) as dense as possible and solder to capacitors as close as possible.

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
