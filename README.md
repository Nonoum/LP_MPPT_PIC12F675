## LOW POWER-oriented LOW-COST MPPT controller based on PIC12F675 MCU with minimal external components.

### Apparently for the first time in my life I'm relasing a project prematurely, as I believe I'm forced to do so due to latest events in my country, in hope that this project is mature enough and can already be used in some areas to help my people.

#### Detailed description and documentation is in top of "pic12f675/main.c" file. Ukrainian version of description will be available on Ukrainian internet resources.

#### I've been working on this project intermittently during Autumn 2022 and it took longer than I wanted as I have extremely limited time/resources.

#### "LOW POWER" note: designed specifically for use in cloudy and winter weather to work properly with generated power starting from around 0.1W, but depending on used components can be used for high power as well. The prototype was designed/assembled for power range 0.1W - 100W. Even higher power can be achieved with strong inductor etc.


### Repository structure:
- pic12f675/main.c - MCU source code with description and documentation;
- pic12f675/Basic_Scheme.png - schematic of solution with some description (drawn by hand, electricity may drop-off any time anyway).
- pic12f675/compiled/pic12f675_lp_mppt__0_1_0_A.hex - firmware compiled with configuration 'A' (current version - 0.1.0), see "FW_VER_OPTION vaiants" in "main.c".

