1. pick pic12f675/compiled/pic12f675_lp_mppt__1_0_198_F.hex firmware file for programming

2. check HOTWO_PROGRAM_PIC12F675_USING_ARDUINO_IN_5_MINUTES.md to program pic

3. check pic12f675/1.0_schematics.jpg and pic12f675/lp_mppt_pic12f675__prototype2_* images
assuming 18v-rated solar panel use nominals referred in 1.0_schematics.jpg

3.1. solar panel voltage rating can be specified inconsistently sometimes, refered rating is maximum power rating, which is also 80-81% of OCV rating (So 18v-rated panel has around 22v OCV);
    There are often some 'system voltage ratings' which is 12v for 18v-rated solar (~12v battery), also often present 24v systems (24v batteries with 36v-rated solars).
3.2. if it's needed to assemble device for 36v-solar - you'll need either a specific linear voltage regulator that tolerates input up to ~45v or make a cascade voltage regulator
    with first one of high input voltage rating and output of 12-24v, after which connect a regulator with 3.3-5v output, small capacitance is required on the input of first one,
    it's necessary to have OUTPUT of first regulator to be not much than 50% OCV rating of solar panel.
    Voltage divider on ADC line for such case requires adjustment so that maximum possible voltage never exceeds PIC's power supply voltage.
    * Cascade regulators weren't tested but these recommendations should work. Regulator used in prototypes tolarates input up to 30v.

4. for output currents more than 2-3A it's recommended to add some radiator to mosfet connected to PWM pin; for short sparks (some 5 seconds) prototype2 was tested with 14A output (different inductor was used) without radiators, but that heats up mosfet very quickly.

5. mind your inductor current rating - it's important that your output current doesn't exceed this (more or less reliable limit) - device will still work with currents more than that rating, but will lose efficiency and stress other components (capacitors and mosfets).

6. for 18v solar and ~12v output with 33uH inductor there's almost never present any coil whine, lower nominal inductor OR bigger difference between solar panel voltage and desired output voltage increase cases when coil whine is present. Coil whine is present when output is limited (LIM pin is being set low) and it's hard to deal with so it will stay as is.
