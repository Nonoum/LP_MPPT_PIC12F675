/*
    A simple MPPT controller with non-canonical design, developed By Evgeniy Evstratov, Autumn 2022 and further.

    This Solar controller is being developed in these dark times when russia is destroying energetic system of my country and has already caused full-scale blackout.

    The project is intended as LOW-COST simple MPPT device that works well in low-power range and:
    - is affordable (very cheap MCU, small and simple scheme, *NO CURRENT SENSOR);
    - gets maximum solar power generation in cloudy and winter weather (proper work specifically in low power range: starting from 0.01 A).
        -- allows using high voltage panels for low-voltage devices in such conditions when very small amount of solar energy can be generated (e.g. using 18v panel to charge a smartphone powerbank).

    Global disclaimer: author has very limited testing capabilities, any hardware implementaion should be tested in required scenarios to ensure proper/correct work.

    Principle of work: according to solar panels voltage/current curves at very most part of the range the maximum power point (MPPT) is at around 80% of open circuit voltage (named 'raw voltage' here).
    So with certain scheme it's possible to measure (almost)open-circuit voltage (with minimal load) and track optimal mode without need of a current sensor.

    Basic scheme description in words:
        1. solar panel should be connected via diode (shottky SR840 is used in the prototype).
        2. input("high voltage" and "gnd") is connected to linear voltage stabilizer and minimal(!) required ceramic capacitor (0.1 - 0.15uf):
            0.15uf cap and "LP2950CZ-3.3G ON" stabilizer are used in the prototype.
        3. linear stabilizer has output ceramic capacitor (used 2.2uf in the prototype) and is connected as the PIC MCU power supply;
            * no mini buck converter can be used instead due to it's high input capacitance.
        4. one N-channel mosfet connects a large buffer capacitor between "high voltage" and "gnd" and occassionally (~once a second) disconnects it to measure open circuit voltage;
            * required PULLUP EXTERNAL resistor (used 200kOhm) - pull to stabilizer's voltage (which is 3.3v or 5v);
            heavy 5x 1000uf 35v LOW ESR capacitors are used in the prototype.
        5. second N-channel mosfet connects an output circuit (inductor+diode+capacitor) between "high voltage" and "gnd" and is being PWMed;
            * required PULLDOWN EXTERNAL resistor (used 200kOhm) - pull to "gnd";
            inductor is around 22uh - 47uh (in currently tested prototype 47uh is used);
            * different inductors can be used depending on other conditions - calculate for your needs;
            recommended output capacitor is at least 2000uf (in currently tested prototype 2000uf is used).
        6. a calculated voltage divider between "high voltage" and "gnd" is connected to PIC MCU's pin with ADC channel:
            so that maximum possible voltage is <= nominal voltage of linear stabilizer;
            3.3kOhm : 20kOhm devider is used in prototype, I'd recommend to not exceed (not significantly) sum resistance around 1kOhm per 1v nominal of solar panel.
        7. mosfets used in prototype are "IRLZ34N" - pretty low input capacitance, proper DS opening at 3.3v.
        8. with this minimum schematic the output is not regulated so it can reach up to "open circuit voltage" of the solar panel connected,
            as a very simple voltage limiter - a buck(step-down) converter can be used.

NOTES at the moment of first release (0.1.0.A) [to be updated later]:
    Output voltage/feedback: it's planned to implement feedback for output voltage limiting with an optocoupler and a comparator;

    Features:
        - "soft start" in current and all foreseeable future versions;
        - low self-consumption: prototype consumes 1..2 ma (stable state with no load), having worse linear stabilizer it would be a little more;

    PIC MCU notes: there are different packages with at least two working voltage ranges: some PICs have 3.6v limit (only 3.3v stabilizer can be used in this case);

    Efficiency notes:
        - prototype has shown efficieny (output RMS power / input RMS power) from around 70 to 92% depending on voltage delta between an MPPT point and target output voltage;
        - efficieny was measured with not expensive equipment in limited environment;
        - in described testing 18v panel was used (with typical MPPT ranging within 15 ... 18v);
        - with plain powerbank as output ("consumed" voltage 4.5 ... 5.2v) deltaV is more than 10 volts and measured efficiency was around 75%;
        - with output 11.4v (same powerbank but luckily triggered by delay to "stabilize at 12V QC mode" and apparently dropping it's buck cycles to maintain that)
            deltaV was 3.1V and measured efficiency was 92.5%.
        - presumably higher voltage linear stabilizer (5v only, more = not allowed for PIC) would yield higher efficiency (or at least with some other mosfets) - this is to be tested;
        - Miller Effect: wasn't observed in prototype with my testing in artificial environment (tested single output driving pin vs dual output driving pin at 3.3v MCU voltage)
            * it was tested with output voltages from 2v to 4v and 12v;
            * 'noticed' in this case means any significant impact on efficiency (around 1% improvement was noticed though it can be due to measuring inaccuracy);
            * mosfets with larger parasitic capacitance could defenitely require doubled/tripled MCU output pins to drive them at high frequencies;
            * PIC16F676 could be (relatively)easily adapted to use entire portB (6 pins) for mosfet driving;
            * mosfet that connects capacitors is rarely triggered so it can (and should) be more powerful (lower RdsON) - planned to test IRL2203 for this;

    Pulsation notes:
        - output voltage pulsations looked good for 0.1 FW version (and much better since 0.3) although they were evaluated with simple testers (I don't have an oscilloscope);
        - even higher pulsations are *presumably okay for charging batteries with good BMS;
        - increasing capacitance (both buffer and output caps) should smooth pulsations very well (partly tested);
        - capacitance can be smaller or bigger than in prototype described above, suggestions for buffer and output caps are:
            -- minimum: 1000uf (buffer), 2000uf(output);
            -- recommended: 3000uf+ (buffer), 5000uf+(output);
        - for future addition of output voltage limitation feedback there are expected to be rare more substantial drop-offs (when reaching voltage limit) for the implementation that I have in mind;
        - pulsations can be induced by MPP overshoot, which can significantly reduce input current and thus generated power - see "PROJ_CONST_THR" notes and "Input diode Vf" notes;
        - pulsations can be induced by partial shading (when shading occurs somewhere inseries);

    PWM notes: code does manual pwm with duty (open output mosfet) lengths starting from 2us and duty/nonduty ratio up to 1:1 (less or much-less duty depending on various circumstances).

    More output voltage limiting notes:
        - powerbank:
            - tested powerbank can be tricked to trigger QC mode and rise voltage - when it's connected it "requests" higher voltage while consuming no substantial load
            and the actual voltage can be rising at the moment so powerbank considers that everything is according to plan and accepts it;
            - if voltage manages to rise higher than powerbank's limit - it will shut down and won't charge;
            - plain buck converter (when added after output) is apparenty fully "opened" and acts as a minor resistance (inductor+mosfet) with it's input and output voltages being almost equal;

    Adding to existing solar (battery + PWM charger) system:
        - it's assumed that this controller can be simply connected in series between solar panel and thirdparty "PWM Solar controller" to increase system efficiency,
        although it's not tested and I'm not 100% that a PWM controller would handle this without any issues
        (specifically when charging is done and PWM controller's input voltage goes up);

    Using as-is (non-regulated output) with a BMS and batteries: *presumably will work fine, might trigger BMS's overvoltage protection in certain cases (depends on BMS).

    Partial shading: use cases of this project don't assume optimal work with significant partial shading (no "full scan" is implemented, nor it's possible with such scheme).
    Personal notes:
        *- although I believe it will work pretty well with real-world partial shading cases;
        *- partial shading isn't a problem that needs solving (except for very specific forced cases) as solar panels as technology simply doesn't assume partial shading - so just do proper installation / consider proper sizes to avoid any partial shading;
*/

#define _XTAL_FREQ 4000000

#include <xc.h>

//BEGIN CONFIG
#pragma config FOSC = INTRCIO
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF // MCLR internal, use pin as IO
#pragma config BOREN = ON // Brown-out Reset Enable bit (BOR enabled)
#pragma config CP = OFF // Flash Program Memory Code Protection bit (Code protection off)
#pragma config CPD = OFF // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
//END CONFIG


/*
    PROJ_CONST_THR notes:
    - for now using a constant hard-hardcoded threshold value (with hardcoded ifdef-generated multiplication function);
    - planning to compile several versions with different offsets for the value to compensate input diode Vf;

    Input diode Vf (voltage drop on the input shottky diode):
    - due to scheme specifics measurings of voltage is done after diode(s)*
        *could be done before diode if only one solar panel is used (one parallel connection);
        *if several panels are connected in parallel - each 'positive' wire should be connected through an individual diode (or each negative, but not mixed);
    - when measuring raw (no load/OCV) voltage - the diode current is minimum, diode Vf is minimum and can be negligible;
    - when measuring voltage during load - Vf is larger and depends on current, specific diode, diode temperature,
        so to expect a correct(more or less) percentage of raw(no load/OCV) voltage on the panel(s) - some offset of diode Vf should be considered,
        so: several versions of FW with different hardcoded THR would be available to accomodate target case the best way;
    - current dependency: for big current Vf would be bigger and appropriate offset (for maximum expected current) should be used to maximize output power (+minimize current fluctuations);
    - other offset considerations: it's better to offset THR a little lower (linear loss comparing to MPP) than higher (rather exponencial loss according to solar panels U/I characteristics);
*/
#define PROJ_CONST_THR 198 // hard constant threshold percentage (x/255) of raw(no load/OCV) voltage to hold

// version info (3 numbers and letter) is accessible at top program-memory addresses right before OSCCAL (retlw commands)
#define FW_VER_MAJOR 0
#define FW_VER_MINOR 9
#define FW_VER_THR PROJ_CONST_THR
#define FW_VER_OPTION 'F' // some letter describing topology/configuration of compiled code. default is 'F'
/*
    FW_VER_OPTION vaiants:
    - 'A' (0x41) : unregulated output;
        * this is deprecated topology - replaced with compatible 'B', which supports extra features.
        GPIO0 - PWM (output-fet-driving pin);
        GPIO1 - controls mosfet of buffer capacior;
        GPIO2 is ADC input;
        // next are debug pins which were recommended to stay disconnected:
        GPIO4 - input mode debug pin, GPIO5 - output mode debug pin.
        GPIO3 - unused floating input;
    - 'B' (0x42) : optionally regulated output, also has inverted PWM pin; 'B' is extension of 'A' in case debug pins weren't used.
        GPIO0 - PWM (output-fet-driving pin);
        GPIO1 - controls mosfet of buffer capacior;
        GPIO2 is ADC input;
        GPIO4 - inverted 'limit' input, has internal pullup. pull to gnd when outputting power needs to be suspended;
        GPIO5 - inverted PWM pin (strict phase matching with non-inverted PWM). can be used to strengthen closing output fet with small extra fet.
        GPIO3 - unused floating input;
    - 'F' (0x46) : optionally regulated output; pinout is incompatible with previous versions;
        * no inverted PWM output is supported for this topology.
        GPIO0 - controls mosfet of buffer capacior;
        GPIO1 - PWM (output-fet-driving pin);
        GPIO2 - PWM (output-fet-driving pin) - any of GPIO1 or GPIO2 or both can be used, assumed to use both for more pin rated current;
        GPIO4 is ADC input;
        GPIO5 - inverted 'limit' input, has internal pullup. pull to gnd when outputting power needs to be suspended;
        GPIO3 - unused floating input;
*/





#define AUX_STRINGIFY_INTERNAL(foo) #foo
#define AUX_STRINGIFY(foo) AUX_STRINGIFY_INTERNAL(foo)

// manual params
#define PROJ_TMR_PRESCALE (0b111) // x256

// pins / topology
#if FW_VER_OPTION == 'B'
#define PROJ_BIT_NUM_OUT_PWM_0 0 // GPIO0 : N-channel mosfet (or 'enable' pin on output line) controlling output line, PULLDOWN external resistor
#define PROJ_BIT_NUM_OUT_PWM_1 PROJ_BIT_NUM_OUT_PWM_0
#define PROJ_BIT_NUM_OUT_CAP 1 // GPIO1 : N-channel mosfet controlling power capacitor, PULLUP external resistor 200k
#define PROJ_BIT_NUM_IN_INV_LIMIT 4 // GPIO4 - inverted 'limit' input
#define PROJ_BIT_NUM_OUT_INV_PWM 5 // GPIO5 - inverted PWM pin, strict phase match
#define PROJ_ADC_CH_VOL 2 // ADC channel - AN2 (GPIO2) : voltage divider for power line - calculate so that it maxes out to chip PSU level (used 3.3 regulator)
#elif FW_VER_OPTION == 'F'
#define PROJ_BIT_NUM_OUT_PWM_0 1 // GPIO1 : N-channel mosfet (or 'enable' pin on output line) controlling output line, PULLDOWN external resistor
#define PROJ_BIT_NUM_OUT_PWM_1 2 // GPIO2 : N-channel mosfet (or 'enable' pin on output line) controlling output line, PULLDOWN external resistor
#define PROJ_BIT_NUM_OUT_CAP 0 // GPIO0 : N-channel mosfet controlling power capacitor, PULLUP external resistor 200k
#define PROJ_BIT_NUM_IN_INV_LIMIT 5 // GPIO5 - inverted 'limit' input
#define PROJ_ADC_CH_VOL 3 // ADC channel - AN3 (GPIO4) : voltage divider for power line - calculate so that it maxes out to chip PSU level (used 3.3 regulator)
#else
// TODO different configurations
#endif

#define PROJ_ADC_ADCS 0b010 // conversion time; 32 Tosc (8 tacts)


GPIObits_t gp_shadow;

uint8_t is_prev_cycle_pwr; // [0:1], if previous cycle applied pwr (limit wasn't reached in the beginning)

uint8_t app_flags;
#define PROJ_APP_FLAG_CYCLE_RETURN_TRAP 0

#define PROJ_TIMER_MULTIPLIER_AS_BIT 1 // multiplier is a degree of 2
uint8_t timer_multiplier = (1 << PROJ_TIMER_MULTIPLIER_AS_BIT);


#define LOCATION__PIR1bits_TMR1IF "12,0"
#define LOCATION__ADCVAL "30" // ADRESH reg


#define PROJ_OUT_FET_PINS_POS_MASK ((1 << PROJ_BIT_NUM_OUT_PWM_0) | (1 << PROJ_BIT_NUM_OUT_PWM_1))
#ifdef PROJ_BIT_NUM_OUT_INV_PWM
    #define PROJ_OUT_FET_PINS_INV_MASK (1 << PROJ_BIT_NUM_OUT_INV_PWM)
#else
    #define PROJ_OUT_FET_PINS_INV_MASK 0
#endif
#define PROJ_OUT_FET_PINS_ALL_MASK (PROJ_OUT_FET_PINS_POS_MASK | PROJ_OUT_FET_PINS_INV_MASK)

#define PROJ_STATE_GPIO_CAPON_PWMON ((1 << PROJ_BIT_NUM_OUT_CAP) | PROJ_OUT_FET_PINS_POS_MASK)
#define PROJ_STATE_GPIO_CAPON_PWMOFF ((1 << PROJ_BIT_NUM_OUT_CAP) | PROJ_OUT_FET_PINS_INV_MASK)
#define PROJ_STATE_GPIO_CAPOFF_PWMOFF (PROJ_OUT_FET_PINS_INV_MASK)


/*
    OSSCAL deviation: on my PIC it was x1.65 difference between min and max frequency (osccal 0x00-min freq and 0xFC-max freq)
    with typical calibrated values checked on 5 pics being around 0x24 - 0x44, so setting osccal to max value is assumed
    to multiply actual frequency by around x1.5 (comparing to 4mhz spec).
*/

// relies on active bank 0
#define PROJ_SET_GPIO(a_value) \
    asm("movlw " AUX_STRINGIFY(a_value)); \
    asm("movwf 5"); /* assign GPIO */ \
    asm("movwf _gp_shadow"); /* refresh gp_shadow */
#define PROJ_SET_GPIO_CAPON_PWMON PROJ_SET_GPIO(PROJ_STATE_GPIO_CAPON_PWMON)
#define PROJ_SET_GPIO_CAPON_PWMOFF PROJ_SET_GPIO(PROJ_STATE_GPIO_CAPON_PWMOFF)
#define PROJ_SET_GPIO_CAPOFF_PWMOFF PROJ_SET_GPIO(PROJ_STATE_GPIO_CAPOFF_PWMOFF)



#define PROJ_SKIP_IF_LIMIT_REACHED asm("btfsc 5," AUX_STRINGIFY(PROJ_BIT_NUM_IN_INV_LIMIT));
#define PROJ_SKIP_IF_LIMIT_NOTREACHED asm("btfss 5," AUX_STRINGIFY(PROJ_BIT_NUM_IN_INV_LIMIT));


// sets up adc on PROJ_ADC_CH_VOL; relies on bank 0 being selected
#define PROJ_ADC_SETUP__WHEN_BANK0() \
    asm("movlw " AUX_STRINGIFY((1 | (PROJ_ADC_CH_VOL << 2)))); \
    asm("movwf 31"); // ADCON0 = w

// triggers ADC start; relies on bank 0 being selected
#define PROJ_ADC_START() \
    asm("bsf 31,1"); // ADCON0bits.GO_DONE = 1;

// awaits ADC completion, result is in ADRESH (30); relies on bank 0 being selected
#define PROJ_ADC_WAIT(lbl_name) \
    asm(""lbl_name":"); \
    asm("btfsc 31,1"); /* while(ADCON0bits.GO_DONE) */ \
    asm("goto "lbl_name);

// erases tmp_ctr and tmp_ctr2 vars
void fast_RunADC_8_rising() {
    asm("movlw 5");
    asm("movwf _tmp_ctr"); // tmp_ctr = 5
    asm("clrf _tmp_ctr2"); // last result
    asm("labe_fast_RunADC_8_rising__rising_cycle:");
    PROJ_ADC_START()
    PROJ_ADC_WAIT("labe_fast_RunADC_8_rising__wait")
    // got result
    asm("movf " LOCATION__ADCVAL ",w");
    asm("addlw 254"); // w = adc - 2
    asm("subwf _tmp_ctr2,w"); // w = prev_adc - (adc - 2) ;; trigger borrow flag
    asm("skipnc"); // skip next if (borrow -> still rising)
    asm("return");
    // still rising
    asm("decfsz _tmp_ctr");
    asm("goto labe_fast_RunADC_8_rising__continue");
    asm("return");

    asm("labe_fast_RunADC_8_rising__continue:");
    asm("movf " LOCATION__ADCVAL ",w"); // refresh last result (tmp_ctr2)
    asm("movwf _tmp_ctr2");
    asm("goto labe_fast_RunADC_8_rising__rising_cycle");
}

uint8_t fast_mul_8x8_res_h;
uint8_t fast_mul_8x8_res_l;
// multiplies w * PROJ_CONST_THR -> fast_mul_8x8_res_h:fast_mul_8x8_res_l
//// TODO consider tabled search (with tilted curve at very low power percentage? this requires accurate voltage divider)
//// optimum voltage seems to be around ~80% even below 1% of nominal power range of a panel: TODO verify again
void fast_mul_8x8() {
    asm("clrf _fast_mul_8x8_res_h");
    asm("clrf _fast_mul_8x8_res_l");

    #if (PROJ_CONST_THR & 0x80)
        asm("addwf _fast_mul_8x8_res_l,f");
        //asm("skipnc"); // redundant for the first multiplied bit
        //asm("incf _fast_mul_8x8_res_h"); // redundant for the first multiplied bit

        //asm("clrc"); // post 7, pre 6 // redundant for the first multiplied bit
        asm("rlf _fast_mul_8x8_res_l,f");
        asm("rlf _fast_mul_8x8_res_h,f");
    #endif
    #if (PROJ_CONST_THR & 0x40)
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");
            asm("clrc"); // post 6, pre 5 // only needed if changed after last RRF command
    #endif
            asm("rlf _fast_mul_8x8_res_l,f");
            asm("rlf _fast_mul_8x8_res_h,f");
    #if (PROJ_CONST_THR & 0x20)
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");
            asm("clrc"); // post 5, pre 4 // only needed if changed after last RRF command
    #endif
            asm("rlf _fast_mul_8x8_res_l,f");
            asm("rlf _fast_mul_8x8_res_h,f");
    #if (PROJ_CONST_THR & 0x10)
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");
            asm("clrc"); // post 4, pre 3 // only needed if changed after last RRF command
    #endif
            asm("rlf _fast_mul_8x8_res_l,f");
            asm("rlf _fast_mul_8x8_res_h,f");
    #if (PROJ_CONST_THR & 0x08)
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");
            asm("clrc"); // post 3, pre 2 // only needed if changed after last RRF command
    #endif
            asm("rlf _fast_mul_8x8_res_l,f");
            asm("rlf _fast_mul_8x8_res_h,f");
    #if (PROJ_CONST_THR & 0x04)
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");
            asm("clrc"); // post 2, pre 1 // only needed if changed after last RRF command
    #endif
            asm("rlf _fast_mul_8x8_res_l,f");
            asm("rlf _fast_mul_8x8_res_h,f");
    #if (PROJ_CONST_THR & 0x02)
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");
            asm("clrc"); // post 1, pre 0 // only needed if changed after last RRF command
    #endif
            asm("rlf _fast_mul_8x8_res_l,f");
            asm("rlf _fast_mul_8x8_res_h,f");
    #if (PROJ_CONST_THR & 0x01)
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");
    #endif
        // asm("return"); // auto generated
}

uint8_t last_raw_adc_val = 0;
uint8_t pending_raw_adc_val;
uint8_t thr_min;
uint8_t thr_hi; // high level voltage threshold
uint8_t pwr_level = 1; // active power level. 0,1 = disconnected; max,max+1 = straight open;
uint8_t tmp_ctr; // temporary
uint8_t tmp_ctr2; // temporary

asm("global _aux_delay_generic");
void aux_delay_generic() {
    // 2(call) + ... + 2(return)
    asm("labe_aux_delay_20:"); asm("fcall labe_aux_delay_4");
    asm("labe_aux_delay_16:"); asm("fcall labe_aux_delay_4");
    asm("labe_aux_delay_12:"); asm("fcall labe_aux_delay_4");
    asm("labe_aux_delay_8:");  asm("fcall labe_aux_delay_4");
    asm("return");
    asm("labe_aux_delay_7:"); asm("nop");
    asm("labe_aux_delay_6:"); asm("nop");
    asm("labe_aux_delay_5:"); asm("nop");
    asm("labe_aux_delay_4:");
    // 'return' is auto-generated
}
#define AUX_DELAY_1 asm("nop");
#define AUX_DELAY_2 asm("nop2"); // compiler knows what to do with nop2; apparently a jump to next command
#define AUX_DELAY_3 asm("nop2"); asm("nop");

#define AUX_DELAY_4 asm("fcall labe_aux_delay_4");
#define AUX_DELAY_5 asm("fcall labe_aux_delay_5");
#define AUX_DELAY_6 asm("fcall labe_aux_delay_6");
#define AUX_DELAY_7 asm("fcall labe_aux_delay_7");
#define AUX_DELAY_8 asm("fcall labe_aux_delay_8");
#define AUX_DELAY_9      asm("fcall labe_aux_delay_4"); asm("fcall labe_aux_delay_5");
#define AUX_DELAY_10     asm("fcall labe_aux_delay_4"); asm("fcall labe_aux_delay_6");
#define AUX_DELAY_11     asm("fcall labe_aux_delay_4"); asm("fcall labe_aux_delay_7");
#define AUX_DELAY_12 asm("fcall labe_aux_delay_12");
#define AUX_DELAY_13     asm("fcall labe_aux_delay_8"); asm("fcall labe_aux_delay_5");
#define AUX_DELAY_14     asm("fcall labe_aux_delay_8"); asm("fcall labe_aux_delay_6");
#define AUX_DELAY_15     asm("fcall labe_aux_delay_8"); asm("fcall labe_aux_delay_7");
#define AUX_DELAY_16 asm("fcall labe_aux_delay_16");
#define AUX_DELAY_17     asm("fcall labe_aux_delay_12"); asm("fcall labe_aux_delay_5");
#define AUX_DELAY_18     asm("fcall labe_aux_delay_12"); asm("fcall labe_aux_delay_6");
#define AUX_DELAY_19     asm("fcall labe_aux_delay_12"); asm("fcall labe_aux_delay_7");
#define AUX_DELAY_20     asm("fcall labe_aux_delay_12"); asm("fcall labe_aux_delay_8");
#define AUX_DELAY_21     asm("fcall labe_aux_delay_16"); asm("fcall labe_aux_delay_5");
#define AUX_DELAY_22     asm("fcall labe_aux_delay_16"); asm("fcall labe_aux_delay_6");
#define AUX_DELAY_23     asm("fcall labe_aux_delay_16"); asm("fcall labe_aux_delay_7");
#define AUX_DELAY_24     asm("fcall labe_aux_delay_16"); asm("fcall labe_aux_delay_8");
#define AUX_DELAY_25     asm("fcall labe_aux_delay_20"); asm("fcall labe_aux_delay_5");
#define AUX_DELAY_26     asm("fcall labe_aux_delay_20"); asm("fcall labe_aux_delay_6");
#define AUX_DELAY_27     asm("fcall labe_aux_delay_20"); asm("fcall labe_aux_delay_7");
#define AUX_DELAY_28     asm("fcall labe_aux_delay_20"); asm("fcall labe_aux_delay_8");

#define AUX_DELAY_32     asm("fcall labe_aux_delay_16"); asm("fcall labe_aux_delay_16");
#define AUX_DELAY_36     asm("fcall labe_aux_delay_20"); asm("fcall labe_aux_delay_16");
#define AUX_DELAY_40     asm("fcall labe_aux_delay_20"); asm("fcall labe_aux_delay_20");



#define PROJ_TEST_LIMIT_RETURN_2_TACTS \
    PROJ_SKIP_IF_LIMIT_NOTREACHED \
    asm("return");

void apply_pwr_level() __at(0x0040)
{
    // asm("bcf 3,5"); // ensure bank0 selected
    asm("movf _pwr_level,w");
    asm("addwf 2,f"); // PCL += pwr_level: tabled jump

    // TODO OPT MIGHT take advantage of jump to offset*2 to avoid intermediate jumps in some cases

    asm("goto labe_apply_pwr_level__pre_zero"); // 0 *
    asm("goto labe_apply_pwr_level__zero"); // 1
    asm("goto labe_apply_pwr_level__d2_c1"); // 2
    asm("goto labe_apply_pwr_level__d2_c2_long"); // 3
    asm("goto labe_apply_pwr_level__d2_c2_short"); // 4
    asm("goto labe_apply_pwr_level__d3_c1"); // 5
    asm("goto labe_apply_pwr_level__d2c1__d3c1"); // 6
    asm("goto labe_apply_pwr_level__d3c1__d3c1"); // 7
    asm("goto labe_apply_pwr_level__d3_c2"); // 8
    asm("goto labe_apply_pwr_level__d3_c3"); // 9
    asm("goto labe_apply_pwr_level__d3_c4"); // 10
    asm("goto labe_apply_pwr_level__d3_c5"); // 11
    asm("goto labe_apply_pwr_level__d3_c7"); // 12
    asm("goto labe_apply_pwr_level__d3_c9"); // 13
    asm("goto labe_apply_pwr_level__d3_c12"); // 14
    asm("goto labe_apply_pwr_level__d3_c15"); // 15
    asm("goto labe_apply_pwr_level__d3_c19"); // 16
    asm("goto labe_apply_pwr_level__d3_c24"); // 17
    asm("goto labe_apply_pwr_level__d4_c18"); // 18
    asm("goto labe_apply_pwr_level__d5_c15"); // 19
    asm("goto labe_apply_pwr_level__d6_c12"); // 20
    asm("goto labe_apply_pwr_level__d7_c10"); // 21
    asm("goto labe_apply_pwr_level__d9_c8"); // 22
    asm("goto labe_apply_pwr_level__d12_c6"); // 23
    asm("goto labe_apply_pwr_level__d15_c5"); // 24
    asm("goto labe_apply_pwr_level__d18_c4"); // 25
    asm("goto labe_apply_pwr_level__d21_c3"); // 26
    asm("goto labe_apply_pwr_level__max"); // 27
    asm("goto labe_apply_pwr_level__post_max"); // 28 max+1 *
    // end table
    // ----- 0
    asm("labe_apply_pwr_level__pre_zero:");
    asm("incf _pwr_level,f"); // edge handling
    asm("labe_apply_pwr_level__zero:");
        PROJ_SET_GPIO_CAPON_PWMOFF
        asm("return");

    // TODO handle PROJ_BIT_NUM_IN_INV_LIMIT as much as possible

    // ----- duration 2
    asm("labe_apply_pwr_level__d2_c1:");
        asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("return");

    asm("labe_apply_pwr_level__d2_c2_long:");
        asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        AUX_DELAY_8
        PROJ_TEST_LIMIT_RETURN_2_TACTS
        // TODO SIZE: can reduce delay and just jump to labe_apply_pwr_level__d2_c1
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("return");

    asm("labe_apply_pwr_level__d2_c2_short:");
        asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        AUX_DELAY_2
        PROJ_TEST_LIMIT_RETURN_2_TACTS
        // TODO SIZE: can reduce delay and just jump to labe_apply_pwr_level__d2_c1
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("return");

    // ----- duration 3
    asm("labe_apply_pwr_level__d3_c1:");
        asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        AUX_DELAY_1
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("return");

    // ----- duration 2+3
    asm("labe_apply_pwr_level__d2c1__d3c1:");
        asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        AUX_DELAY_5 // distribute load more evenly
        PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        AUX_DELAY_1
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("return");

    // ----- duration 3+3
    asm("labe_apply_pwr_level__d3c1__d3c1:");
        asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        AUX_DELAY_1
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        AUX_DELAY_5 // distribute load more evenly
        PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        AUX_DELAY_1
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("return");

    asm("labe_apply_pwr_level__d3_c2:");
        asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        AUX_DELAY_1
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        AUX_DELAY_1
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("return");

    asm("labe_apply_pwr_level__d3_c3:");
    asm("movlw 1");
    asm("goto labe_apply_pwr_level__d3_X2cycleP1_preinit");

    asm("labe_apply_pwr_level__d3_c4:");
    asm("movlw 2");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
    asm("goto labe_apply_pwr_level__d3_X2cycle_mid");

    asm("labe_apply_pwr_level__d3_c5:");
    asm("movlw 2");
    asm("goto labe_apply_pwr_level__d3_X2cycleP1_preinit");

    asm("labe_apply_pwr_level__d3_c7:");
    asm("movlw 3");
    asm("goto labe_apply_pwr_level__d3_X2cycleP1_preinit");

    asm("labe_apply_pwr_level__d3_c9:");
    asm("movlw 4");
    asm("goto labe_apply_pwr_level__d3_X2cycleP1_preinit");

    asm("labe_apply_pwr_level__d3_c12:");
    asm("movlw 6");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
    asm("goto labe_apply_pwr_level__d3_X2cycle_mid");

    asm("labe_apply_pwr_level__d3_c15:");
    asm("movlw 7");
    asm("goto labe_apply_pwr_level__d3_X2cycleP1_preinit");

    asm("labe_apply_pwr_level__d3_c19:");
    asm("movlw 9");
    asm("goto labe_apply_pwr_level__d3_X2cycleP1_preinit");

    asm("labe_apply_pwr_level__d3_c24:");
    asm("movlw 12");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
    asm("goto labe_apply_pwr_level__d3_X2cycle_mid");

    // ----- generic 3
    asm("labe_apply_pwr_level__d3_X2cycleP1_preinit:");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
    asm("labe_apply_pwr_level__d3_cycle:");
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_1
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("labe_apply_pwr_level__d3_X2cycle_mid:");
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_1
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d3_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_1
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d4_c18:");
    asm("movlw 8");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
    // ----- generic 4
    asm("labe_apply_pwr_level__d4_cycle:");
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_2
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_2
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d4_cycle");
            // last iterations - after cycle, to minimize further delay
            PROJ_TEST_LIMIT_RETURN_2_TACTS
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_2
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
                AUX_DELAY_2
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_2
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d5_c15:");
    asm("movlw 7");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
    // ----- generic 5
    asm("labe_apply_pwr_level__d5_cycle:");
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_3
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_1
            PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_3
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d5_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_3
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d6_c12:");
    asm("movlw 11");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
    // ----- generic 6
    asm("labe_apply_pwr_level__d6_cycle:");
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_4
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d6_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_4
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d7_c10:");
    asm("movlw 9");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
    // ----- generic 7
    asm("labe_apply_pwr_level__d7_cycle:");
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_5
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d7_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_5
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d9_c8:");
    asm("movlw 7");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
    // ----- generic 9
    asm("labe_apply_pwr_level__d9_cycle:");
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_7
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_2
            PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d9_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_7
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d12_c6:");
    asm("movlw 5");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
    // ----- generic 12
    asm("labe_apply_pwr_level__d12_cycle:");
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_10
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_5
            PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d12_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_10
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d15_c5:");
    asm("movlw 4");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
    // ----- generic 15
    asm("labe_apply_pwr_level__d15_cycle:");
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_13
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_8
            PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d15_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_13
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d18_c4:");
    asm("movlw 3");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w"); // relying on getting here only when current PWM state=0
    // ----- generic 18
    asm("labe_apply_pwr_level__d18_cycle:");
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_16
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_11
            PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d18_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_16
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");


    // special handler
    asm("labe_apply_pwr_level__special:");
    asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
    asm("movwf 5"); /* assign GPIO: output = 0 */
        AUX_DELAY_17
    asm("goto labe_apply_pwr_level__pre_max_level");
    //

    asm("labe_apply_pwr_level__d21_c3:");
    asm("movlw 2");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    //
    asm("btfsc 5," AUX_STRINGIFY(PROJ_BIT_NUM_OUT_PWM_0)); // check if we came here from pwr MAX - then pre-apply 0
    asm("goto labe_apply_pwr_level__special");
    //
    // ----- generic 21
    asm("labe_apply_pwr_level__pre_max_level:");
    asm("labe_apply_pwr_level__d21_cycle:");
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_19
        asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_14
            PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d21_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_19
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("xorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_ALL_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__post_max:");
    asm("decf _pwr_level,f"); // edge handling
    asm("labe_apply_pwr_level__max:");
        PROJ_SET_GPIO_CAPON_PWMON
        asm("return");
}

void main() {
    TRISIO = ((1 << PROJ_BIT_NUM_OUT_PWM_0)
            | (1 << PROJ_BIT_NUM_OUT_PWM_1)
            | (1 << PROJ_BIT_NUM_OUT_CAP)
#ifdef PROJ_BIT_NUM_OUT_INV_PWM
            | (1 << PROJ_BIT_NUM_OUT_INV_PWM)
#endif
            ) ^ 0x3F
            ;
    ANSEL = (PROJ_ADC_ADCS << 4) | (1 << PROJ_ADC_CH_VOL); // set conversion time and ADC channel

    asm("bcf 3,5"); // ensure bank0 selected
    PROJ_SET_GPIO_CAPON_PWMOFF // initial assignment

    CMCON = 0x07;
    WPU = 0xFF; // all possible pullups
    OSCCAL = __osccal_val();

    is_prev_cycle_pwr = 0;

    app_flags = 0;

    OPTION_REG = (PROJ_TMR_PRESCALE); // (!nGPPU - enable pullups), prescaler setup
    TMR1L = 0;
    TMR1H = 0;
    T1CON = 0x31; // enable timer1 with x8 prescaler;
    // INTCON = 0x00; // (default is 0x00); no interrupts

    asm("bcf 3,5"); // ensure bank0 selected
    PROJ_ADC_SETUP__WHEN_BANK0()
    AUX_DELAY_40
    PROJ_ADC_START()
    PROJ_ADC_WAIT("labe_preinit_adc_min_voltage")
    asm("movf " LOCATION__ADCVAL ",w"); // w = ADRESH
    asm("movwf _thr_min"); // thr_min = w (ADC result)

    // TODO revise thr_min? in case of having very high startup current (connecting to high-saturated solar; which would drive thr_min too high)
    // this is probably not necessary with 'PWRTE = OFF' as startup time is very small. TODO calculate/verify
    asm("movlw 6");
    asm("addwf _thr_min,f"); // +2%. ensure no overflow
    asm("skipnc"); // must be no overflow - otherwise it's mistake in scheme or ADC failure
    asm("subwf _thr_min,f"); // undo if overflowed

    // pre-init raw adc with pre-wait 1 second
    {
        asm("main__time_waiter:");
        asm("btfss " LOCATION__PIR1bits_TMR1IF);
        asm("goto main__time_waiter");
        asm("bcf " LOCATION__PIR1bits_TMR1IF);
        asm("decfsz _timer_multiplier,f");
        asm("goto main__time_waiter");
            asm("bsf _timer_multiplier," AUX_STRINGIFY(PROJ_TIMER_MULTIPLIER_AS_BIT));
    }
    {
        PROJ_SET_GPIO_CAPON_PWMOFF
        PROJ_SET_GPIO_CAPOFF_PWMOFF

        asm("fcall _fast_RunADC_8_rising");
        asm("movf " LOCATION__ADCVAL ",w"); // w = ADRESH (ADC result)
        asm("movwf _last_raw_adc_val");

        asm("bsf _gp_shadow," AUX_STRINGIFY(PROJ_BIT_NUM_OUT_CAP));
        asm("movf _gp_shadow,w");
        asm("movwf 5"); // attach capacitor
    }
    {
        //asm("bcf 3,5"); // ensure bank0 selected
        asm("movf _last_raw_adc_val,w");
        asm("fcall _fast_mul_8x8");

        asm("movf _fast_mul_8x8_res_h,w");
        asm("movwf _thr_hi"); // thr_hi = last_raw_adc_val * g_thr / 256
        //
        asm("subwf _thr_min,w"); // w(thr_hi) = thr_min - w ; if borrow - no action
        asm("skipnc"); // skip next if borrow
        asm("addwf _thr_hi,f"); // thr_hi += (thr_min - thr_hi)
    }
    //
    PROJ_ADC_START()
    AUX_DELAY_3 // align with cycle internal delays
    asm("labe_main__cycle:");
        PROJ_SKIP_IF_LIMIT_NOTREACHED
        asm("goto labe_main__cycle_handle_limit_pocket");
            // else - limit not reached
            asm("bsf _is_prev_cycle_pwr,0");
            asm("incf _pwr_level,f"); // pre-increment (will either stay incremented...)
            asm("movf _thr_hi,w"); // w = thr_hi
            // ADC has finished by this point - commands between launching ADC and using result execute 8 tacts, so 9 tacts (36Tosc) inbetween
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi
            asm("movlw 2"); // doesn't affect status
            asm("skipc"); // c == no borrow
            asm("subwf _pwr_level,f"); // pwr_level = pwr_level - w(2) (...or will be decremented)
            // TODO consider higher drop-offs as some short-circut handling/more aggressive balancing?

            asm("fcall _apply_pwr_level");
            // main part done


        asm("labe_main__cycle_pre_check_flags:");
        PROJ_ADC_START()
        asm("btfss " LOCATION__PIR1bits_TMR1IF);
        asm("goto labe_main__cycle");
        asm("btfsc _app_flags," AUX_STRINGIFY(PROJ_APP_FLAG_CYCLE_RETURN_TRAP));
        asm("return");

        asm("decfsz _timer_multiplier,f");
        asm("goto labe_main__cycle_timer_unlocker");
            asm("bsf _timer_multiplier," AUX_STRINGIFY(PROJ_TIMER_MULTIPLIER_AS_BIT));

        asm("bsf _app_flags," AUX_STRINGIFY(PROJ_APP_FLAG_CYCLE_RETURN_TRAP));
        // measure/update raw voltage
        PROJ_SET_GPIO_CAPON_PWMOFF
        PROJ_SET_GPIO_CAPOFF_PWMOFF
        // detached cap, give some time to establish raw voltage
        asm("fcall _fast_RunADC_8_rising");
        PROJ_SET_GPIO_CAPON_PWMOFF // attach capacitor back
        // save result
        asm("movf " LOCATION__ADCVAL ",w"); // w = ADRESH (ADC result)
        asm("movwf _pending_raw_adc_val");
        //// measured; run a cycle to minimize delay gap
            PROJ_ADC_START()
            AUX_DELAY_1
            asm("fcall labe_main__cycle");
        //// refresh thr if needed
        asm("movf _pending_raw_adc_val,w"); // TODO input1 can be union with last_raw_adc_val to save 2 commands
        asm("xorwf _last_raw_adc_val,w");
        asm("skipnz");
        asm("goto labe_main__cycle_flags_done"); // no changes
        // different - needs recalculation

        asm("clrf _fast_mul_8x8_res_h");
        asm("clrf _fast_mul_8x8_res_l");
            //// run cycle - minimize difference in delays
            PROJ_ADC_START()
            AUX_DELAY_1 // align delays
            asm("fcall labe_main__cycle");
            ////

        asm("movf _pending_raw_adc_val,w");
        asm("movwf _last_raw_adc_val");
        // inlined multiplication
        #if (PROJ_CONST_THR & 0x80)
            asm("addwf _fast_mul_8x8_res_l,f");
            asm("rlf _fast_mul_8x8_res_l,f");
            asm("rlf _fast_mul_8x8_res_h,f");
        #endif
        #if (PROJ_CONST_THR & 0x40)
            asm("addwf _fast_mul_8x8_res_l,f");
            asm("skipnc");
            asm("incf _fast_mul_8x8_res_h");
        #endif
                asm("clrc"); // post 6, pre 5
                asm("rlf _fast_mul_8x8_res_l,f");
                asm("rlf _fast_mul_8x8_res_h,f");
            //// run cycle - minimize difference in delays
            PROJ_ADC_START()
            AUX_DELAY_1 // align delays
            asm("fcall labe_main__cycle");
            asm("movf _pending_raw_adc_val,w"); // recover W
            ////
        #if (PROJ_CONST_THR & 0x20)
            asm("addwf _fast_mul_8x8_res_l,f");
            asm("skipnc");
            asm("incf _fast_mul_8x8_res_h");
        #endif
                asm("clrc"); // post 5, pre 4
                asm("rlf _fast_mul_8x8_res_l,f");
                asm("rlf _fast_mul_8x8_res_h,f");
        #if (PROJ_CONST_THR & 0x10)
            asm("addwf _fast_mul_8x8_res_l,f");
            asm("skipnc");
            asm("incf _fast_mul_8x8_res_h");
        #endif
                asm("clrc"); // post 4, pre 3
                asm("rlf _fast_mul_8x8_res_l,f");
                asm("rlf _fast_mul_8x8_res_h,f");
        #if (PROJ_CONST_THR & 0x08)
            asm("addwf _fast_mul_8x8_res_l,f");
            asm("skipnc");
            asm("incf _fast_mul_8x8_res_h");
        #endif
                asm("clrc"); // post 3, pre 2
                asm("rlf _fast_mul_8x8_res_l,f");
                asm("rlf _fast_mul_8x8_res_h,f");
        #if (PROJ_CONST_THR & 0x04)
            asm("addwf _fast_mul_8x8_res_l,f");
            asm("skipnc");
            asm("incf _fast_mul_8x8_res_h");
        #endif
                asm("clrc"); // post 2, pre 1
                asm("rlf _fast_mul_8x8_res_l,f");
                asm("rlf _fast_mul_8x8_res_h,f");
            //// run cycle - minimize difference in delays
            PROJ_ADC_START()
            AUX_DELAY_1 // align delays
            asm("fcall labe_main__cycle");
            asm("movf _pending_raw_adc_val,w"); // recover W
            ////
        #if (PROJ_CONST_THR & 0x02)
            asm("addwf _fast_mul_8x8_res_l,f");
            asm("skipnc");
            asm("incf _fast_mul_8x8_res_h");
        #endif
                asm("clrc"); // post 1, pre 0
                asm("rlf _fast_mul_8x8_res_l,f");
                asm("rlf _fast_mul_8x8_res_h,f");
        #if (PROJ_CONST_THR & 0x01)
            asm("addwf _fast_mul_8x8_res_l,f");
            asm("skipnc");
            asm("incf _fast_mul_8x8_res_h");
        #endif
        // end ------------------
        asm("movf _fast_mul_8x8_res_h,w");
        asm("movwf _thr_hi"); // thr_hi = last_raw_adc_val * g_thr / 256

        asm("subwf _thr_min,w"); // w(thr_hi) = thr_min - w ; if borrow - no action
        asm("skipnc"); // skip next if borrow
        asm("addwf _thr_hi,f"); // thr_hi += (thr_min - thr_hi)

            //// run cycle - minimize difference in delays
            PROJ_ADC_START()
            AUX_DELAY_1 // align delays
            asm("fcall labe_main__cycle");
            ////

        // ---------------------------------
        asm("labe_main__cycle_flags_done:");
        asm("bcf _app_flags," AUX_STRINGIFY(PROJ_APP_FLAG_CYCLE_RETURN_TRAP));
        asm("labe_main__cycle_timer_unlocker:");
        PROJ_ADC_START()
        asm("bcf " LOCATION__PIR1bits_TMR1IF);
        asm("goto labe_main__cycle");

        // ------------------------------------------
        asm("labe_main__cycle_handle_limit_pocket:");
            PROJ_SET_GPIO_CAPON_PWMOFF // ensure output is off (in case we're at max level)
            asm("btfss _is_prev_cycle_pwr,0");
            asm("goto labe_main__cycle_pre_check_flags");
                asm("decf _pwr_level"); // decrement pwr level
                // ADC result is ready by this moment
                asm("movf _thr_hi,w"); // w = thr_hi
                asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi
                asm("skipnc"); // nc == borrow
                asm("decf _pwr_level"); // decrement again if adcval >= thr_hi; as it will very likely be incremented on next cycle
                // check underflow - ensure minimal pwr_level is 1 after this block
                asm("decf _pwr_level"); // ---
                asm("btfsc _pwr_level,7"); // test msb
                asm("clrf _pwr_level"); // if underflowed - reset
                asm("incf _pwr_level"); // ---
            asm("bcf _is_prev_cycle_pwr,0");
            asm("goto labe_main__cycle_pre_check_flags");
} //



asm("global _aux_version_info_footer");
void aux_version_info_footer(void) __at(0x03FF - 5) {
    asm("retlw " AUX_STRINGIFY(FW_VER_MAJOR));
    asm("retlw " AUX_STRINGIFY(FW_VER_MINOR));
    asm("retlw " AUX_STRINGIFY(FW_VER_THR));
    asm("retlw " AUX_STRINGIFY(FW_VER_OPTION));
    // 'return' is auto-generated here so 1 last byte is skipped to ensure it won't erase OSCCAL
}
