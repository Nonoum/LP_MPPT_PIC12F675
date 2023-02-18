/*
    A simple MPPT controller with non-canonical design, developed By Evgeniy Evstratov, Autumn 2022 and further.

    This Solar controller is being developed in these dark times when russia is destroying energetic system of my country and has already caused full-scale blackout.

    The project is intended as LOW-COST simple MPPT device that works well in low-power range and:
    - is affordable (very cheap MCU, small and simple scheme, *NO CURRENT SENSOR);
    - gets maximum solar power generation in cloudy and winter weather (proper work specifically in low power range: starting from 0.01 A).
        -- allows using high voltage panels for low-voltage devices in such conditions when very small amount of solar energy can be generated (e.g. using 18v panel to charge a smartphone powerbank).

    Global disclaimer: author has very limited testing capabilities, any hardware implementaion should be tested in required scenarios to ensure proper/correct work.

    Principle of work: according to solar panels voltage/current curves at very most part of the range the maximum power point (MPP) is at around 80-81% of open circuit voltage (OCV, also named 'raw voltage' here).
    So with certain scheme it's possible to measure (almost)open circuit voltage (with minimal load) and track optimal mode without need of a current sensor.

    Basic scheme description in words:
        1. solar panel should be connected via diode (shottky SR840 is used in prototypes).
        2. input("high voltage" and "gnd") is connected to linear voltage stabilizer and minimal(!) required ceramic capacitor (0.1 - 0.15uf):
            0.15uf cap and "LP2950CZ-3.3G ON" stabilizer are used in the first prototype (similar 5v stabilizer is used in second prototype).
        3. linear stabilizer has output ceramic capacitor (used 2.2uf in the first prototype) and is connected as the PIC MCU power supply;
            * it turned out that it's necessary to add extra (larger) output capacitor to ensure adequate work of LP2950CZ, used extra 47uf electrolythic cap;
 *          * added extra 47uf capacitor requires FW 0.10 and higher for correct work (related to removed measuring of startup voltage);
            * no mini buck converter can be used instead, due to it's high input capacitance.
        4. one N-channel mosfet connects a large buffer capacitor between "high voltage" and "gnd" and occassionally (~once a second) disconnects it to measure open circuit voltage;
            * required PULLUP EXTERNAL resistor (used 200kOhm) - pull to stabilizer's voltage (which is 3.3v or 5v);
            heavy 5x 1000uf 35v LOW ESR capacitors are used in the first prototype;
            2x 1000uf are used in second prototype.
        5. second N-channel mosfet connects an output circuit (inductor+diode+capacitor) between "high voltage" and "gnd" and is being PWMed;
            * required PULLDOWN EXTERNAL resistor (used 200kOhm) - pull to "gnd";
            recommended inductor is around 22uh - 47uh (preferably 33uh, but optimal choice depends on circumstances);
            * different inductors can be used depending on other conditions - calculate for your needs;
            recommended output capacitor is 2000uf or more.
        6. a calculated voltage divider between "high voltage" and "gnd" is connected to PIC MCU's pin with ADC channel:
            so that maximum possible voltage is <= nominal voltage of linear stabilizer;
            3.3kOhm : 20kOhm devider is used in first prototype (with 3.3v stabilizer), I'd recommend to not exceed (not significantly) sum resistance around 1kOhm per 1v nominal of solar panel;
            5.6kOhm : 22kOhm divider is used in second prototype with 5v stabilizer.
        7. mosfets used in prototype are "IRLZ34N" - pretty low input capacitance, proper DS opening at 3.3v;
            second prototype uses 'buffer' mosfet IRL2203.
        8. with this minimum schematic the output is not regulated so it can reach up to "open circuit voltage" of the solar panel connected,
            as a very simple voltage limiter - a buck(step-down) converter could be used;
            * FW has native limiting mechanism - see FW_VER_OPTION notes section.

NOTES for release 0.10.*.*:
    Features:
        - "soft start" in current and all foreseeable future versions;
        - low self-consumption: prototypes consume 1..2 ma (stable state with no load), having worse linear stabilizer it would be a little more;
        - output limiting PIN (second prototype uses extra circuit with an optocoupler and a comparator to implement limiting logic);
        - inverted PWM pin (fully synchronous with non-inverted, might be used with external fet drivers for very-high power devices).

    PIC MCU notes: there are different packages with at least two working voltage ranges: some PICs have 3.6v limit (only 3.3v stabilizer can be used in this case);

    Efficiency notes:
        - second prototype has shown conversion efficieny (output RMS power / input RMS power) from around 80 to 97%;
        - efficieny depends on output voltage and power - lookup charts for buck DC-DC converters, typical efficiency of second prototype with 5v output is 87%, with 12v output - 92-94%;
        - in described testing 18v panel was used (with typical MPP ranging within 15 ... 18v);
        - no extra mosfet drivers are needed at least for mosfets with input capacitance up to around 3nf;

    Pulsation notes:
        - output voltage pulsations are highly reduced since initial 0.1 FW version;
        - pulsations are different on different power ranges, overall pretty small, on higher power ranges the device can drive inductor in continuous mode and minimize pulsations even further;
        - even higher pulsations are *presumably okay for charging batteries with good BMS;
        - increasing capacitance (both buffer and output caps) should smooth/improve pulsations (if needed);
        - capacitance can be smaller or bigger than in prototypes described above, suggestions for buffer and output caps are:
            -- minimum: 1000uf (buffer); 1000uf per 1A output current (output cap);
            -- recommended: 2000uf (buffer), 2000uf per 1A output current (output cap);
        - pulsations can be induced by MPP overshoot, which can significantly reduce input current and thus generated power - see "PROJ_CONST_THR" notes and "Input diode Vf" notes;
        - pulsations can be induced by partial shading (when shading occurs somewhere inseries);

    PWM notes:
        - code does manual PWM with duty (open output mosfet) lengths starting from 2us;
        - for mosfet driving power considerations expect average switching (ON->OFF or OFF->ON) interval of 3us (worst case), single PIN is rated for 25ma, dual pin - 50ma;
        - currently (FW 0.10) maximum PWM duty ON length is 40us (+fully open state that can lead this to infinity).

    More output voltage limiting notes:
        - powerbank:
            - tested powerbank can be tricked to trigger QC mode and rise voltage - when it's connected it "requests" higher voltage while consuming no substantial load
            and the actual voltage can be rising at the moment so powerbank considers that everything is according to plan and accepts it;
            - if voltage manages to rise higher than powerbank's limit - it will shut down and won't charge (*or powerbank will die - happened with one simple model when 12v was applied during testing);
            - plain buck converter (when added after output) is apparenty fully "opened" and acts as a minor resistance (inductor+mosfet) with it's input and output voltages being almost equal;
        - powerbank2: pay attention at powerbank's capabilities - if it doesn't support QC/PD then it will likely die from higher voltages (not just shut-off).

    Adding to existing solar (battery + PWM charger) system:
        - it's assumed that this controller can be simply connected in series between solar panel and thirdparty "PWM Solar controller" to increase system efficiency,
        although it's not tested and I'm not 100% that a PWM controller would handle this without any issues
        (specifically when charging is done and PWM controller's input voltage goes up);

    Using as-is with non-regulated output with a BMS and batteries: *presumably will work fine, might trigger BMS's overvoltage protection in certain cases (depends on BMS).

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
#define FW_VER_MINOR 10
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
#define AUX_DELAY_20 asm("fcall labe_aux_delay_20");
#define AUX_DELAY_21     asm("fcall labe_aux_delay_16"); asm("fcall labe_aux_delay_5");
#define AUX_DELAY_22     asm("fcall labe_aux_delay_16"); asm("fcall labe_aux_delay_6");
#define AUX_DELAY_23     asm("fcall labe_aux_delay_16"); asm("fcall labe_aux_delay_7");
#define AUX_DELAY_24     asm("fcall labe_aux_delay_16"); asm("fcall labe_aux_delay_8");
#define AUX_DELAY_25     asm("fcall labe_aux_delay_20"); asm("fcall labe_aux_delay_5");
#define AUX_DELAY_26     asm("fcall labe_aux_delay_20"); asm("fcall labe_aux_delay_6");
#define AUX_DELAY_27     asm("fcall labe_aux_delay_20"); asm("fcall labe_aux_delay_7");
#define AUX_DELAY_28     asm("fcall labe_aux_delay_20"); asm("fcall labe_aux_delay_8");
//
#define AUX_DELAY_32     asm("fcall labe_aux_delay_16"); asm("fcall labe_aux_delay_16");
#define AUX_DELAY_36     asm("fcall labe_aux_delay_20"); asm("fcall labe_aux_delay_16");
#define AUX_DELAY_40     asm("fcall labe_aux_delay_20"); asm("fcall labe_aux_delay_20");

#define AUX_1CMD_DELAY_4 asm("fcall labe_aux_delay_4");
#define AUX_1CMD_DELAY_5 asm("fcall labe_aux_delay_5");
#define AUX_1CMD_DELAY_6 asm("fcall labe_aux_delay_6");
#define AUX_1CMD_DELAY_7 asm("fcall labe_aux_delay_7");
#define AUX_1CMD_DELAY_8 asm("fcall labe_aux_delay_8");
#define AUX_1CMD_DELAY_12 asm("fcall labe_aux_delay_12");
#define AUX_1CMD_DELAY_16 asm("fcall labe_aux_delay_16");
#define AUX_1CMD_DELAY_20 asm("fcall labe_aux_delay_20");



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

#define PROJ_ADC_ADCS 0b000 // conversion time ~ 5.5 tacts


uint8_t is_prev_cycle_pwr; // [0:1], if previous cycle applied pwr (limit wasn't reached in the beginning)

uint8_t app_flags;
#define PROJ_APP_FLAG_REDUCE_3_TO_2 1

#define PROJ_TIMER_MULTIPLIER_AS_BIT 1 // multiplier is a degree of 2
uint8_t timer_multiplier = (1 << PROJ_TIMER_MULTIPLIER_AS_BIT);

// parameters for raw voltage measuring: temporary flow, to be updated (developed before finding the culprit of ADC issue)
#define PROJ_ADC_ARRAY_SIZE 7
#define PROJ_ADC_ARRAY_ADDR 32
uint8_t adc_array[PROJ_ADC_ARRAY_SIZE] __at(0x0020);
uint8_t adc_last_val;


#define LOCATION__PIR1bits_TMR1IF "12,0"
#define LOCATION__ADCVAL "30" // ADRESH reg
#define LOCATION__FSR "4"
#define LOCATION__INDF "0"

// eeprom-related locations (NOTE: registers bank1)
#define LOCATION__EEDATA "9Ah"
#define LOCATION__EEADDR "9Bh"
#define LOCATION__EECON1 "9Ch"
#define LOCATION__EECON2 "9Dh"
#define LOCATION__EECON1_WREN "9Ch,2"
#define LOCATION__EECON1_WR "9Ch,1"


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
    asm("movwf 5"); /* assign GPIO */
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

// adjusts tmp_ctr, tmp_ctr2, adc_* vars, controls GPIO internally
void fast_RunADC_8_rising() {
    PROJ_SET_GPIO_CAPON_PWMOFF
    PROJ_SET_GPIO_CAPOFF_PWMOFF
    asm("movlw " AUX_STRINGIFY(PROJ_ADC_ARRAY_ADDR - 1));
    asm("movwf " LOCATION__FSR);
    PROJ_ADC_START()
    asm("movlw " AUX_STRINGIFY(PROJ_ADC_ARRAY_SIZE));
    asm("movwf _tmp_ctr"); // max iterations by 2 samples
    asm("clrf _tmp_ctr2"); // amount of equal values
    asm("incf _tmp_ctr2"); // = 1
    asm("clrf _adc_last_val");
    asm("labe_fast_RunADC_8_rising__rising_cycle:");
        asm("incf " LOCATION__FSR); // ++addr
        asm("movf " LOCATION__ADCVAL ",w");
        PROJ_ADC_START()
        asm("movwf " LOCATION__INDF);
        asm("xorwf _adc_last_val,f"); // compare
        asm("movwf _adc_last_val"); // update last value / doesn't affect flags
        asm("skipz");
            asm("clrf _tmp_ctr2"); // clear amount of equal values if mismatch
        asm("incf _tmp_ctr2"); // ++
        asm("btfsc _tmp_ctr2,2"); // test if >= 4
            asm("goto labe_fast_RunADC_8_rising__match_quit");
        asm("movf " LOCATION__ADCVAL ",w");
        PROJ_ADC_START()
        asm("addwf " LOCATION__INDF ",f"); // form ...
        asm("rrf " LOCATION__INDF ",f"); //   ...  avg
        asm("xorwf _adc_last_val,f"); // compare
        asm("movwf _adc_last_val"); // update last value / doesn't affect flags
        asm("skipz");
            asm("clrf _tmp_ctr2"); // clear amount of equal values if mismatch
        asm("incf _tmp_ctr2"); // ++
        asm("btfsc _tmp_ctr2,2"); // test if >= 4
            asm("goto labe_fast_RunADC_8_rising__match_quit");
        asm("decfsz _tmp_ctr");
            asm("goto labe_fast_RunADC_8_rising__rising_cycle");

    // timed-out, calculate avg of last 4 entries (8 samples)
    PROJ_SET_GPIO_CAPON_PWMOFF // attach capacitor back
        PROJ_ADC_START() // run adc several times now to adjust to normal state
    asm("movf " LOCATION__INDF ",w"); // last val
    asm("decf " LOCATION__FSR); // --addr
    asm("addwf " LOCATION__INDF ",f"); // form ...
    asm("rrf " LOCATION__INDF ",w"); //   ...  avg and store it in W
    asm("movwf _tmp_ctr"); // tmp_ctr = avg of last 4 samples
    asm("decf " LOCATION__FSR); // --addr
        PROJ_ADC_START()
    asm("movf " LOCATION__INDF ",w");
    asm("decf " LOCATION__FSR); // --addr
    asm("addwf " LOCATION__INDF ",f"); // form ...
    asm("rrf " LOCATION__INDF ",w"); //   ...  avg and store it in W
    asm("movwf _adc_last_val"); // adc_last_val = avg of pre last 4 samples
    asm("movf _tmp_ctr,w");
        PROJ_ADC_START()
    asm("addwf _adc_last_val,f");
    asm("rrf _adc_last_val,f"); // adc_last_val = avg of last 8 adc samples
    asm("return");

    asm("labe_fast_RunADC_8_rising__match_quit:");
        PROJ_SET_GPIO_CAPON_PWMOFF // attach capacitor back
        PROJ_ADC_START() // run adc several times now to adjust to normal state
        // TODO useful work while running adc one more time?
        asm("return");
}

uint8_t fast_mul_8x8_res_h;
uint8_t fast_mul_8x8_res_l;
// multiplies w * PROJ_CONST_THR -> fast_mul_8x8_res_h:fast_mul_8x8_res_l
//// TODO consider tabled search if have sufficient space (with tilted curve at very low power percentage? this requires accurate voltage divider)
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
uint8_t thr_min;
uint8_t thr_hi; // high level voltage threshold
uint8_t pwr_level = 1; // active power level. 0,1 = disconnected; max,max+1 = straight open;
uint8_t tmp_ctr; // temporary
uint8_t tmp_ctr2; // temporary


#define PROJ_TEST_LIMIT_RETURN_2_TACTS \
    PROJ_SKIP_IF_LIMIT_NOTREACHED \
    asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));

#define PROJ_DEFINE_BLOCK_2V3_NO_PRECHECK(lbl_part_name) \
    asm("labe_apply_pwr_level__" lbl_part_name ":"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); \
        asm("movwf 5");

#define PROJ_DEFINE_BLOCK_2V3(lbl_part_name) \
    asm("labe_apply_pwr_level__" lbl_part_name "__precheck:"); PROJ_SKIP_IF_LIMIT_REACHED \
    PROJ_DEFINE_BLOCK_2V3_NO_PRECHECK(lbl_part_name)



#define PROJ_DEFINE_BLOCK_3V3_NO_PRECHECK(lbl_part_name) \
    asm("labe_apply_pwr_level__" lbl_part_name ":"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        AUX_DELAY_1 \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); \
        asm("movwf 5");

#define PROJ_DEFINE_BLOCK_3V3(lbl_part_name) \
    asm("labe_apply_pwr_level__" lbl_part_name "__precheck:"); PROJ_SKIP_IF_LIMIT_REACHED \
    PROJ_DEFINE_BLOCK_3V3_NO_PRECHECK(lbl_part_name)

#define PROJ_1CMD_SET_REDUCE_3_TO_2() asm("bcf _app_flags," AUX_STRINGIFY(PROJ_APP_FLAG_REDUCE_3_TO_2));



#define PROJ_DEFINE_BLOCK_4V4_NO_PRECHECK(lbl_part_name) \
    asm("labe_apply_pwr_level__" lbl_part_name ":"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); /* = 1 */ \
        AUX_DELAY_2 \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); \
        asm("movwf 5"); /* = 0 */ \
        AUX_DELAY_1

#define PROJ_DEFINE_BLOCK_4V4(lbl_part_name) \
    asm("labe_apply_pwr_level__" lbl_part_name "__precheck:"); PROJ_SKIP_IF_LIMIT_REACHED \
    PROJ_DEFINE_BLOCK_4V4_NO_PRECHECK(lbl_part_name)



#define PROJ_DEFINE_BLOCK_COMBO_34(part1, part2) \
    asm("labe_apply_pwr_level__" part1 "__" part2 ":"); \
        asm("fcall labe_apply_pwr_level__" part1); \
        asm("movwf 5"); \
        asm("goto labe_apply_pwr_level__" part2 "__precheck");

#define PROJ_DEFINE_BLOCK_COMBO_45(part1, num2) \
    asm("labe_apply_pwr_level__" part1 "__d5v4_c" AUX_STRINGIFY(num2) ":"); \
        asm("fcall labe_apply_pwr_level__" part1); \
        asm("movwf 5"); \
        PROJ_TEST_LIMIT_RETURN_2_TACTS \
        asm("movlw " AUX_STRINGIFY(num2 - 1)); \
        asm("movwf _tmp_ctr"); \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        asm("goto labe_apply_pwr_level__d5v4_generic_enter");

#define PROJ_DEFINE_BLOCK_COMBO_56(num1, num2) \
    asm("labe_apply_pwr_level__d5v4_c" AUX_STRINGIFY(num1) "__d6v4_c" AUX_STRINGIFY(num2) ":"); \
        asm("movlw " AUX_STRINGIFY(num1 - 1)); \
        asm("movwf _tmp_ctr"); \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        asm("fcall labe_apply_pwr_level__d5v4_generic_enter"); \
        asm("movwf 5"); \
        PROJ_TEST_LIMIT_RETURN_2_TACTS \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        asm("movlw " AUX_STRINGIFY(num2 - 1)); \
        asm("goto labe_apply_pwr_level__d6v4_generic_enter_minus1");

#define PROJ_DEFINE_BLOCK_COMBO_67(num1, num2) \
    asm("labe_apply_pwr_level__d6v4_c" AUX_STRINGIFY(num1) "__d7v4_c" AUX_STRINGIFY(num2) ":"); \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        asm("movlw " AUX_STRINGIFY(num1 - 1)); \
        asm("fcall labe_apply_pwr_level__d6v4_generic_enter_minus1"); \
        asm("movwf 5"); \
        PROJ_TEST_LIMIT_RETURN_2_TACTS \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        asm("movlw " AUX_STRINGIFY(num2 - 1)); \
        asm("goto labe_apply_pwr_level__d7v4_generic_enter_minus1");

#define PROJ_DEFINE_BLOCK_COMBO_78(num1, num2) \
    asm("labe_apply_pwr_level__d7v4_c" AUX_STRINGIFY(num1) "__d8v4_c" AUX_STRINGIFY(num2) ":"); \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        asm("movlw " AUX_STRINGIFY(num1 - 1)); \
        asm("fcall labe_apply_pwr_level__d7v4_generic_enter_minus1"); \
        asm("movwf 5"); \
        PROJ_TEST_LIMIT_RETURN_2_TACTS \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        asm("movlw " AUX_STRINGIFY(num2 - 1)); \
        asm("goto labe_apply_pwr_level__d8v4_generic_enter_minus1");

#define PROJ_DEFINE_BLOCK_COMBO_89(num1, num2) \
    asm("labe_apply_pwr_level__d8v4_c" AUX_STRINGIFY(num1) "__d9v4_c" AUX_STRINGIFY(num2) ":"); \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        asm("movlw " AUX_STRINGIFY(num1 - 1)); \
        asm("fcall labe_apply_pwr_level__d8v4_generic_enter_minus1"); \
        asm("movwf 5"); \
        PROJ_TEST_LIMIT_RETURN_2_TACTS \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        asm("movlw " AUX_STRINGIFY(num2 - 1)); \
        asm("goto labe_apply_pwr_level__d9v4_generic_enter_minus1");

#define PROJ_DEFINE_BLOCK_COMBO_910(num1, num2) \
    asm("labe_apply_pwr_level__d9v4_c" AUX_STRINGIFY(num1) "__d10v4_c" AUX_STRINGIFY(num2) ":"); \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        asm("movlw " AUX_STRINGIFY(num1 - 1)); \
        asm("fcall labe_apply_pwr_level__d9v4_generic_enter_minus1"); \
        asm("movwf 5"); \
        PROJ_TEST_LIMIT_RETURN_2_TACTS \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        asm("movlw " AUX_STRINGIFY(num2 - 1)); \
        asm("goto labe_apply_pwr_level__d10v4_generic_enter_minus1");

#define PROJ_DEFINE_BLOCK_COMBO_1011(num1, num2) \
    asm("labe_apply_pwr_level__d10v4_c" AUX_STRINGIFY(num1) "__d11v4_c" AUX_STRINGIFY(num2) ":"); \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        asm("movlw " AUX_STRINGIFY(num1 - 1)); \
        asm("fcall labe_apply_pwr_level__d10v4_generic_enter_minus1"); \
        asm("movwf 5"); \
        PROJ_TEST_LIMIT_RETURN_2_TACTS \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); \
        asm("movlw " AUX_STRINGIFY(num2 - 1)); \
        asm("goto labe_apply_pwr_level__d11v4_generic_enter_minus1");



#define PROJ_SPECIAL_MODE_SINGULAR_NOPS_FOR_30 \
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); \
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); \
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); \
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); \
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); \
    asm("nop"); asm("nop"); asm("nop"); asm("nop");

#define PROJ_SPECIAL_MODE_NUM_LEVELS 30
#define PROJ_SPECIAL_MODE_PWR_LEVEL_START 61
#define PROJ_SPECIAL_MODE_PWR_LEVEL_END_INCLUDING (PROJ_SPECIAL_MODE_PWR_LEVEL_START + PROJ_SPECIAL_MODE_NUM_LEVELS)
#define PROJ_SPECIAL_MODE_CALC_MAGIC asm("sublw " AUX_STRINGIFY(PROJ_SPECIAL_MODE_PWR_LEVEL_END_INCLUDING-1));

asm("global _apply_pwr_level__storage1");
void apply_pwr_level__storage1() __at(0x0008)
{
    asm("labe_apply_pwr_level__special_mode_enter:");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            PROJ_TEST_LIMIT_RETURN_2_TACTS // may be triggered as loop doesn't assume post check; // delayed by 2(+2)
            asm("movf _pwr_level,w"); // delayed by 3(+2)
            PROJ_SPECIAL_MODE_CALC_MAGIC // delayed by 4(+2)
            asm("addwf 2,f"); // PCL += offset: calculated delay with nops;  // delayed by 5(+2)
            PROJ_SPECIAL_MODE_SINGULAR_NOPS_FOR_30
            // delayed by 5(+2)+x; ON length = [7..37)
            PROJ_ADC_START() // delayed by 6(+2)+x; ON length = [8..38)
            AUX_DELAY_3 // balancing + align; // delayed by 9(+2)+x; ON length = [11..41)
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("btfsc " LOCATION__PIR1bits_TMR1IF);
            asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
            asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
            asm("movwf 5"); // = 1; delayed by 6 (OFF -> ON)
                asm("movf _pwr_level,w"); // delayed by 1(+2)
                PROJ_SPECIAL_MODE_CALC_MAGIC // delayed by 2(+2)
                asm("addwf 2,f"); // PCL += offset: calculated delay with nops;  // delayed by 3(+2)
                    PROJ_SPECIAL_MODE_SINGULAR_NOPS_FOR_30
                    // delayed by 3(+2)+x; ON length = [5..35)
                // adjust pwr level up or down
                asm("incf _pwr_level,f"); // pre-increment (will either stay incremented...)
                asm("movf _thr_hi,w"); // w = thr_hi
                asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi
                asm("movlw 2"); // doesn't affect status
                asm("skipc"); // c == no borrow
                asm("subwf _pwr_level,f"); // pwr_level = pwr_level - w(2) (...or will be decremented)
                // delayed by 9(+2)+x; ON length = [11..41)
            asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // prepare W before looping
            // fallthrough
    asm("movwf 5");
    // external entry point ----------------------------------------------------
    asm("labe_apply_pwr_level__enter:");
    // asm("bcf 3,5"); // ensure bank0 selected
    asm("movf _pwr_level,w");
    asm("addwf 2,f"); // PCL += pwr_level: tabled jump

    // table
    asm("incf _pwr_level,f"); // 0 * edge handling
    asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // min (0 pwm)
    //
    asm("goto labe_apply_pwr_level__d2");
    asm("goto labe_apply_pwr_level__d2d2__wait8");
    asm("goto labe_apply_pwr_level__d2d2");
    asm("goto labe_apply_pwr_level__d3");
    asm("goto labe_apply_pwr_level__d2d3");
    asm("goto labe_apply_pwr_level__d2d2d3");
    asm("goto labe_apply_pwr_level__d3v3_c2");
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c3");
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c4");
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c5");
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c6");
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c7");
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c8");
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c9");
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c10");
    asm("goto labe_apply_pwr_level__d3v3_c9__d4v4_c1"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c8__d4v4_c2"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c7__d4v4_c3"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c6__d4v4_c4"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c5__d4v4_c5"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c4__d4v4_c6"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c3__d4v4_c7"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c2__d4v4_c8"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c1__d4v4_c9"); // 3-4
    asm("goto labe_apply_pwr_level__d4v4_c10");
    asm("goto labe_apply_pwr_level__d4v4_c8__d5v4_c2"); // 4-5
    asm("goto labe_apply_pwr_level__d4v4_c6__d5v4_c4"); // 4-5
    asm("goto labe_apply_pwr_level__d4v4_c4__d5v4_c6"); // 4-5
    asm("goto labe_apply_pwr_level__d4v4_c2__d5v4_c8"); // 4-5
    asm("goto labe_apply_pwr_level__d5v4_c10");
    asm("goto labe_apply_pwr_level__d5v4_c8__d6v4_c2"); // 5-6
    asm("goto labe_apply_pwr_level__d5v4_c6__d6v4_c4"); // 5-6
    asm("goto labe_apply_pwr_level__d5v4_c4__d6v4_c6"); // 5-6
    asm("goto labe_apply_pwr_level__d5v4_c2__d6v4_c8"); // 5-6
    asm("goto labe_apply_pwr_level__d6v4_c10");
    asm("goto labe_apply_pwr_level__d6v4_c8__d7v4_c2"); // 6-7
    asm("goto labe_apply_pwr_level__d6v4_c6__d7v4_c4"); // 6-7
    asm("goto labe_apply_pwr_level__d6v4_c4__d7v4_c6"); // 6-7
    asm("goto labe_apply_pwr_level__d6v4_c2__d7v4_c8"); // 6-7
    asm("goto labe_apply_pwr_level__d7v4_c10");
    asm("goto labe_apply_pwr_level__d7v4_c8__d8v4_c2"); // 7-8
    asm("goto labe_apply_pwr_level__d7v4_c5__d8v4_c5"); // 7-8
    asm("goto labe_apply_pwr_level__d7v4_c2__d8v4_c8"); // 7-8
    asm("goto labe_apply_pwr_level__d8v4_c10");
    asm("goto labe_apply_pwr_level__d8v4_c7__d9v4_c3"); // 8-9
    asm("goto labe_apply_pwr_level__d8v4_c3__d9v4_c7"); // 8-9
    asm("goto labe_apply_pwr_level__d9v4_c10");
    asm("goto labe_apply_pwr_level__d9v4_c5__d10v4_c5"); // 9-10
    asm("goto labe_apply_pwr_level__d10v4_c10");
    asm("goto labe_apply_pwr_level__d10v4_c5__d11v4_c5"); // 10-11
    asm("goto labe_apply_pwr_level__d11v4_c10");
    //
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x1
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x2
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x3
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x4
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x5
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x6
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x7
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x8
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x9
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x10
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x11
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x12
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x13
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x14
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x15
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x16
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x17
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x18
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x19
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x20
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x21
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x22
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x23
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x24
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x25
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x26
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x27
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x28
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x29
    asm("goto labe_apply_pwr_level__special_mode_enter"); // x PROJ_SPECIAL_MODE_NUM_LEVELS
    //
    asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); // max
    // end table             max+1 * edge handling
    asm("decf _pwr_level,f");
    asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));

    // ----- duration 2
    asm("labe_apply_pwr_level__d2d2__wait8:");
        AUX_DELAY_8
    PROJ_DEFINE_BLOCK_2V3_NO_PRECHECK("d2d2")
    PROJ_DEFINE_BLOCK_2V3("d2")
    asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));

    // ----- durations 2+3 (mixed)
    PROJ_DEFINE_BLOCK_2V3_NO_PRECHECK("d2d2d3")
    PROJ_DEFINE_BLOCK_2V3("d2d3")
    PROJ_SKIP_IF_LIMIT_REACHED
    asm("labe_apply_pwr_level__d3:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5");
        asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));


    PROJ_DEFINE_BLOCK_COMBO_34("d3v3_c9", "d4v4_c1")
    PROJ_DEFINE_BLOCK_COMBO_34("d3v3_c8", "d4v4_c2")
    PROJ_DEFINE_BLOCK_COMBO_34("d3v3_c7", "d4v4_c3")
    PROJ_DEFINE_BLOCK_COMBO_34("d3v3_c6", "d4v4_c4")
    PROJ_DEFINE_BLOCK_COMBO_34("d3v3_c5", "d4v4_c5")
    PROJ_DEFINE_BLOCK_COMBO_34("d3v3_c4", "d4v4_c6")
    PROJ_DEFINE_BLOCK_COMBO_34("d3v3_c3", "d4v4_c7")
    PROJ_DEFINE_BLOCK_COMBO_34("d3v3_c2", "d4v4_c8")
    PROJ_DEFINE_BLOCK_COMBO_34("d3v3_c1", "d4v4_c9")

    PROJ_DEFINE_BLOCK_COMBO_45("d4v4_c8", 2)
    PROJ_DEFINE_BLOCK_COMBO_45("d4v4_c6", 4)
    PROJ_DEFINE_BLOCK_COMBO_45("d4v4_c4", 6)
    PROJ_DEFINE_BLOCK_COMBO_45("d4v4_c2", 8)

    asm("labe_apply_pwr_level__d5v4_c10:");
        asm("movlw " AUX_STRINGIFY(10 - 1));
        asm("movwf _tmp_ctr");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5");
        asm("goto labe_apply_pwr_level__d5v4_generic_enter");

    PROJ_DEFINE_BLOCK_COMBO_56(8, 2)
    PROJ_DEFINE_BLOCK_COMBO_56(6, 4)
    PROJ_DEFINE_BLOCK_COMBO_56(4, 6)
    PROJ_DEFINE_BLOCK_COMBO_56(2, 8)

    asm("labe_apply_pwr_level__d6v4_c10:");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5");
        asm("movlw " AUX_STRINGIFY(10 - 1));
        asm("goto labe_apply_pwr_level__d6v4_generic_enter_minus1");

    PROJ_DEFINE_BLOCK_COMBO_67(8, 2)
    PROJ_DEFINE_BLOCK_COMBO_67(6, 4)
    PROJ_DEFINE_BLOCK_COMBO_67(4, 6)
    PROJ_DEFINE_BLOCK_COMBO_67(2, 8)

    asm("labe_apply_pwr_level__d7v4_c10:");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5");
        asm("movlw " AUX_STRINGIFY(10 - 1));
        asm("goto labe_apply_pwr_level__d7v4_generic_enter_minus1");

    PROJ_DEFINE_BLOCK_COMBO_78(8, 2)
    PROJ_DEFINE_BLOCK_COMBO_78(5, 5)
    PROJ_DEFINE_BLOCK_COMBO_78(2, 8)

    asm("labe_apply_pwr_level__d8v4_c10:");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5");
        asm("movlw " AUX_STRINGIFY(10 - 1));
        asm("goto labe_apply_pwr_level__d8v4_generic_enter_minus1");

    PROJ_DEFINE_BLOCK_COMBO_89(7, 3)
    PROJ_DEFINE_BLOCK_COMBO_89(3, 7)

    asm("labe_apply_pwr_level__d9v4_c10:");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5");
        asm("movlw " AUX_STRINGIFY(10 - 1));
        asm("goto labe_apply_pwr_level__d9v4_generic_enter_minus1");

    PROJ_DEFINE_BLOCK_COMBO_910(5, 5)

    asm("labe_apply_pwr_level__d10v4_c10:");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5");
        asm("movlw " AUX_STRINGIFY(10 - 1));
        asm("goto labe_apply_pwr_level__d10v4_generic_enter_minus1");

    PROJ_DEFINE_BLOCK_COMBO_1011(5, 5)

    asm("labe_apply_pwr_level__d11v4_c10:");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5");
        asm("movlw " AUX_STRINGIFY(10 - 1));
        asm("goto labe_apply_pwr_level__d11v4_generic_enter_minus1");


    // ----- duration 3: (3/3)
    PROJ_DEFINE_BLOCK_3V3_NO_PRECHECK("d3v3_c10")
    PROJ_DEFINE_BLOCK_3V3("d3v3_c9")
    PROJ_DEFINE_BLOCK_3V3("d3v3_c8")
    PROJ_DEFINE_BLOCK_3V3("d3v3_c7")
    PROJ_DEFINE_BLOCK_3V3("d3v3_c6")
    PROJ_DEFINE_BLOCK_3V3("d3v3_c5")
    PROJ_DEFINE_BLOCK_3V3("d3v3_c4")
    asm("labe_apply_pwr_level__d3v3_c3__precheck:"); PROJ_SKIP_IF_LIMIT_REACHED
    asm("labe_apply_pwr_level__d3v3_c3:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("btfss _app_flags," AUX_STRINGIFY(PROJ_APP_FLAG_REDUCE_3_TO_2));
        asm("movwf 5"); // triggered since here if chosen length is 3; d3d3d3 sequence
        asm("movwf 5"); // triggered since here if chosen length is 2; d2d3d3 sequence
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5");
    asm("labe_apply_pwr_level__d3v3_c2__precheck:"); PROJ_SKIP_IF_LIMIT_REACHED
    asm("labe_apply_pwr_level__d3v3_c2:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5");
        asm("bcf _app_flags," AUX_STRINGIFY(PROJ_APP_FLAG_REDUCE_3_TO_2)); // clear flag
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5");
    asm("labe_apply_pwr_level__d3v3_c1__precheck:"); PROJ_SKIP_IF_LIMIT_REACHED
    asm("labe_apply_pwr_level__d3v3_c1:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5");
        asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));


    // ----- duration 4: 4/4 with dense limiting
    PROJ_DEFINE_BLOCK_4V4_NO_PRECHECK("d4v4_c10")
    PROJ_DEFINE_BLOCK_4V4("d4v4_c9")
    PROJ_DEFINE_BLOCK_4V4("d4v4_c8")
    PROJ_DEFINE_BLOCK_4V4("d4v4_c7")
    PROJ_DEFINE_BLOCK_4V4("d4v4_c6")
    PROJ_DEFINE_BLOCK_4V4("d4v4_c5")
    PROJ_DEFINE_BLOCK_4V4("d4v4_c4")
    PROJ_DEFINE_BLOCK_4V4("d4v4_c3")
    PROJ_DEFINE_BLOCK_4V4("d4v4_c2")
    asm("labe_apply_pwr_level__d4v4_c1__precheck:"); PROJ_SKIP_IF_LIMIT_REACHED
    asm("labe_apply_pwr_level__d4v4_c1:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("return"); // W is post-applied after call


    //asm("movlw " AUX_STRINGIFY(n_cycle_iters_minus_1));
    //asm("movwf _tmp_ctr");
    //asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
    //asm("movwf 5"); // = 1
    //asm("goto labe_apply_pwr_level__d5v4_generic_enter"); // 1's delay +2
    asm("labe_apply_pwr_level__d5v4_generic_enter:");
        AUX_DELAY_1 // 1's delay +3
        asm("labe_apply_pwr_level__d5v4_generic_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // 1's delay +4
        asm("movwf 5"); // = 0
        PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); // 0's delay +3
        asm("movwf 5"); // = 1 (if not reached limit)
        asm("decfsz _tmp_ctr"); // delay (+2 if quit / +3 if repeat)
        asm("goto labe_apply_pwr_level__d5v4_generic_pre0"); // reenters with delay +3
            asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // 1's delay +4

    //asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
    //asm("movwf 5"); // = 1
    //asm("movlw " AUX_STRINGIFY(n_cycle_iters_minus_1)); // 1's delay +1
    //asm("goto labe_apply_pwr_level__d6v4_generic_enter_minus1"); // 1's delay +3
    asm("labe_apply_pwr_level__d6v4_generic_enter_minus1:");
        asm("movwf _tmp_ctr"); // 1's delay +4
        asm("labe_apply_pwr_level__d6v4_generic_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // delay +5
        asm("movwf 5"); // = 0
        PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); // 0's delay +3
        asm("movwf 5"); // = 1 (if not reached limit)
        AUX_DELAY_1 // 1's delay +1
        asm("decfsz _tmp_ctr"); // delay (+3 if quit / +4 if repeat)
        asm("goto labe_apply_pwr_level__d6v4_generic_pre0"); // reenters with delay +4
            asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // 1's delay +5

    //asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
    //asm("movwf 5"); // = 1
    //asm("movlw " AUX_STRINGIFY(n_cycle_iters_minus_1)); // 1's delay +1
    //asm("goto labe_apply_pwr_level__d7v4_generic_enter_minus1"); // 1's delay +3
    asm("labe_apply_pwr_level__d7v4_generic_enter_minus1:");
        asm("movwf _tmp_ctr"); // 1's delay +4
        AUX_DELAY_1 // 1's delay +5
        asm("labe_apply_pwr_level__d7v4_generic_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // delay +6
        asm("movwf 5"); // = 0
        PROJ_TEST_LIMIT_RETURN_2_TACTS
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); // 0's delay +3
        asm("movwf 5"); // = 1 (if not reached limit)
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +2
        asm("decfsz _tmp_ctr"); // delay (+4 if quit / +5 if repeat)
        asm("goto labe_apply_pwr_level__d7v4_generic_pre0"); // reenters with delay +5
            asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // 1's delay +6

    //asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
    //asm("movwf 5"); // = 1
    //asm("movlw " AUX_STRINGIFY(n_cycle_iters_minus_1)); // 1's delay +1
    //asm("goto labe_apply_pwr_level__d8v4_generic_enter_minus1"); // 1's delay +3
    asm("labe_apply_pwr_level__d8v4_generic_enter_minus1:");
        asm("movwf _tmp_ctr"); // 1's delay +4
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +6
        asm("labe_apply_pwr_level__d8v4_generic_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // delay +7
        asm("movwf 5"); // = 0
        PROJ_TEST_LIMIT_RETURN_2_TACTS
        //AUX_DELAY_1
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); // 0's delay +3
        asm("movwf 5"); // = 1 (if not reached limit)
        AUX_DELAY_1
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +2
        asm("decfsz _tmp_ctr"); // delay (+4 if quit / +5 if repeat)
        asm("goto labe_apply_pwr_level__d8v4_generic_pre0"); // reenters with delay +5
            asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // 1's delay +6

    //asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
    //asm("movwf 5"); // = 1
    //asm("movlw " AUX_STRINGIFY(n_cycle_iters_minus_1)); // 1's delay +1
    //asm("goto labe_apply_pwr_level__d9v4_generic_enter_minus1"); // 1's delay +3
    asm("labe_apply_pwr_level__d9v4_generic_enter_minus1:");
        asm("movwf _tmp_ctr"); // 1's delay +4
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +6
        AUX_DELAY_1 // 1's delay +7
        asm("labe_apply_pwr_level__d9v4_generic_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // delay +8
        asm("movwf 5"); // = 0
        PROJ_TEST_LIMIT_RETURN_2_TACTS
        //AUX_DELAY_1
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); // 0's delay +3
        asm("movwf 5"); // = 1 (if not reached limit)
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +2
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +4
        asm("decfsz _tmp_ctr"); // delay (+6 if quit / +7 if repeat)
        asm("goto labe_apply_pwr_level__d9v4_generic_pre0"); // reenters with delay +7
            asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // 1's delay +8

    //asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
    //asm("movwf 5"); // = 1
    //asm("movlw " AUX_STRINGIFY(n_cycle_iters_minus_1)); // 1's delay +1
    //asm("goto labe_apply_pwr_level__d10v4_generic_enter_minus1"); // 1's delay +3
    asm("labe_apply_pwr_level__d10v4_generic_enter_minus1:");
        asm("movwf _tmp_ctr"); // 1's delay +4
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +6
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +8
        asm("labe_apply_pwr_level__d10v4_generic_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // delay +9
        asm("movwf 5"); // = 0
        PROJ_TEST_LIMIT_RETURN_2_TACTS
        //AUX_DELAY_1
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); // 0's delay +3
        asm("movwf 5"); // = 1 (if not reached limit)
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +2
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +4
        AUX_DELAY_1 // 1's delay +5
        asm("decfsz _tmp_ctr"); // delay (+7 if quit / +8 if repeat)
        asm("goto labe_apply_pwr_level__d10v4_generic_pre0"); // reenters with delay +8
            asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // 1's delay +9

    //asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
    //asm("movwf 5"); // = 1
    //asm("movlw " AUX_STRINGIFY(n_cycle_iters_minus_1)); // 1's delay +1
    //asm("goto labe_apply_pwr_level__d11v4_generic_enter_minus1"); // 1's delay +3
    asm("labe_apply_pwr_level__d11v4_generic_enter_minus1:");
        asm("movwf _tmp_ctr"); // 1's delay +4
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +6
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +8
        AUX_DELAY_1 // 1's delay +9
        asm("labe_apply_pwr_level__d11v4_generic_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // delay +10
        asm("movwf 5"); // = 0
        PROJ_TEST_LIMIT_RETURN_2_TACTS
        //AUX_DELAY_1
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); // 0's delay +3
        asm("movwf 5"); // = 1 (if not reached limit)
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +2
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +4
        PROJ_TEST_LIMIT_RETURN_2_TACTS // 1's delay +6
        asm("decfsz _tmp_ctr"); // delay (+8 if quit / +9 if repeat)
        asm("goto labe_apply_pwr_level__d11v4_generic_pre0"); // reenters with delay +9
            asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); // 1's delay +10
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
    asm("movlw 38h");
    asm("movwf _thr_min");

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
        asm("fcall _fast_RunADC_8_rising");
        asm("movf _adc_last_val,w");
        asm("movwf _last_raw_adc_val");
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
    asm("labe_main__pre_cycle:");
    PROJ_ADC_START()
        asm("bcf " LOCATION__PIR1bits_TMR1IF);
        PROJ_SKIP_IF_LIMIT_REACHED // aligned equally - delay 1 tact
        asm("goto labe_main__cycle_pre_apply_pwr_call"); // -- delay 2 tact if skipped
        // for first call it's ok to enter in the middle of cycle as pwr_level is 0
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

            asm("labe_main__cycle_pre_apply_pwr_call:");
            asm("fcall labe_apply_pwr_level__enter");
            asm("movwf 5"); // final post-apply W to GPIO
            // main part done


        asm("labe_main__cycle_pre_check_flags:");
        PROJ_ADC_START()
        asm("btfss " LOCATION__PIR1bits_TMR1IF);
        asm("goto labe_main__cycle");

        asm("decfsz _timer_multiplier,f");
        asm("goto labe_main__pre_cycle");
            asm("bsf _timer_multiplier," AUX_STRINGIFY(PROJ_TIMER_MULTIPLIER_AS_BIT));

        asm("fcall _fast_RunADC_8_rising");
        // save result
        asm("movf _adc_last_val,w");
        asm("xorwf _last_raw_adc_val,w");
        asm("skipnz");
        asm("goto labe_main__pre_cycle"); // no changes
        // different - needs recalculation

        PROJ_ADC_START() // keep discharging
        asm("movf _adc_last_val,w");
        asm("movwf _last_raw_adc_val");
        //
        PROJ_SKIP_IF_LIMIT_NOTREACHED
        asm("goto labe_main__cycle_proceed_calculate_thr");
            // note: LOCATION__PIR1bits_TMR1IF isn't cleared by this moment
            asm("fcall labe_apply_pwr_level__enter");
            asm("movwf 5"); // final post-apply W to GPIO
        //
        asm("labe_main__cycle_proceed_calculate_thr:");
        asm("movf _last_raw_adc_val,w"); // can be moved UP under if
        asm("fcall _fast_mul_8x8");
        PROJ_ADC_START() // keep discharging

        asm("movf _fast_mul_8x8_res_h,w");
        asm("movwf _thr_hi"); // thr_hi = last_raw_adc_val * g_thr / 256

        asm("subwf _thr_min,w"); // w = thr_min - thr_hi; if borrow - no action
        asm("skipnc"); // skip next if borrow
        asm("addwf _thr_hi,f"); // thr_hi += (thr_min - thr_hi)
        //
        asm("goto labe_main__pre_cycle");

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
