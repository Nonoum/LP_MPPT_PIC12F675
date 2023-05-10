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

NOTES for release 1.0.*.*:
    Features:
        1. "soft start" (even at very high input power the device will start outputting power increasing current step-by-step, which can take several to several-tens of milliseconds, depending on voltage levels);
        2. low self-consumption: prototypes consume 1..2 ma in static state (stable state with no load), having worse linear stabilizer it would be a little more;
        3. output limiting PIN (second prototype uses extra circuit with an optocoupler and a comparator to implement limiting logic)
            * should do pretty good balancing for current-limiting as well (although in current-limiting mode it's not tested at the moment);
            * so a dual-comparator chip (e.g. LM393P used in 2nd prototype) can serve for both output voltage and output current limiting - check out sample schematics;
        4. inverted PWM pin (fully synchronous with non-inverted, might be used with external fet drivers - e.g. for very-high power devices).
        5. dynamic PWM frequency (dynamic duty-ON, duty-OFF is almost always 4us) that in theory provides best possible (least present) coil whine as for low frequency MCU;

    PIC MCU notes: there are different packages with at least two working voltage ranges: some PICs have 3.6v limit (only 3.3v stabilizer can be used in this case);

    Efficiency notes:
        - second prototype has shown conversion efficieny (output RMS power / input RMS power) from around 80 to 97%;
        - efficieny depends on output voltage and power - lookup charts for buck DC-DC converters, typical efficiency of second prototype with 5v output - 85%, with 12v output - 92-94%;
        - in main testing 18v panel was used (with typical MPP ranging within 15 ... 18v);
        - no extra mosfet drivers are needed at least for mosfets with input capacitance up to around 3nf;

    Pulsation notes:
        - output voltage pulsations are highly reduced since initial 0.1 FW version;
        - pulsations are different on different power ranges, overall small, on mid and high power ranges the device can drive inductor in continuous mode and minimize pulsations even further;
        - capacitance can be smaller or bigger than in prototypes described above, suggestions for buffer and output caps are:
            -- minimum: 1000uf (buffer); 1000uf per 1A output current (output cap);
            -- recommended: 2000uf (buffer), 2000uf per 1A output current (output cap);
        - pulsations can be induced by MPP overshoot, which can significantly reduce input current and thus generated power - see "PROJ_CONST_THR" notes and "Input diode Vf" notes;
        - pulsations can be induced by partial shading (when shading occurs somewhere inseries);

    PWM extra notes:
        - code does manual PWM with duty-ON (open output mosfet) lengths starting from 2us;
        - for mosfet driving power considerations expect average switching (ON->OFF or OFF->ON) interval of 3us (worst case), single PIN is rated for 25ma, dual pin - 50ma;
        - currently (FW 1.0) maximum PWM duty-ON length is 33us (+fully open state that can lead this to infinity).
        - duty-OFF time is up to 4us (in vast majority of modes - it's 4us), with exceptions that:
            * obviously during OCV measurings (once a second) there's no power pushed to output;
            * in lower modes (duty-ON <= 5us) there are extra gaps between outputting power, mostly around 20us per such gap (between last ON->OFF in sequence and next OFF->ON);
            * starting from duty-ON = 5us (some of these 5us modes and all further) there's special chained PWM implementation with no disruptions except for OCV measuring once a second and one more processing of timer once a second that results in extra gap of some ten us;

    OCV measuring extra notes:
        - best possible OCV measuring disruption (gap between duty-OFF and next duty-ON) is 52us (0.052ms), second best is 75us, others are worse,
            these two have special code and specifically optimized for targeting high (and mid) input currents to minimize output capacitor requirements.

    Powerbank and some output voltage limiting notes:
        - powerbank 1 (QC supported; device used in most of testing during almost all of development):
            - tested powerbank can be tricked to trigger QC mode and rise voltage - when it's connected it "requests" higher voltage while consuming no substantial load
            and the actual voltage can be rising at the moment so powerbank considers that everything is according to plan and accepts it;
            - if voltage manages to rise higher than powerbank's limit - it will shut down and won't charge (*or powerbank will die - happened with one simple model when 12v was applied during testing);
            - plain buck converter (when added after output) is apparenty fully "opened" and acts as a minor resistance (inductor+mosfet) with it's input and output voltages being almost equal;
        - powerbank 2 (no QC no PD): pay attention at powerbank's capabilities - if it doesn't support QC/PD then it will likely die from higher voltages (not just shut-off).
        - powerbank 3 (PD supported): denies voltages higher than ~5.3v and simply doesn't charge, often charges poorly with around 5v provided (doesn't consume all current available);

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
#define FW_VER_MAJOR 1
#define FW_VER_MINOR 1
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
#define AUX_DELAY_2 asm("nop2"); // compiler knows what to do with nop2; a jump to next command
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

#define AUX_1CMD_DELAY_2 AUX_DELAY_2
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
// TODO different configurations if needed
#endif

#define PROJ_ADC_ADCS 0b000 // conversion time ~ 5.5 tacts


uint8_t app_flags;
#define PROJ_APP_FLAG_REDUCE_3_TO_2 0
uint8_t counter_45;
uint8_t jmp_dist;

#define PROJ_TIMER_MULTIPLIER_AS_BIT 1 // multiplier is a degree of 2
uint8_t timer_multiplier = (1 << PROJ_TIMER_MULTIPLIER_AS_BIT);

// vars for raw voltage measuring
uint8_t adc_last_val;
uint8_t adc_extra_val;
uint8_t adc_ocv_prev;


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
#define LOCATION__EECON1_RD "9Ch,0"


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



void tmr_proc() {
    PROJ_SET_GPIO_CAPON_PWMOFF
    asm("bcf " LOCATION__PIR1bits_TMR1IF);
    asm("decfsz _timer_multiplier,f");
        asm("return");
    ////// measure OCV. for high currents - as fast as possible
    PROJ_SET_GPIO_CAPOFF_PWMOFF
    PROJ_ADC_START()
        AUX_DELAY_7
    // always waste first measured sample
    PROJ_ADC_START()
        asm("bsf _timer_multiplier," AUX_STRINGIFY(PROJ_TIMER_MULTIPLIER_AS_BIT)); // recover timer
        asm("movlw 16");
        asm("movwf _tmp_ctr"); // prepare cnt for cycle
        asm("movlw 2");
        asm("movwf _tmp_ctr2"); // prepare equal sequence for best cycle case
        asm("movf _adc_last_val,w");
        asm("movwf _adc_ocv_prev");
    PROJ_ADC_START() // sample #2 rdy
        asm("movf " LOCATION__ADCVAL ",w"); // sample #2
        asm("movwf _adc_last_val");
        asm("movwf _adc_extra_val");
        AUX_DELAY_4
    PROJ_ADC_START() // sample #3 rdy
        asm("movf " LOCATION__ADCVAL ",w"); // sample #3
        asm("xorwf _adc_last_val,f"); // compare #3 with #2
        asm("movwf _adc_last_val");
        asm("skipz");
            asm("goto labe_tmr_proc__mismatch_3_2");
        asm("movf _adc_ocv_prev,w");
        asm("xorwf _adc_last_val,f"); // compare #3(==#2) with previous OCV
    PROJ_ADC_START() // sample #4 rdy
        asm("skipnz");
            asm("goto labe_tmr_proc__quit_fastest"); // ..45 tacts between pwm_off and returning to called code (+7 tacts more to pwm_on. best case)
        asm("xorwf _adc_last_val,f"); // undo xor - recover correct value, we still need it
        asm("movf " LOCATION__ADCVAL ",w"); // sample #4
        asm("xorwf _adc_last_val,f"); // compare #4 with #3
        asm("movwf _adc_last_val");
        asm("skipz");
            asm("goto labe_tmr_proc__mismatch_4_3");
        // #4==#3
    PROJ_ADC_START() // sample #5 rdy
        asm("xorwf _adc_extra_val,f"); // compare #4(==#3) with #2
        asm("skipnz");
            asm("goto labe_tmr_proc__wrap_up"); // ..65 tacts between pwm_off and returning to called code (+7 tacts more to pwm_on) [considering tabled multiply]
        asm("movf " LOCATION__ADCVAL ",w"); // sample #5
        asm("xorwf _adc_last_val,f"); // compare #5 with #4
        asm("movwf _adc_last_val");
        asm("skipz");
            asm("clrf _tmp_ctr2");
        asm("incf _tmp_ctr2"); // 1 or 3

    // cycle (low current / slow measuring)
    asm("labe_tmr_proc__cycle:");
        PROJ_ADC_START()
        asm("movf " LOCATION__ADCVAL ",w");
        asm("xorwf _adc_last_val,f"); // compare
        asm("movwf _adc_last_val"); // update last value / doesn't affect flags
        asm("skipz");
            asm("clrf _tmp_ctr2"); // clear amount of equal values if mismatch
        asm("incf _tmp_ctr2"); // ++
        asm("btfsc _tmp_ctr2,2"); // test if >= 4
            asm("goto labe_tmr_proc__wrap_up");
        asm("decfsz _tmp_ctr");
            asm("goto labe_tmr_proc__cycle");

    // cycle is over
    asm("movf " LOCATION__ADCVAL ",w");
    // avg last 2 samples just in case
    asm("addwf _adc_last_val,f");
    asm("rrf _adc_last_val,f"); // adc_last_val = avg of last 2 adc samples
    //
    asm("labe_tmr_proc__wrap_up:");
        PROJ_SET_GPIO_CAPON_PWMOFF
        PROJ_ADC_START() // run adc for 1 sample after measuring and ignore that sample (recover working voltage)

        asm("btfss _adc_last_val,7"); // test MSB
            asm("goto labe_tmr_proc__div2_tabled");
        // MSB=1; straight tabled lookup
        asm("bsf 3,5"); // bank1 ------------
        asm("movf _adc_last_val,w");
        asm("movwf " LOCATION__EEADDR);
        asm("bsf " LOCATION__EECON1_RD);
        asm("movf " LOCATION__EEDATA ",w");
        asm("bcf 3,5"); // back to bank0 ----
        asm("movwf _thr_hi");
        asm("return");

    asm("labe_tmr_proc__quit_fastest:");
        PROJ_SET_GPIO_CAPON_PWMOFF
        // previous/current ADC run partly overlaps with CAPOFF->CAPON transition
        // at the moment nevermind of not too settled voltage on ADC sample, in worst case we will output slightly more power at single next iteration
        asm("return");

    asm("labe_tmr_proc__mismatch_3_2:"); // TODO may be improved at least this particular one
    asm("labe_tmr_proc__mismatch_4_3:"); // TODO improve
        asm("clrf _tmp_ctr2"); // clear amount of equal values if mismatch
        asm("goto labe_tmr_proc__cycle");

    asm("labe_tmr_proc__div2_tabled:"); // MSB is zero
        asm("bsf 3,5"); // bank1 ------------
        asm("rlf _adc_last_val,w"); // w = adc * 2; C-flag = 0
        asm("movwf " LOCATION__EEADDR);
        asm("bsf " LOCATION__EECON1_RD);
        asm("rrf " LOCATION__EEDATA ",w"); // w = mul result / 2
        asm("bcf 3,5"); // back to bank0 ----
        asm("movwf _thr_hi");
        // validate minimum
        asm("subwf _thr_min,w"); // w(thr_hi) = thr_min - w ; if borrow - no action
        asm("skipnc"); // skip next if borrow
        asm("addwf _thr_hi,f"); // thr_hi += (thr_min - thr_hi)
        // 'return' is auto-generated
}

uint8_t thr_min;
uint8_t thr_hi; // high level voltage threshold
uint8_t pwr_level = 1; // active power level. 0,1 = disconnected
uint8_t tmp_ctr; // temporary
uint8_t tmp_ctr2; // temporary


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



// counter_45 controls 4-vs-5 duty-ON length (keep 4ON while positive, post-increment)
// last iteration is only with length 4
#define PROJ_DEFINE_BLOCK_4V4_NO_PRECHECK(lbl_part_name) \
    asm("labe_apply_pwr_level__" lbl_part_name ":"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON)); \
        asm("movwf 5"); /* = 1 */ \
            asm("btfsc _counter_45,7"); \
                AUX_1CMD_DELAY_2 \
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF)); \
        asm("movwf 5"); /* = 0 */ \
        asm("incf _counter_45");

#define PROJ_DEFINE_BLOCK_4V4(lbl_part_name) \
    asm("labe_apply_pwr_level__" lbl_part_name "__precheck:"); PROJ_SKIP_IF_LIMIT_REACHED \
    PROJ_DEFINE_BLOCK_4V4_NO_PRECHECK(lbl_part_name)



#define PROJ_DEFINE_BLOCK_COMBO_34(part1, part2) \
    asm("labe_apply_pwr_level__" part1 "__" part2 ":"); \
        asm("fcall labe_apply_pwr_level__" part1); \
        asm("movwf 5"); \
        asm("goto labe_apply_pwr_level__" part2 "__precheck");



asm("global _apply_pwr_level__storage1");
void apply_pwr_level__storage1() __at(0x0001)
{
    // code using tables - requires locating within a 256-byte page
            asm("labe_apply_pwr_level__gen_pre_jump_addwf_3:"); asm("movlw 3");
            asm("labe_apply_pwr_level__gen_pre_jump_addwf:");   asm("addwf _jmp_dist,f");
            asm("labe_apply_pwr_level__gen_pre_jump:");         asm("movf _jmp_dist,w");
            asm("addwf 2,f"); // PCL += jmp_dist: tabled jump
                // 6 tacts has passed since pwm ON
                //------------- table
                asm("goto labe_apply_pwr_level__gen_full_open");
                AUX_1CMD_DELAY_2 AUX_1CMD_DELAY_2 AUX_1CMD_DELAY_2 AUX_1CMD_DELAY_2 AUX_1CMD_DELAY_2
                asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
                asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
                asm("goto labe_apply_pwr_level__gen_continue");
                // fall to previous modes
                asm("goto labe_apply_pwr_level__at_c12_delay2end");   // complete(and proceed) with duty = 12 tacts
                asm("goto labe_apply_pwr_level__at_c12_pre_tm_pre0"); // complete(and proceed) with duty = 11 tacts
                asm("goto labe_apply_pwr_level__at_c10_end");         // complete(and proceed) with duty = 10 tacts
                //------------- end table
                // table always exited by jump (with at least +2 tacts delay), so at least 8 tacts since pwm ON
            asm("labe_apply_pwr_level__gen_continue:"); asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            PROJ_ADC_START() // delayed by 11+
        asm("labe_apply_pwr_level__gen_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__gen_fall_m1"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("movf _jmp_dist,w");
            asm("addwf 2,f"); // PCL += jmp_dist: tabled jump
                // 2 tacts has passed since pwm ON
                //------------- table
                asm("nop"); // edge, but no need to handle in this table (will never jump here)
                AUX_1CMD_DELAY_2 AUX_1CMD_DELAY_2 AUX_1CMD_DELAY_2 AUX_1CMD_DELAY_2 AUX_1CMD_DELAY_2
                asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
                asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
                //------------- end table
                // at least 2 tacts delay
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            // 2-or-3 delay for faster correction
            asm("skipnc");
                AUX_DELAY_2 // if increasing power - 2 tact delay here, otherwise - 1 tact
            //
            asm("movlw 255"); // increasing power by default
            asm("skipc"); // c == no borrow; skip next if need increasing power
                asm("movlw 1"); // need decreasing power
            asm("addwf _jmp_dist,f"); // delayed by 11+
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        // loop gen mode
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__gen_fall_m1"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_2 // TODO this one command can be saved by adding extra label to such a jump
            asm("goto labe_apply_pwr_level__gen_pre_jump");
    // -------------------------------------------------------------------------

    // external entry point ----------------------------------------------------
    // code using tables - requires locating within a 256-byte page
    asm("labe_apply_pwr_level__enter:");
    asm("clrf _counter_45");
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
    asm("goto labe_apply_pwr_level__d3v3_c3"); // level 10
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c4");
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c5");
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c6");
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c7");
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c8"); // level 20
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c9");
    PROJ_1CMD_SET_REDUCE_3_TO_2()
    asm("goto labe_apply_pwr_level__d3v3_c10"); // level 24
    asm("goto labe_apply_pwr_level__d3v3_c9__d4v4_c1"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c8__d4v4_c2"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c7__d4v4_c3"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c6__d4v4_c4"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c5__d4v4_c5"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c4__d4v4_c6"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c3__d4v4_c7"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c2__d4v4_c8"); // 3-4
    asm("goto labe_apply_pwr_level__d3v3_c1__d4v4_c9"); // 3-4
    asm("goto labe_apply_pwr_level__d4v4_c10"); // level 34
#define PROJ_LVL_45_MIN 35
#define PROJ_LVL_45_4CNT_MAX 9
    asm("goto labe_apply_pwr_level__d4v4_mixed_d5v4"); // 4x9-5x1 [4x8-5x1-4x1] // level 35
    asm("goto labe_apply_pwr_level__d4v4_mixed_d5v4"); // 4x8-5x2
    asm("goto labe_apply_pwr_level__d4v4_mixed_d5v4"); // 4x7-5x3
    asm("goto labe_apply_pwr_level__d4v4_mixed_d5v4"); // 4x6-5x4
    asm("goto labe_apply_pwr_level__d4v4_mixed_d5v4"); // 4x5-5x5
    asm("goto labe_apply_pwr_level__d4v4_mixed_d5v4"); // 4x4-5x6
    asm("goto labe_apply_pwr_level__d4v4_mixed_d5v4"); // 4x3-5x7
    asm("goto labe_apply_pwr_level__d4v4_mixed_d5v4"); // 4x2-5x8 // level 42
    //
    asm("goto labe_apply_pwr_level__at_c0"); // level 43
    asm("decf _pwr_level,f"); // handle max+1 just in case
    asm("goto labe_apply_pwr_level__at_c0");


    asm("labe_apply_pwr_level__d4v4_mixed_d5v4:");
        asm("movf _pwr_level,w");
        asm("addlw " AUX_STRINGIFY(128 - PROJ_LVL_45_MIN - PROJ_LVL_45_4CNT_MAX + 1)); // (+1):last iteration is only with length 4
        asm("movwf _counter_45");
        asm("goto labe_apply_pwr_level__d4v4_c10");


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


    asm("labe_apply_pwr_level__pre0_pre_decrement:");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        asm("labe_apply_pwr_level__pre_decrement:");
        asm("decf _pwr_level,f");
        PROJ_SKIP_IF_LIMIT_REACHED
            asm("goto labe_apply_pwr_level__enter");
        asm("retlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));

    // chained part ----------------------------
    asm("labe_apply_pwr_level__at_c0:"); // 5/4; 5/4; [5/4] cycled -------------
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__pre_decrement"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            PROJ_ADC_START()
        asm("labe_apply_pwr_level__at_c0_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__pre_decrement"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            AUX_DELAY_1
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__pre_decrement"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("skipc"); // c == no borrow; skip next if need increasing
            asm("goto labe_apply_pwr_level__pre0_pre_decrement"); // need decreasing
            AUX_DELAY_1 // else - continue
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0

    asm("labe_apply_pwr_level__at_c1:"); // 5/4; 5/4; [5/4] cycled (one more) --
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c0"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            asm("labe_apply_pwr_level__at_c1_pre_tm_pre0:"); PROJ_ADC_START()
        asm("labe_apply_pwr_level__at_c1_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c0"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            AUX_DELAY_1
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c0"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("skipc"); // c == no borrow; skip next if need increasing
            asm("goto labe_apply_pwr_level__at_c0_pre0"); // need decreasing
            AUX_DELAY_1 // else - continue
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0

    asm("labe_apply_pwr_level__at_c2:"); // 6/4; 5/4; [5/4] cycled -------------
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c1"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_1
            asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            asm("labe_apply_pwr_level__at_c2_pre_tm_pre0:"); PROJ_ADC_START()
        asm("labe_apply_pwr_level__at_c2_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c1"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            AUX_DELAY_1
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c1"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("skipc"); // c == no borrow; skip next if need increasing
            asm("goto labe_apply_pwr_level__fall_to_c1"); // need decreasing
            AUX_DELAY_1 // else - continue
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0

    asm("labe_apply_pwr_level__at_c3:"); // 6/4; 6/4; [5/4] cycled -------------
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c2"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_1
            asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            asm("labe_apply_pwr_level__at_c3_pre_tm_pre0:"); PROJ_ADC_START()
        asm("labe_apply_pwr_level__at_c3_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c2"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            AUX_DELAY_2
            //---------
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c2"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("skipc"); // c == no borrow; skip next if need increasing
            asm("goto labe_apply_pwr_level__fall_to_c2"); // need decreasing
            AUX_DELAY_1 // else - continue
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0

    asm("labe_apply_pwr_level__at_c4:"); // 6/4; 6/4; [6/4] cycled -------------
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c3"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_1
            asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            asm("labe_apply_pwr_level__at_c4_pre_tm_pre0:"); PROJ_ADC_START()
        asm("labe_apply_pwr_level__at_c4_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c3"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            AUX_DELAY_2
            //---------
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c3"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("skipc"); // c == no borrow; skip next if need increasing
            asm("goto labe_apply_pwr_level__at_c3_pre_tm_pre0"); // need decreasing
            AUX_DELAY_2 // else - continue
            //---------
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0

    asm("labe_apply_pwr_level__at_c5:"); // 7/4; 6/4; [6/4] cycled -------------
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c4"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_2
            //---------
            asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            asm("labe_apply_pwr_level__at_c5_pre_tm_pre0:"); PROJ_ADC_START()
        asm("labe_apply_pwr_level__at_c5_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c4"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            AUX_DELAY_2
            //---------
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c4"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("skipc"); // c == no borrow; skip next if need increasing
            asm("goto labe_apply_pwr_level__at_c4_pre_tm_pre0"); // need decreasing
            AUX_DELAY_2 // else - continue
            //---------
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0

    asm("labe_apply_pwr_level__at_c6:"); // 7/4; 7/4; cycled -------------------
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c5"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_2
            //---------
            asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            asm("labe_apply_pwr_level__at_c6_pre_tm_pre0:"); PROJ_ADC_START()
        asm("labe_apply_pwr_level__at_c6_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c5"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            asm("skipc"); // c == no borrow; skip next if need increasing
            asm("goto labe_apply_pwr_level__fall_to_c5"); // need decreasing
            AUX_DELAY_1
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0

    asm("labe_apply_pwr_level__at_c7:"); // 8/4; 7/4; cycled -------------------
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c6"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_3
            //---------
            //---------
            asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            asm("labe_apply_pwr_level__at_c7_pre_tm_pre0:"); PROJ_ADC_START()
        asm("labe_apply_pwr_level__at_c7_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c6"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            asm("skipc"); // c == no borrow; skip next if need increasing
            asm("goto labe_apply_pwr_level__fall_to_c6"); // need decreasing
            AUX_DELAY_1
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0

    asm("labe_apply_pwr_level__at_c8:"); // 8/4; 8/4; cycled -------------------
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c7"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_3
            //---------
            //---------
            asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            asm("labe_apply_pwr_level__at_c8_pre_tm_pre0:"); PROJ_ADC_START()
        asm("labe_apply_pwr_level__at_c8_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c7"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            asm("skipc"); // c == no borrow; skip next if need increasing
            asm("goto labe_apply_pwr_level__at_c7_pre_tm_pre0"); // need decreasing
            AUX_DELAY_2
            //---------
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0

    asm("labe_apply_pwr_level__at_c9:"); // 9/4; 8/4; cycled -------------------
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c8"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_4
            //---------
            //---------
            //---------
            asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            asm("labe_apply_pwr_level__at_c9_pre_tm_pre0:"); PROJ_ADC_START()
        asm("labe_apply_pwr_level__at_c9_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c8"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            asm("skipc"); // c == no borrow; skip next if need increasing
            asm("goto labe_apply_pwr_level__at_c8_pre_tm_pre0"); // need decreasing
            AUX_DELAY_2
            //---------
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0

    asm("labe_apply_pwr_level__at_c10:"); // 9/4; 9/4; cycled -------------------
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c9"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_4
            //---------
            //---------
            //---------
            asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            asm("labe_apply_pwr_level__at_c10_pre_tm_pre0:"); PROJ_ADC_START()
        asm("labe_apply_pwr_level__at_c10_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c9"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_1
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            asm("skipc"); // c == no borrow; skip next if need increasing
            asm("goto labe_apply_pwr_level__at_c9_pre_tm_pre0"); // need decreasing
            AUX_DELAY_2
            //---------
        asm("labe_apply_pwr_level__at_c10_end:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0

    asm("labe_apply_pwr_level__at_c11:"); // 10/4; 10/4; cycled -----------------
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c10"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_5
            //---------
            //---------
            //---------
            //---------
            asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            asm("labe_apply_pwr_level__at_c11_pre_tm_pre0:"); PROJ_ADC_START()
        asm("labe_apply_pwr_level__at_c11_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c10"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_2
            //---------
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            asm("skipc"); // c == no borrow; skip next if need increasing
            asm("goto labe_apply_pwr_level__at_c10_pre_tm_pre0"); // need decreasing
            AUX_DELAY_2
            //---------
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0

    asm("labe_apply_pwr_level__at_c12:"); // 11/4; 11/4; cycled -----------------
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c11"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_6
            //---------
            //---------
            //---------
            //---------
            //---------
            asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            asm("labe_apply_pwr_level__at_c12_pre_tm_pre0:"); PROJ_ADC_START()
        asm("labe_apply_pwr_level__at_c12_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c11"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_3
            //---------
            //---------
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            asm("skipc"); // c == no borrow; skip next if need increasing
            asm("goto labe_apply_pwr_level__at_c11_pre_tm_pre0"); // need decreasing
            asm("labe_apply_pwr_level__at_c12_delay2end:"); AUX_DELAY_2
            //---------
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0

    asm("labe_apply_pwr_level__at_c13:"); // 12/4; 12/4; cycled -----------------
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c12"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_7
            //---------
            //---------
            //---------
            //---------
            //---------
            //---------
            asm("btfsc " LOCATION__PIR1bits_TMR1IF);
                asm("fcall _tmr_proc");
            asm("labe_apply_pwr_level__at_c13_pre_tm_pre0:"); PROJ_ADC_START()
        asm("labe_apply_pwr_level__at_c13_pre0:"); asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c12"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_4
            //---------
            //---------
            //---------
            asm("movf _thr_hi,w");               // w = thr_hi
            asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
            asm("skipc"); // c == no borrow; skip next if need increasing
            asm("goto labe_apply_pwr_level__at_c12_pre_tm_pre0"); // need decreasing
            AUX_DELAY_2
            //---------
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0

    asm("labe_apply_pwr_level__gen:"); // 13+/4; 13+/4; cycled ------------------
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c13"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("movlw 16"); // set up starting duty = minimum
            asm("movwf _jmp_dist");
            asm("goto labe_apply_pwr_level__gen_pre_jump");
            // continued on zero page (first 256 bytes of code) to use table jumps (PCLATH always refers zero page)


    // gen helpers ----------------------------
    asm("labe_apply_pwr_level__gen_full_open:"); // entered with pwm ON
        asm("incf _jmp_dist,f"); // preliminary recover previous gen mode (=1)
        asm("labe_apply_pwr_level__gen_full_open_loop:"); PROJ_ADC_START()
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__gen_pre0"); // fall to previous
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__gen_pre0"); // fall to previous
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__gen_pre0"); // fall to previous
        asm("movf _thr_hi,w");               // w = thr_hi
        asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi; setup C-flag state
        asm("skipc"); // c == no borrow; skip next if need increasing power
            asm("goto labe_apply_pwr_level__gen_pre0"); // need decreasing power : fall to previous
        //
        asm("btfss " LOCATION__PIR1bits_TMR1IF);
            asm("goto labe_apply_pwr_level__gen_full_open_loop");
        asm("fcall _tmr_proc");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
        asm("goto labe_apply_pwr_level__gen_full_open_loop");


    asm("labe_apply_pwr_level__gen_fall_m1:");
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__gen_fall_m2"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
        asm("movlw 1"); // decrease power by 1
        asm("goto labe_apply_pwr_level__gen_pre_jump_addwf");

    asm("labe_apply_pwr_level__gen_fall_m2:");
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__gen_fall_m3"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
        asm("movlw 2"); // decrease power by 2
        asm("goto labe_apply_pwr_level__gen_pre_jump_addwf");

    asm("labe_apply_pwr_level__gen_fall_m3:");
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__gen_fall_m4_d1"); // limited
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
        asm("goto labe_apply_pwr_level__gen_pre_jump_addwf_3"); // decrease power by 3

    //... then just stick to 3 level reductions at time to support maximum correct timings
    asm("labe_apply_pwr_level__gen_fall_m4_d1:");
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__gen_fall_m4_d2"); // limited
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
        asm("goto labe_apply_pwr_level__gen_pre_jump_addwf_3"); // decrease power by 3

    asm("labe_apply_pwr_level__gen_fall_m4_d2:");
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__gen_fall_m4_d3"); // limited
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
        asm("goto labe_apply_pwr_level__gen_pre_jump_addwf_3"); // decrease power by 3

    asm("labe_apply_pwr_level__gen_fall_m4_d3:");
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__gen_fall_m4_d4"); // limited
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
        asm("goto labe_apply_pwr_level__gen_pre_jump_addwf_3"); // decrease power by 3

    asm("labe_apply_pwr_level__gen_fall_m4_d4:");
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__gen_fall_m4_d5"); // limited
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
        asm("goto labe_apply_pwr_level__gen_pre_jump_addwf_3"); // decrease power by 3

    asm("labe_apply_pwr_level__gen_fall_m4_d5:");
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c13"); // still limited - quit to last chained mode
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
        asm("goto labe_apply_pwr_level__gen_pre_jump_addwf_3"); // decrease power by 3


    // chained helpers ----------------------
    asm("labe_apply_pwr_level__fall_to_c1:");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c0"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("goto labe_apply_pwr_level__at_c1_pre_tm_pre0");

    asm("labe_apply_pwr_level__fall_to_c2:");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c1"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            asm("goto labe_apply_pwr_level__at_c2_pre_tm_pre0");

    //
    asm("labe_apply_pwr_level__fall_to_c5:");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c4"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_2 // TODO this one command can be saved by adding extra label
            //---------
            asm("goto labe_apply_pwr_level__at_c5_pre_tm_pre0");

    asm("labe_apply_pwr_level__fall_to_c6:");
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMOFF));
        asm("movwf 5"); // = 0
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_apply_pwr_level__at_c5"); // limited - fall to previous
        asm("movlw " AUX_STRINGIFY(PROJ_STATE_GPIO_CAPON_PWMON));
        asm("movwf 5"); // = 1
            AUX_DELAY_2 // TODO this one command can be saved by adding extra label
            //---------
            asm("goto labe_apply_pwr_level__at_c6_pre_tm_pre0");
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

    adc_last_val = 0;

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
        asm("incf _timer_multiplier");
        asm("fcall _tmr_proc");
    }

    asm("labe_main__cycle:");
        asm("incf _pwr_level,f"); // pre-increment (will either stay incremented...)
        asm("movf _thr_hi,w"); // w = thr_hi
        // ADC has finished by this exact point
        asm("subwf " LOCATION__ADCVAL ",w"); // w = adcval - thr_hi
        asm("movlw 2"); // doesn't affect status
        asm("skipc"); // c == no borrow
        asm("subwf _pwr_level,f"); // pwr_level = pwr_level - w(2) (...or will be decremented)

        asm("labe_main__cycle_pre_apply_pwr_call:");
        asm("fcall labe_apply_pwr_level__enter");
        asm("movwf 5"); // final post-apply W to GPIO

        PROJ_ADC_START()
        PROJ_SKIP_IF_LIMIT_NOTREACHED
            asm("goto labe_main__limited_maybe_decrement");

        asm("btfss " LOCATION__PIR1bits_TMR1IF);
            asm("goto labe_main__cycle");
        asm("fcall _tmr_proc");
        PROJ_ADC_START()
        AUX_DELAY_3 // align adc readiness
        asm("goto labe_main__cycle");

        // ------------------------------------------
        asm("labe_main__limited_maybe_decrement:");
            asm("movlw 3"); // minimum non-null power level + 1
            asm("subwf _pwr_level,w"); // w = pwr_level - 3
            asm("skipnc"); // nc == borrow (skip if pwr_level < 3)
                asm("decf _pwr_level,f");
            PROJ_SKIP_IF_LIMIT_REACHED
                asm("goto labe_main__delayed_apply_pwr");
            // limit reached for too long - keep decrementing power level till minimum
            asm("goto labe_main__limited_maybe_decrement");

        asm("labe_main__delayed_apply_pwr:");
            AUX_DELAY_6 // align timings with non-limited flow
            asm("goto labe_main__cycle_pre_apply_pwr_call");
} //


// optimal voltage is around 80-81% for evenly saturated solar panel, it doesn't depend on radiation level nor on temperature.
// define mapping of thr muplitplication in eeprom (128..255)
#define TMP_THR_H 0x80
#define TMP_C(low) (((TMP_THR_H | low) * PROJ_CONST_THR) >> 8)

__EEPROM_DATA(TMP_C(0),TMP_C(1),TMP_C(2),TMP_C(3),TMP_C(4),TMP_C(5),TMP_C(6),TMP_C(7));
__EEPROM_DATA(TMP_C(8),TMP_C(9),TMP_C(10),TMP_C(11),TMP_C(12),TMP_C(13),TMP_C(14),TMP_C(15));
#undef TMP_THR_H
#define TMP_THR_H 0x90
__EEPROM_DATA(TMP_C(0),TMP_C(1),TMP_C(2),TMP_C(3),TMP_C(4),TMP_C(5),TMP_C(6),TMP_C(7));
__EEPROM_DATA(TMP_C(8),TMP_C(9),TMP_C(10),TMP_C(11),TMP_C(12),TMP_C(13),TMP_C(14),TMP_C(15));
#undef TMP_THR_H
#define TMP_THR_H 0xA0
__EEPROM_DATA(TMP_C(0),TMP_C(1),TMP_C(2),TMP_C(3),TMP_C(4),TMP_C(5),TMP_C(6),TMP_C(7));
__EEPROM_DATA(TMP_C(8),TMP_C(9),TMP_C(10),TMP_C(11),TMP_C(12),TMP_C(13),TMP_C(14),TMP_C(15));
#undef TMP_THR_H
#define TMP_THR_H 0xB0
__EEPROM_DATA(TMP_C(0),TMP_C(1),TMP_C(2),TMP_C(3),TMP_C(4),TMP_C(5),TMP_C(6),TMP_C(7));
__EEPROM_DATA(TMP_C(8),TMP_C(9),TMP_C(10),TMP_C(11),TMP_C(12),TMP_C(13),TMP_C(14),TMP_C(15));
#undef TMP_THR_H
#define TMP_THR_H 0xC0
__EEPROM_DATA(TMP_C(0),TMP_C(1),TMP_C(2),TMP_C(3),TMP_C(4),TMP_C(5),TMP_C(6),TMP_C(7));
__EEPROM_DATA(TMP_C(8),TMP_C(9),TMP_C(10),TMP_C(11),TMP_C(12),TMP_C(13),TMP_C(14),TMP_C(15));
#undef TMP_THR_H
#define TMP_THR_H 0xD0
__EEPROM_DATA(TMP_C(0),TMP_C(1),TMP_C(2),TMP_C(3),TMP_C(4),TMP_C(5),TMP_C(6),TMP_C(7));
__EEPROM_DATA(TMP_C(8),TMP_C(9),TMP_C(10),TMP_C(11),TMP_C(12),TMP_C(13),TMP_C(14),TMP_C(15));
#undef TMP_THR_H
#define TMP_THR_H 0xE0
__EEPROM_DATA(TMP_C(0),TMP_C(1),TMP_C(2),TMP_C(3),TMP_C(4),TMP_C(5),TMP_C(6),TMP_C(7));
__EEPROM_DATA(TMP_C(8),TMP_C(9),TMP_C(10),TMP_C(11),TMP_C(12),TMP_C(13),TMP_C(14),TMP_C(15));
#undef TMP_THR_H
#define TMP_THR_H 0xF0
__EEPROM_DATA(TMP_C(0),TMP_C(1),TMP_C(2),TMP_C(3),TMP_C(4),TMP_C(5),TMP_C(6),TMP_C(7));
__EEPROM_DATA(TMP_C(8),TMP_C(9),TMP_C(10),TMP_C(11),TMP_C(12),TMP_C(13),TMP_C(14),TMP_C(15));



asm("global _aux_sort_of_static_assert_for_nop2");
void aux_sort_of_static_assert_for_nop2(void) __at(0x03FF - 5 - 2) {
    AUX_1CMD_DELAY_2
    // 'return' is auto-generated here
}

asm("global _aux_version_info_footer");
void aux_version_info_footer(void) __at(0x03FF - 5) {
    asm("retlw " AUX_STRINGIFY(FW_VER_MAJOR));
    asm("retlw " AUX_STRINGIFY(FW_VER_MINOR));
    asm("retlw " AUX_STRINGIFY(FW_VER_THR));
    asm("retlw " AUX_STRINGIFY(FW_VER_OPTION));
    // 'return' is auto-generated here so 1 last byte is skipped to ensure it won't erase OSCCAL
}
