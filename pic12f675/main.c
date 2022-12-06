/*
    A simple MPPT controller with non-canonical design, developed By Evgeniy Evstratov, Autumn 2022.

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
            * however it was tested with output voltages from 2v to 4v, and the effect could rather be noticed at higher output voltages - this is to be tested;
            * 'noticed' in this case means any significant impact on efficiency;
            * mosfets with larger parasitic capacitance could defenitely require doubled/tripled MCU output pins to drive them at high frequencies;
            * PIC16F676 could be (relatively)easily adapted to use entire portB (6 pins) for mosfet driving;
            * mosfet that connects capacitors is rarely triggered so it can (and should) be more powerful (lower RdsON) - planned to test IRL2203 for this;

    Pulsation notes:
        - output voltage pulsations looked good for current FW version although they were evaluated with simple testers (I don't have an oscilloscope);
        - even higher pulsations are *presumably okay for charging batteries with good BMS;
        - increasing capacitance (both buffer and output caps) should smooth pulsations very well (partly tested);
        - capacitance can be smaller or bigger than in prototype described above, suggestions for buffer and output caps are:
            -- minimum: 1000uf (buffer), 2000uf(output);
            -- recommended: 3000uf+ (buffer), 5000uf+(output);
        - for future addition of output voltage limitation feedback there are expected to be rare more substantial drop-offs (when reaching voltage limit) for the implementation that I have in mind;

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



// version info (3 numbers and letter) is accessible at top program-memory addresses right before OSCCAL (retlw commands)
#define FW_VER_MAJOR 0
#define FW_VER_MINOR 2
#define FW_VER_PATCH 0
#define FW_VER_OPTION 'A' // some letter describing topology/configuration of compiled code. default is 'A'
/*
    FW_VER_OPTION vaiants:
    - 'A' (0x41) : unregulated output;
        single output-fet-driving pin is GPIO0;
        GPIO1 controls mosfet of buffer capacior;
        GPIO2 is ADC input;
        GPIO4 - input mode debug pin, GPIO5 - output mode debug pin.
*/





#define AUX_STRINGIFY_INTERNAL(foo) #foo
#define AUX_STRINGIFY(foo) AUX_STRINGIFY_INTERNAL(foo)

// manual params
#define PROJ_TMR_PRESCALE (0b111) // x256

// pins / topology
#define PROJ_BIT_IN_ADC_VOL GPIO2 // voltage divider for power line - calculate so that it maxes out to chip PSU level (used 3.3 regulator)
#define PROJ_BIT_IN_BTN GPIO4 // button press (see PROJ_BTN_DELAY) adjusts percentage level of raw voltage to hold
#define PROJ_BIT_OUT_CAP GPIO1 // N-channel mosfet controlling power capacitor, PULLUP external resistor 200k
#define PROJ_BIT_OUT_OUT GPIO0 // N-channel mosfet (or 'enable' pin on output line) controlling output line, PULLDOWN external resistor
#define PROJ_BIT_OUT_DEBUG GPIO5 // output led, blinks wiht 0.1s intervals (0.1s ON / 0.1s OFF)

#define PROJ_BIT_NUM_OUT_OUT 0 // GPIO0 (PROJ_BIT_OUT_OUT)
#define PROJ_BIT_NUM_OUT_CAP 1 // GPIO1 (PROJ_BIT_OUT_CAP)
#define PROJ_BIT_NUM_IN_BTN 4 // GPIO4 (PROJ_BIT_IN_BTN)
#define PROJ_BIT_NUM_OUT_DEBUG 5 // GPIO5 (PROJ_BIT_OUT_DEBUG)

#define PROJ_ADC_CH_VOL 2 // ADC channel - AN2 (GPIO2)

// ADC allows up to 10k resistance, thus for 5vols it's NOT LESS than 0.5ma current for each ADC channel
#define PROJ_ADC_ADCS 0b010 // conversion time

#define PROJ_THR_START 204 // initial threshold percentage (x/255) of raw(no load) voltage to hold
#define PROJ_THR_MIN 199 // min, including (round-robin)
#define PROJ_THR_MAX 219 // max, including (round-robin)
#define PROJ_THR_DIFF_INSTANT_DROPOUT 6 // thr_hi - adcval >= 'this' ? full power drop
#define PROJ_THR_STEP 5 // threshold percantage (x/255) to add at button press
#define PROJ_BTN_DELAY 4 // amount of 150ms periods for button state, at pre-last iteration triggers button action

#define PROJ_PWR_LEVEL_MAX 26

#define PROJ_PWR_RISE_DELAY 3 // number of consequent times for ADC checks to trigger single rise in certain cases

volatile GPIObits_t gp_shadow;

uint8_t app_flags;
#define APP_FLAG_LAST_BTN_ON 0
#define APP_FLAG_TMP_BTN_ON 1

uint8_t app_btn_timer;
uint8_t app_debug_blinks;
uint8_t app_rise_counter;

#define PROJ_OUT_FET_PINS_MASK (1 << PROJ_BIT_NUM_OUT_OUT)
#define PROJ_OUT_FET_PINS_NEG_MASK (~PROJ_OUT_FET_PINS_MASK)

uint8_t dirty_flags;
#define APP_DIRTY_FLAG_TRAW 0 // bit numbers. if another 1000ms have passed and requires processing
#define APP_DIRTY_FLAG_TDEBUG 1 // debug time (not used if not debug build)
#define APP_DIRTY_FLAG_THRS 2 // if thresholds need recalculation
#define APP_DIRTY_FLAG_NOP1 3
#define APP_DIRTY_FLAG_NOP2 4

/*
    OSSCAL deviation: on my PIC it was x1.65 difference between min and max frequency (osccal 0x00-min freq and 0xFC-max freq)
    with typical calibrated values checked on 5 pics being around 0x24 - 0x44, so setting osccal to max value is assumed
    to multiply actual frequency by around x1.5 (comparing to 4mhz spec).
*/

// relies on active bank 0
#define PROJ_ASSIGN_OUT_1 \
    asm("movf _gp_shadow,w"); \
    asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK)); \
    asm("movwf 5"); /* assign GPIO */ \
    asm("movwf _gp_shadow"); /* refresh gp_shadow */
#define PROJ_ASSIGN_OUT_0 \
    asm("movf _gp_shadow,w"); \
    asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK)); \
    asm("movwf 5"); /* assign GPIO */ \
    asm("movwf _gp_shadow"); /* refresh gp_shadow */


//#define PROJ_DEBUG_BTN

// local isr vars
#ifdef PROJ_DEBUG_BTN
    #define PROJ_ISR_CTR_INITER_T0 2 // 128 in steps by 64
    #define PROJ_ISR_CTR_INITER_T1 8 // 1s in steps by 128

    uint8_t isr_ctr_t0 = PROJ_ISR_CTR_INITER_T0;
    uint8_t isr_ctr_t1 = PROJ_ISR_CTR_INITER_T1;
#else
    #define PROJ_ISR_CTR_INITER_T0 16

    uint8_t isr_ctr_t0 = PROJ_ISR_CTR_INITER_T0;
#endif

// isr cache
uint8_t temp_w;
uint8_t temp_status;
//

asm("global _isr");
void isr(void) __at(0x0004)
{
    asm("movwf _temp_w");
    asm("swapf 3,w");
    asm("movwf _temp_status");
    //asm("bcf 3,5"); // select bank 0 - may be not necessary in isr
    // context saved

    PROJ_ASSIGN_OUT_0 // we actually should never get here as we catch upcoming timer interrupt and process manually
    // ++64ms
#ifdef PROJ_DEBUG_BTN
    asm("decfsz _isr_ctr_t0,f"); // --ctr, test
    asm("goto labe_isr_out");
        asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_TDEBUG));
        asm("movlw " AUX_STRINGIFY(PROJ_ISR_CTR_INITER_T0));
        asm("movwf _isr_ctr_t0"); // reinit ctr
        asm("decfsz _isr_ctr_t1,f"); // --ctr, test
        asm("goto labe_isr_out");
            asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_TRAW));
            asm("movlw " AUX_STRINGIFY(PROJ_ISR_CTR_INITER_T1));
            asm("movwf _isr_ctr_t1"); // reinit ctr
#else
    asm("decfsz _isr_ctr_t0,f"); // --ctr, test
    asm("goto labe_isr_out");
        asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_TRAW));
        asm("movlw " AUX_STRINGIFY(PROJ_ISR_CTR_INITER_T0));
        asm("movwf _isr_ctr_t0"); // reinit ctr
#endif
    asm("labe_isr_out:");
    asm("bcf 11,2"); // INTCONbits.T0IF = 0; // regardless of current bank

    // context restoring
    asm("swapf _temp_status,w");
    asm("movwf 3"); // recover status
    asm("swapf _temp_w,f");
    asm("swapf _temp_w,w"); // recover w
    //
    asm("retfie"); // compiler auto-generates "return" here, nevermind at the moment, just use explicit retfie.
}

#ifdef PROJ_DEBUG_BTN
// >= 3 tacts: 3 or 8 or 10
#define PROJ_INLINED_ISR_REPLACEMENT(lbl_name) \
    asm("decfsz _isr_ctr_t0,f"); \
    asm("goto "lbl_name); \
        asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_TDEBUG)); \
        asm("movlw " AUX_STRINGIFY(PROJ_ISR_CTR_INITER_T0)); \
        asm("movwf _isr_ctr_t0"); \
        asm("decfsz _isr_ctr_t1,f"); \
        asm("goto "lbl_name); \
            asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_TRAW)); \
            asm("movlw " AUX_STRINGIFY(PROJ_ISR_CTR_INITER_T1)); \
            asm("movwf _isr_ctr_t1"); \
    asm(""lbl_name":");
#else
// >= 3 tacts: 3 or 5
#define PROJ_INLINED_ISR_REPLACEMENT(lbl_name) \
    asm("decfsz _isr_ctr_t0,f"); \
    asm("goto "lbl_name); \
        asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_TRAW)); \
        asm("movlw " AUX_STRINGIFY(PROJ_ISR_CTR_INITER_T0)); \
        asm("movwf _isr_ctr_t0"); \
    asm(""lbl_name":");
#endif

// >= 5 tacts: 5 or 5+(3 or 8 or 10) == 5 or 8 or 13 or 15 tacts :: debug
// >= 5 tacts: 5 or 5+(3 or 5) == 5 or 8 or 10 tacts             :: non-debug
#define PROJ_CATCH_AVOID_ISR(lbl_name_exit, n_timer_periods) \
    asm("movlw " AUX_STRINGIFY(256-n_timer_periods)); \
    /* asm("bcf	3,5"); select bank 0 (for tmr0 reg) */ \
    asm("subwf 1,w"); \
    asm("skipc"); /* c == no borrow -> timer is lower than constant specified */ \
    asm("goto "lbl_name_exit); \
    asm("clrf 1"); /* TMR0 = 0; avoid ISR */ \
    PROJ_INLINED_ISR_REPLACEMENT(lbl_name_exit)



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
    // got result in '30' (ADRESH)
    asm("movf 30,w");
    asm("addlw 254"); // w = adc - 2
    asm("subwf _tmp_ctr2,w"); // w = prev_adc - (adc - 2) ;; trigger borrow flag
    asm("skipnc"); // skip next if (borrow -> still rising)
    asm("return");
    // still rising
    asm("decfsz _tmp_ctr");
    asm("goto labe_fast_RunADC_8_rising__continue");
    asm("return");

    asm("labe_fast_RunADC_8_rising__continue:");
    asm("movf 30,w"); // refresh last result (tmp_ctr2)
    asm("movwf _tmp_ctr2");
    asm("goto labe_fast_RunADC_8_rising__rising_cycle");
}


// TODO multiplication will be optimized to reduce delay, likely in one of two ways (if calibration is fully discarded):
// 1. shorter multiply-by-constant code
// 2. mapped table (addwf 2,...; retlw ... retlw) with around 220-230 bytes of program memory
// another option is several predefined multiply-by-constant blocks with combined tabled jump to required one
uint8_t fast_mul_8x8_res_h;
uint8_t fast_mul_8x8_res_l;
uint8_t fast_mul_8x8_input1; // input2 is w
void fast_mul_8x8() {
    asm("clrf _fast_mul_8x8_res_h");
    asm("clrf _fast_mul_8x8_res_l");

    asm("btfss _fast_mul_8x8_input1,7");
    asm("goto labe_fast_mul_8x8__post7");
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");

        asm("clrc"); // post 7, pre 6
        asm("rlf _fast_mul_8x8_res_l,f");
        asm("rlf _fast_mul_8x8_res_h,f");
    asm("labe_fast_mul_8x8__post7:");
    //
    asm("btfss _fast_mul_8x8_input1,6");
    asm("goto labe_fast_mul_8x8__post6");
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");
    asm("labe_fast_mul_8x8__post6:");
            asm("clrc"); // post 6, pre 5
            asm("rlf _fast_mul_8x8_res_l,f");
            asm("rlf _fast_mul_8x8_res_h,f");
    asm("btfss _fast_mul_8x8_input1,5");
    asm("goto labe_fast_mul_8x8__post5");
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");
    asm("labe_fast_mul_8x8__post5:");
            asm("clrc"); // post 5, pre 4
            asm("rlf _fast_mul_8x8_res_l,f");
            asm("rlf _fast_mul_8x8_res_h,f");
    asm("btfss _fast_mul_8x8_input1,4");
    asm("goto labe_fast_mul_8x8__post4");
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");
    asm("labe_fast_mul_8x8__post4:");
            asm("clrc"); // post 4, pre 3
            asm("rlf _fast_mul_8x8_res_l,f");
            asm("rlf _fast_mul_8x8_res_h,f");
    asm("btfss _fast_mul_8x8_input1,3");
    asm("goto labe_fast_mul_8x8__post3");
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");
    asm("labe_fast_mul_8x8__post3:");
            asm("clrc"); // post 3, pre 2
            asm("rlf _fast_mul_8x8_res_l,f");
            asm("rlf _fast_mul_8x8_res_h,f");
    asm("btfss _fast_mul_8x8_input1,2");
    asm("goto labe_fast_mul_8x8__post2");
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");
    asm("labe_fast_mul_8x8__post2:");
            asm("clrc"); // post 2, pre 1
            asm("rlf _fast_mul_8x8_res_l,f");
            asm("rlf _fast_mul_8x8_res_h,f");
    asm("btfss _fast_mul_8x8_input1,1");
    asm("goto labe_fast_mul_8x8__post1");
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");
    asm("labe_fast_mul_8x8__post1:");
            asm("clrc"); // post 1, pre 0
            asm("rlf _fast_mul_8x8_res_l,f");
            asm("rlf _fast_mul_8x8_res_h,f");
    asm("btfss _fast_mul_8x8_input1,0");
    asm("return"); // asm("goto labe_fast_mul_8x8__post0");
        asm("addwf _fast_mul_8x8_res_l,f");
        asm("skipnc");
        asm("incf _fast_mul_8x8_res_h");
    asm("labe_fast_mul_8x8__post0:");
    // asm("return"); // auto generated
}

uint8_t g_thr = PROJ_THR_START;
uint8_t last_raw_adc_val = 0;
uint8_t pending_raw_adc_val;
uint8_t thr_min;
uint8_t thr_hi; // high level voltage threshold
uint8_t thr_lo; // low/instant-off voltage threshold
uint8_t pwr_level = 0; // active power level - the higher - the lower is inductor frequency. zero is disconnected
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
#define AUX_DELAY_2 asm("nop"); asm("nop");
#define AUX_DELAY_3 asm("nop"); asm("nop"); asm("nop");

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


void apply_pwr_level() __at(0x0040)
{
    // asm("bcf 3,5"); // ensure bank0 selected
    asm("movf _pwr_level,w");
    asm("addwf 2,f"); // PCL += pwr_level: tabled jump

    // TODO OPT MIGHT take advantage of jump to offset*2 to avoid intermediate jumps in some cases

    asm("goto labe_apply_pwr_level__zero"); // 0
    asm("goto labe_apply_pwr_level__d2_c1"); // 1
    asm("goto labe_apply_pwr_level__d2_c2_long"); // 2
    asm("goto labe_apply_pwr_level__d2_c2_short"); // 3
    asm("goto labe_apply_pwr_level__d3_c1"); // 4
    asm("goto labe_apply_pwr_level__d2c1__d3c1"); // 5
    asm("goto labe_apply_pwr_level__d3c1__d3c1"); // 6
    asm("goto labe_apply_pwr_level__d3_c2"); // 7
    asm("goto labe_apply_pwr_level__d3_c3"); // 8
    asm("goto labe_apply_pwr_level__d3_c4"); // 9
    asm("goto labe_apply_pwr_level__d3_c5"); // 10
    asm("goto labe_apply_pwr_level__d3_c7"); // 11
    asm("goto labe_apply_pwr_level__d3_c9"); // 12
    asm("goto labe_apply_pwr_level__d3_c12"); // 13
    asm("goto labe_apply_pwr_level__d3_c15"); // 14
    asm("goto labe_apply_pwr_level__d3_c19"); // 15
    asm("goto labe_apply_pwr_level__d3_c24"); // 16
    asm("goto labe_apply_pwr_level__d4_c18"); // 17
    asm("goto labe_apply_pwr_level__d5_c15"); // 18
    asm("goto labe_apply_pwr_level__d6_c12"); // 19
    asm("goto labe_apply_pwr_level__d7_c10"); // 20
    asm("goto labe_apply_pwr_level__d9_c8"); // 21
    asm("goto labe_apply_pwr_level__d12_c6"); // 22
    asm("goto labe_apply_pwr_level__d15_c5"); // 23
    asm("goto labe_apply_pwr_level__d18_c4"); // 24
    asm("goto labe_apply_pwr_level__d21_c3"); // 25
    asm("goto labe_apply_pwr_level__max"); // 26
    // end table
    // ----- 0
    asm("labe_apply_pwr_level__zero:");
        PROJ_ASSIGN_OUT_0
        asm("return");

    // ----- duration 2
    asm("labe_apply_pwr_level__d2_c1:");
        asm("movf _gp_shadow,w");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("movwf _gp_shadow"); // refresh gp_shadow
        asm("return");

    asm("labe_apply_pwr_level__d2_c2_long:");
        asm("movf _gp_shadow,w");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        AUX_DELAY_10
        // TODO SIZE: can reduce delay and just jump to labe_apply_pwr_level__d2_c1
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("movwf _gp_shadow"); // refresh gp_shadow
        asm("return");

    asm("labe_apply_pwr_level__d2_c2_short:");
        asm("movf _gp_shadow,w");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        AUX_DELAY_4
        // TODO SIZE: can reduce delay and just jump to labe_apply_pwr_level__d2_c1
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("movwf _gp_shadow"); // refresh gp_shadow
        asm("return");

    // ----- duration 3
    asm("labe_apply_pwr_level__d3_c1:");
        asm("movf _gp_shadow,w");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        AUX_DELAY_1
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("movwf _gp_shadow"); // refresh gp_shadow
        asm("return");

    // ----- duration 2+3
    asm("labe_apply_pwr_level__d2c1__d3c1:");
        asm("movf _gp_shadow,w");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        AUX_DELAY_7 // distribute load more evenly
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        AUX_DELAY_1
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("movwf _gp_shadow");
        asm("return");

    // ----- duration 3+3
    asm("labe_apply_pwr_level__d3c1__d3c1:");
        asm("movf _gp_shadow,w");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        AUX_DELAY_1
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        AUX_DELAY_7 // distribute load more evenly
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        AUX_DELAY_1
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("movwf _gp_shadow");
        asm("return");

    asm("labe_apply_pwr_level__d3_c2:");
    asm("movlw 1");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c3:");
    asm("movlw 2");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c4:");
    asm("movlw 3");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c5:");
    asm("movlw 4");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c7:");
    asm("movlw 6");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c9:");
    asm("movlw 8");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c12:");
    asm("movlw 11");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c15:");
    asm("movlw 14");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c19:");
    asm("movlw 18");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c24:");
    asm("movlw 23");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    // ----- generic 3
    asm("labe_apply_pwr_level__d3_cycle_preinit:");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    asm("labe_apply_pwr_level__d3_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_1
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d3_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("movwf _gp_shadow"); // update gp_shadow earlier... by same reason
            asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_1
            asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d4_c18:");
    asm("movlw 17");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 4
    asm("labe_apply_pwr_level__d4_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_2
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d4_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("movwf _gp_shadow"); // update gp_shadow earlier... by same reason
            asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_2
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d5_c15:");
    asm("movlw 14");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 5
    asm("labe_apply_pwr_level__d5_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_3
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d5_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("movwf _gp_shadow"); // update gp_shadow earlier... by same reason
            asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_3
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d6_c12:");
    asm("movlw 11");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 6
    asm("labe_apply_pwr_level__d6_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_4
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_1
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d6_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("movwf _gp_shadow"); // update gp_shadow earlier... by same reason
            asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_4
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d7_c10:");
    asm("movlw 9");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 7
    asm("labe_apply_pwr_level__d7_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_5
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_2
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d7_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("movwf _gp_shadow"); // update gp_shadow earlier... by same reason
            asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_5
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d9_c8:");
    asm("movlw 7");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 9
    asm("labe_apply_pwr_level__d9_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_7
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_4
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d9_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("movwf _gp_shadow"); // update gp_shadow earlier... by same reason
            asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_7
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d12_c6:");
    asm("movlw 5");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 12
    asm("labe_apply_pwr_level__d12_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_10
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_7
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d12_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("movwf _gp_shadow"); // update gp_shadow earlier... by same reason
            asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_10
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d15_c5:");
    asm("movlw 4");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 15
    asm("labe_apply_pwr_level__d15_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_13
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_10
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d15_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("movwf _gp_shadow"); // update gp_shadow earlier... by same reason
            asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_13
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__d18_c4:");
    asm("movlw 3");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 18
    asm("labe_apply_pwr_level__d18_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_16
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_13
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d18_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("movwf _gp_shadow"); // update gp_shadow earlier... by same reason
            asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_16
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");


    // special handler
    asm("labe_apply_pwr_level__special:");
    asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
    asm("movwf 5"); /* assign GPIO: output = 0 */
        AUX_DELAY_17
    asm("goto labe_apply_pwr_level__pre_max_level");
    //

    asm("labe_apply_pwr_level__d21_c3:");
    asm("movlw 2");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    //
    asm("btfsc 5," AUX_STRINGIFY(PROJ_BIT_NUM_OUT_OUT)); // check if we came here from pwr MAX - then pre-apply 0
    asm("goto labe_apply_pwr_level__special");
    //
    // ----- generic 21
    asm("labe_apply_pwr_level__pre_max_level:");
    asm("labe_apply_pwr_level__d21_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_19
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_16
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d21_cycle");
            // last iteration - after cycle, to minimize further delay
            asm("movwf _gp_shadow"); // update gp_shadow earlier... by same reason
            asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
            asm("movwf 5"); /* assign GPIO: output = 1 */
                AUX_DELAY_19
            // TODO SIZE: can reduce delay and just jump to such common quit
            asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
            asm("movwf 5"); /* assign GPIO: output = 0 */
            asm("return");

    asm("labe_apply_pwr_level__max:");
        PROJ_ASSIGN_OUT_1
        asm("return");
}

void main() {
    TRISIO = 0x1C; // outputs (0s): GPIO0, GPIO1, GPIO5
    ANSEL = (PROJ_ADC_ADCS << 4) | 0x04; // set conversion time;; enable AN2
    GPIO = 0;
    OSCCAL = 0;

    CMCON = 0x07;
    WPU = 0xFF; // all possible pullups
    OSCCAL = __osccal_val();

    gp_shadow = GPIObits;

    app_flags = 0;
    dirty_flags = 0;
    app_btn_timer = 0;

    app_debug_blinks = 0;
    app_rise_counter = PROJ_PWR_RISE_DELAY;

    OPTION_REG = (PROJ_TMR_PRESCALE); // (!nGPPU - enable pullups), ps = b100 (prescale 32)
    TMR0 = 0;
    INTCON = 0xA0; // GIE + T0IE: enable interrupts and T0 timer

    asm("bcf 3,5"); // ensure bank0 selected
    PROJ_ADC_SETUP__WHEN_BANK0()

    asm("bsf _gp_shadow," AUX_STRINGIFY(PROJ_BIT_NUM_OUT_CAP));
    asm("movf _gp_shadow,w");
    asm("movwf 5"); // ensure attaching capacitor, we'll be dropping out to BOD here at start and at insufficient power cases
    AUX_DELAY_40
    PROJ_ADC_START()
    PROJ_ADC_WAIT("labe_preinit_adc_min_voltage")
    asm("movf 30,w"); // w = ADRESH
    asm("movwf _thr_min"); // thr_min = w (ADC result)

    if((thr_min + (3 + PROJ_THR_DIFF_INSTANT_DROPOUT)) > thr_min) { // +1% +dropout. ensure no overflow
        // must be no overflow - otherwise it's mistake in scheme or ADC failure
        thr_min += (3 + PROJ_THR_DIFF_INSTANT_DROPOUT);
    }

    // pre-init raw adc with pre-wait 1 second
    asm("bcf 3,5"); // ensure bank0 selected
    asm("labe_main__preinit_raw_adc:");
    asm("btfss _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_TRAW));
    asm("goto labe_main__preinit_raw_adc");
    asm("clrf _dirty_flags");
    {
        PROJ_ASSIGN_OUT_0
        asm("bcf _gp_shadow," AUX_STRINGIFY(PROJ_BIT_NUM_OUT_CAP));
        asm("movf _gp_shadow,w");
        asm("movwf 5"); // detach cap

        asm("fcall _fast_RunADC_8_rising");
        asm("movf 30,w"); // w = ADRESH (ADC result)
        asm("movwf _last_raw_adc_val");

        asm("bsf _gp_shadow," AUX_STRINGIFY(PROJ_BIT_NUM_OUT_CAP));
        asm("movf _gp_shadow,w");
        asm("movwf 5"); // attach capacitor
    }
    {
        //asm("bcf 3,5"); // ensure bank0 selected
        asm("movf _last_raw_adc_val,w"); // TODO input1 can be union with last_raw_adc_val to save 2 commands
        asm("movwf _fast_mul_8x8_input1");
        asm("movf _g_thr,w");
        asm("fcall _fast_mul_8x8");
        asm("movf _fast_mul_8x8_res_h,w");
        asm("movwf _thr_hi"); // thr_hi = last_raw_adc_val * g_thr / 256
        //
        asm("subwf _thr_min,w"); // w(thr_hi) = thr_min - w ; if borrow - no action
        asm("skipnc"); // skip next if borrow
        asm("addwf _thr_hi,f"); // thr_hi += (thr_min - thr_hi)
        //
        asm("movlw " AUX_STRINGIFY(PROJ_THR_DIFF_INSTANT_DROPOUT));
        asm("subwf _thr_hi,w"); // w = thr_hi - PROJ_THR_DIFF_INSTANT_DROPOUT
        asm("movwf _thr_lo");
    }
    //

    asm("labe_main__cycle:");
        // asm("bcf 3,5"); // ensure bank0 selected
        PROJ_ADC_START()
        PROJ_CATCH_AVOID_ISR("labe_main__intercept_isr", 2) // 2 => at least 512 tacts until isr
        // main part:
            PROJ_ADC_WAIT("labe_main__adc") // TODO if ADC time selection changes to 4us - explicit awaiting can be omitted in this place
#define VAR_ADCVAL "30" // avoid copying ADC result to minimize executed code
//            asm("movf 30,w"); // w = ADRESH
//            asm("movwf _tmp_ctr"); // tmp_ctr = w (ADC result)
//#define VAR_ADCVAL "_tmp_ctr"
            // relying on bank0 selected here
            asm("movf _thr_hi,w"); // w = thr_hi
            asm("subwf " VAR_ADCVAL ",w"); // w = adcval - thr_hi
            asm("skipnc"); // nc == borrow
            asm("goto labe_main__ge_thr_hi"); // jump if adcval >= thr_hi
                // else : adcval < thr_hi
                asm("movlw " AUX_STRINGIFY(1)); // w = 1
                asm("movwf _app_rise_counter"); // app_rise_counter = 1 (allow fastest rise)

                asm("movf " VAR_ADCVAL ",w"); // w = adcval
                asm("subwf _thr_lo,w"); // w = thr_lo - adcval
                asm("skipnc"); // nc == borrow
                asm("goto labe_main__le_thr_lo"); // jump if thr_lo >= adcval
                    // else : adcval > thr_lo ; adcval < thr_hi
                    asm("movf _pwr_level,f"); // test self
                    asm("skipz"); // check for non-zero
                    asm("decf _pwr_level,f"); // decrement (if non zero)
                    asm("goto labe_main__pre_apply_pwr");
                asm("labe_main__le_thr_lo:");
                // thr_lo >= adcval
                asm("clrf _pwr_level");
                asm("goto labe_main__pre_apply_pwr");
                ////
                asm("labe_main__eq_thr_hi:"); // adcval == thr_hi
                asm("movf _pwr_level,f"); // test curr power
                asm("skipnz");
                asm("goto labe_main__maybe_rise"); // allow such rising from pwr0 to pwr1 only, otherwise - reset counter, consider a 'stable state'
                asm("movlw " AUX_STRINGIFY(1)); // w = 1
                asm("movwf _app_rise_counter"); // app_rise_counter = 1 (allow fastest rise)
                asm("goto labe_main__pre_apply_pwr");
                ////
            asm("labe_main__ge_thr_hi:");
            // adcval >= thr_hi
            asm("skipnz");
            asm("goto labe_main__eq_thr_hi"); // jump if adcval == thr_hi
            // adcval > thr_hi
            asm("labe_main__maybe_rise:");
            asm("decfsz _app_rise_counter");
            asm("goto labe_main__pre_apply_pwr"); // rising not allowed yet, jump to apply pwr
                // else - rising allowed, reset the counter and rise
                asm("movlw " AUX_STRINGIFY(PROJ_PWR_RISE_DELAY)); // w = PROJ_PWR_RISE_DELAY
                asm("movwf _app_rise_counter"); // app_rise_counter = PROJ_PWR_RISE_DELAY
            asm("movlw " AUX_STRINGIFY(PROJ_PWR_LEVEL_MAX));
            asm("subwf _pwr_level,w");
            asm("skipc");
            asm("incf _pwr_level,f"); // increment, if not max
            // redundant cmd: asm("goto labe_main__pre_apply_pwr");
            asm("labe_main__pre_apply_pwr:");
            asm("fcall _apply_pwr_level");
        // main part done


        // asm("bcf 3,5"); // ensure bank0 selected
        asm("movf _dirty_flags,f"); // test
        asm("skipnz");
        asm("goto labe_main__cycle");
            // something is dirty
            //asm("bcf 3,5"); // ensure bank0 selected
            asm("btfss _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_TRAW));
            asm("goto labe_main__traw_non_dirty");
                // do these steps first...
                PROJ_ASSIGN_OUT_0 // ensure load is detached first
                asm("bcf _gp_shadow," AUX_STRINGIFY(PROJ_BIT_NUM_OUT_CAP));
                asm("movf _gp_shadow,w");
                asm("movwf 5"); // detach cap, give some time to establish raw voltage
                // now flags
                asm("bcf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_TRAW));
                asm("movlw " AUX_STRINGIFY(PROJ_PWR_RISE_DELAY)); // w = PROJ_PWR_RISE_DELAY
                asm("movwf _app_rise_counter"); // app_rise_counter = PROJ_PWR_RISE_DELAY
                // .. gonna put some dirt in your flags
                asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_THRS));
                asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_NOP1));
                asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_NOP2));

                // and run adc and bring cap back
                asm("fcall _fast_RunADC_8_rising");
                // attach cap first
                asm("bsf _gp_shadow," AUX_STRINGIFY(PROJ_BIT_NUM_OUT_CAP));
                asm("movf _gp_shadow,w");
                asm("movwf 5"); // attach capacitor
                // now save result
                asm("movf 30,w"); // w = ADRESH (ADC result)
                asm("movwf _pending_raw_adc_val");
                asm("goto labe_main__cycle"); // enough delaying work - to cycle start
            asm("labe_main__traw_non_dirty:");


            // cascade nops - used together (nop2 is nested in nop1 to minimize cycle path delay)
            asm("btfss _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_NOP1));
            asm("goto labe_main__nop1_non_dirty");
                asm("btfss _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_NOP2));
                asm("goto labe_main__nop2_non_dirty");
                    asm("bcf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_NOP2));
                    asm("goto labe_main__cycle");
                asm("labe_main__nop2_non_dirty:");
                //
                asm("bcf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_NOP1));
                asm("goto labe_main__cycle");
            asm("labe_main__nop1_non_dirty:");


            asm("btfss _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_THRS));
            asm("goto labe_main__thr_non_dirty");
                asm("bcf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_THRS));
                //
                asm("movf _pending_raw_adc_val,w"); // TODO input1 can be union with last_raw_adc_val to save 2 commands
                asm("xorwf _last_raw_adc_val,w");
                asm("skipnz");
                asm("goto labe_main__cycle"); // no changes - to cycle start
                // different - needs recalculation
                asm("movf _pending_raw_adc_val,w");
                asm("movwf _last_raw_adc_val");
                asm("movwf _fast_mul_8x8_input1");
                //
                asm("movlw " AUX_STRINGIFY(PROJ_PWR_RISE_DELAY)); // w = PROJ_PWR_RISE_DELAY
                asm("movwf _app_rise_counter"); // app_rise_counter = PROJ_PWR_RISE_DELAY

                asm("movf _g_thr,w");
                asm("fcall _fast_mul_8x8");
                asm("movf _fast_mul_8x8_res_h,w");
                asm("movwf _thr_hi"); // thr_hi = last_raw_adc_val * g_thr / 256
                //
                asm("subwf _thr_min,w"); // w(thr_hi) = thr_min - w ; if borrow - no action
                asm("skipnc"); // skip next if borrow
                asm("addwf _thr_hi,f"); // thr_hi += (thr_min - thr_hi)
                //
                asm("movlw " AUX_STRINGIFY(PROJ_THR_DIFF_INSTANT_DROPOUT));
                asm("subwf _thr_hi,w"); // w = thr_hi - PROJ_THR_DIFF_INSTANT_DROPOUT
                asm("movwf _thr_lo");
                asm("goto labe_main__cycle"); // enough delaying work - to cycle start
            asm("labe_main__thr_non_dirty:");

#ifdef PROJ_DEBUG_BTN
            asm("btfss _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_TDEBUG));
            asm("goto labe_main__tdebug_non_dirty");
                asm("bcf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_TDEBUG));
                //asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_NOP1));
                //asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_NOP2));
                //asm("bcf 3,5"); // ensure bank0 selected
                asm("movlw " AUX_STRINGIFY(PROJ_PWR_RISE_DELAY)); // w = PROJ_PWR_RISE_DELAY
                asm("movwf _app_rise_counter"); // app_rise_counter = PROJ_PWR_RISE_DELAY
                //
                asm("bcf _app_flags," AUX_STRINGIFY(APP_FLAG_TMP_BTN_ON));
                asm("btfss 5," AUX_STRINGIFY(PROJ_BIT_NUM_IN_BTN)); // GPIObits.PROJ_BIT_IN_BTN
                asm("bsf _app_flags," AUX_STRINGIFY(APP_FLAG_TMP_BTN_ON)); // app.bits.tmp_btn_on = (0 == GPIObits.PROJ_BIT_IN_BTN);

                asm("clrw");
                asm("btfsc _app_flags," AUX_STRINGIFY(APP_FLAG_TMP_BTN_ON));
                asm("addlw 1");
                asm("btfsc _app_flags," AUX_STRINGIFY(APP_FLAG_LAST_BTN_ON));
                asm("addlw 255");
                // now W is zero if APP_FLAG_TMP_BTN_ON==APP_FLAG_LAST_BTN_ON
                asm("iorlw 0"); // test for zero
                asm("skipnz");
                asm("goto labe_main__btn_unchanged");
                    // last_btn_on = tmp_btn_on
                    asm("bcf _app_flags," AUX_STRINGIFY(APP_FLAG_LAST_BTN_ON));
                    asm("btfsc _app_flags," AUX_STRINGIFY(APP_FLAG_TMP_BTN_ON));
                    asm("bsf _app_flags," AUX_STRINGIFY(APP_FLAG_LAST_BTN_ON));
                    // app_btn_timer = 0
                    asm("clrf _app_btn_timer");
                    asm("goto labe_main__btn_timer_updated");
                asm("labe_main__btn_unchanged:");
                asm("movlw " AUX_STRINGIFY(PROJ_BTN_DELAY));
                asm("subwf _app_btn_timer,w");
                asm("skipc");
                asm("incf _app_btn_timer,f"); // increment, if not max
                //
                asm("labe_main__btn_timer_updated:");
                //
                //
                asm("btfss _app_flags," AUX_STRINGIFY(APP_FLAG_LAST_BTN_ON));
                asm("goto labe_main__btn_processed");
                // if last_btn_on
                asm("movlw " AUX_STRINGIFY(PROJ_BTN_DELAY - 1));
                asm("subwf _app_btn_timer,w");
                asm("skipz");
                asm("goto labe_main__btn_processed");
                // if btn_timer == (PROJ_BTN_DELAY-1)
                // TODO asmify if keep this
                { // handle btn
                    g_thr += PROJ_THR_STEP;
                    if(g_thr > PROJ_THR_MAX) {
                        g_thr = PROJ_THR_MIN;
                    }
                    if(! app_debug_blinks) {
                        uint8_t tmp = g_thr - PROJ_THR_MIN;
                        app_debug_blinks = 1;
                        // instead of heavy division operation
                        while(tmp) {
                            tmp -= PROJ_THR_STEP;
                            ++app_debug_blinks;
                        }
                        app_debug_blinks <<= 1;
                    }
                }
                asm("labe_main__btn_processed:");

                if(app_debug_blinks) {
                    --app_debug_blinks;
                    gp_shadow.PROJ_BIT_OUT_DEBUG = ! gp_shadow.PROJ_BIT_OUT_DEBUG; // invert led
                    GPIObits = gp_shadow;
                }
            asm("labe_main__tdebug_non_dirty:");
            asm("bcf 3,5"); // ensure bank0 selected
#endif
        asm("goto labe_main__cycle");
} //



asm("global _aux_version_info_footer");
void aux_version_info_footer(void) __at(0x03FF - 5) {
    asm("retlw " AUX_STRINGIFY(FW_VER_MAJOR));
    asm("retlw " AUX_STRINGIFY(FW_VER_MINOR));
    asm("retlw " AUX_STRINGIFY(FW_VER_PATCH));
    asm("retlw " AUX_STRINGIFY(FW_VER_OPTION));
    // 'return' is auto-generated here so 1 last byte is skipped to ensure it won't erase OSCCAL
}
