/*
    A simple MPPT controller with non-canonical design, developed By Evgeniy Evstratov, Autumn 2022.

    This Solar controller is being developed in these dark times when russia is destroying energetic system of my country and has already caused full-scale blackout.

    The project is intended as LOW-COST simple low power MPPT device to:
    - be affordable (very cheap MCU, small and simple scheme, *NO CURRENT SENSOR);
    - get maximum solar power generation in cloudy and winter weather (proper work specifically in low power range: starting from 0.01 A).
        -- allow using high voltage panels for low-voltage devices in such conditions when very small amount of solar energy can be generated (e.g. using 18v panel to charge a smartphone powerbank).

    Principle of work: according to solar panels voltage/current curves at very most part of the range the maximum power point (MPPT) is at around 80% of open circuit voltage (named 'raw voltage' here).
    So with certain scheme it's possible to measure (almost)open-circuit voltage (with minimal load) and track optimal mode without need of a current sensor.

    Basic scheme description in words:
        1. solar panel should be connected via diode (shottky SR840 is used in the prototype).
        2. input("high voltage" and "gnd") is connected to linear voltage stabilizer and minimal(!) required ceramic capacitor (0.1 - 0.15uf):
            0.15uf cap and "LP2950CZ-3.3G ON" stabilizer are used in the prototype.
        3. linear stabilizer has output ceramic capacitor (used 2.2uf in the prototype) and is connected as the PIC MCU power supply;
            * no mini buck converter can be used instead due to it's high input capacitance;
        4. one N-channel mosfet connects a large buffer capacitor between "high voltage" and "gnd" and occassionally disconnects it to measure open circuit voltage;
            heavy 5x 1000uf 35v LOW ESR capacitors are used in the prototype.
        5. second N-channel mosfet connects an output circuit (inductor+diode+capacitor) between "high voltage" and "gnd" and is being PWMed;
            inductor is around 22uh - 47uh (in currently tested prototype 47uh is used);
            recommended capacitor is at least 2000uf (in currently tested prototype 2000uf is used);
        6. a calculated voltage divider between "high voltage" and "gnd" is connected to PIC MCU's pin with ADC channel:
            so that maximum possible voltage is <= nominal voltage of linear stabilizer;
            3.3kOhm / 20kOhm are used in prototype, I'd recommend to not exceed total resistance around 1kOhm per 1v nominal of solar panel;
        7. mosfets used in prototype are "IRLZ34N" - pretty low input capacitance, proper DS opening at 3.3v;
        8. with this minimym the output is not regulated so it can reach up to "opn circuit voltage" of the solar panel connected,
            as a very simple voltage limiter - a buck converter can be used;

    Output voltage/feedback: it's planned to implement feedback for output voltage limiting with an optocoupler and a comparator;



    Efficiency notes:
        - prototype has shown efficieny (output RMS power / input RMS power) from around 70 to 92% depending on voltage delta between an MPPT point and target output voltage;
        - efficieny was measured with not expensive equipment in limited environment;
        - used 18v panel (with typical MPPT around 15 ... 18v);
        - with plain powerbank as output ("consumed" voltage 4.5 ... 5.2v) deltaV is more than 10 volts and measured efficiency was around 75%;
        - with output 11.4v (same powerbank but luckily triggered to "stabilize at 12V QC mode" and apparently dropping it's buck cycles to maintain that)
            deltaV was around 4V and measured efficiency was 92.5%.
        - presumably higher voltage linear stabilizer (5v, more = not allowed for PIC) could do better (at least with some other mosfets) - this is to be tested;
        - Miller Effect: wasn't observed in prototype with my testing in artificial environment (tested single output driving pin vs dual output driving pin at 3.3v MCU voltage)
            * however it was tested with output voltages from 2v to 4v, and the effect could be noticed at higher output voltages - this is to be tested;
            * mosfets with larger parasitic capacitance could defenitely require doubled/tripled MCU output pin to drive them at high frequencies;
            * PIC16F676 could be easily adapted to use entire portB (6 pins) for mosfet driving;
            * mosfet that connects capacitors is rarely triggered so it can (and should) be more powerful (lower RdsON) - planned to test IRL2203 for this;

    More output voltage limiting notes:
        - powerbank:
            - tested powerbank can be tricked to trigger QC mode and rise voltage - when it's connected it "requests" higher voltage while pushing no substantial load
            and the actual voltage can be rising at the moment so powerbank considers that everything is according to plan and accepts it;
            - if voltage manages to rise higher than powerbank's limit - it will shut down and won't charge;
            - plain buck converter on output is apparenty fully "opened" and acts as a minor resistance (inductor+mosfet) with it's input and output voltages being almost equal;

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

#define PROJ_ADC_CH_VOL 2 // AN2 (GPIO2)

// ADC allows up to 10k resistance, thus for 5vols it's NOT LESS than 0.5ma current for each ADC channel
#define PROJ_ADC_ADCS 0b110 // conversion time
#define PROJ_ADC_TIME_US 16

#define PROJ_THR_START 204 // initial threshold percentage (x/255) of raw(no load) voltage to hold
#define PROJ_THR_MIN 199 // min, including (round-robin)
#define PROJ_THR_MAX 219 // max, including (round-robin)
#define PROJ_THR_DIFF_LARGER_DROPOUT 2 // thr_hi - adcval >= 'this' ? larger power drop
#define PROJ_THR_DIFF_INSTANT_DROPOUT 6 // thr_hi - adcval >= 'this' ? full power drop
#define PROJ_THR_STEP 5 // percantage to add at button press
#define PROJ_BTN_DELAY 4 // amount of 100ms periods for button state, at pre-last iteration triggers button action

#define PROJ_PWR_LEVEL_MAX 24

#define PROJ_PWR_RISE_DELAY 3 // number of consequent times for ADC checks to trigger single rise

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
#define APP_DIRTY_FLAG_1000MS 0 // bit numbers. if another 1000ms have passed and requires processing
#define APP_DIRTY_FLAG_150MS 1
#define APP_DIRTY_FLAG_THRS 2 // if thresholds need recalculation
#define APP_DIRTY_FLAG_NOP1 3
#define APP_DIRTY_FLAG_NOP2 4

#define PROJ_INITER_128MS_S0 2 // 128 in steps by 64
#define PROJ_INITER_1S_S1 8 // 1s in steps by 128

/*
    OSSCAL deviation: on my PIC it was x1.65 difference between min and max frequency (osccal 0x00-min freq and 0xFC-max freq)
    with typical calibrated values checked on 5 pics being around 0x24 - 0x44, so setting osccal to max value is assumed
    to multiply actual frequency by around x1.5 (comparing to 4mhz spec).
*/

#define PROJ_DELAY_X3_P1(a_value, lbl_name) \
    asm("movlw " AUX_STRINGIFY(a_value)); \
    asm("movwf _tmp_ctr"); \
    asm(""lbl_name":"); \
    asm("decfsz _tmp_ctr"); \
    asm("goto "lbl_name);

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



// local isr vars
uint8_t isr_ctr_128_s0 = PROJ_INITER_128MS_S0;
uint8_t isr_ctr_1s_s1 = PROJ_INITER_1S_S1;

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
    asm("decfsz _isr_ctr_128_s0,f"); // --ctr, test
    asm("goto labe_isr_out");
        asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_150MS));
        asm("movlw " AUX_STRINGIFY(PROJ_INITER_128MS_S0));
        asm("movwf _isr_ctr_128_s0"); // reinit ctr
        asm("decfsz _isr_ctr_1s_s1,f"); // --ctr, test
        asm("goto labe_isr_out");
            asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_1000MS));
            asm("movlw " AUX_STRINGIFY(PROJ_INITER_1S_S1));
            asm("movwf _isr_ctr_1s_s1"); // reinit ctr
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

#define PROJ_INLINED_ISR_REPLACEMENT(lbl_name) \
    asm("decfsz _isr_ctr_128_s0,f"); \
    asm("goto "lbl_name); \
        asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_150MS)); \
        asm("movlw " AUX_STRINGIFY(PROJ_INITER_128MS_S0)); \
        asm("movwf _isr_ctr_128_s0"); \
        asm("decfsz _isr_ctr_1s_s1,f"); \
        asm("goto "lbl_name); \
            asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_1000MS)); \
            asm("movlw " AUX_STRINGIFY(PROJ_INITER_1S_S1)); \
            asm("movwf _isr_ctr_1s_s1"); \
    asm(""lbl_name":");

#define PROJ_CATCH_AVOID_ISR(lbl_name_exit, n_timer_periods) \
    asm("movlw " AUX_STRINGIFY(256-n_timer_periods)); \
    /* asm("bcf	3,5"); select bank 0 (for tmr0 reg) */ \
    asm("subwf 1,w"); \
    asm("skipc"); /* c == no borrow -> timer is lower than constant specified */ \
    asm("goto "lbl_name_exit); \
    asm("clrf 1"); /* TMR0 = 0; avoid ISR */ \
    PROJ_INLINED_ISR_REPLACEMENT(lbl_name_exit)

/*
uint8_t RunADC_8bit(uint8_t ch) {
    // ch=0..3
    ADCON0 = 0x01 | (uint8_t)(ch << 2);// ADON=1 + set channel
    __delay_us(PROJ_ADC_TIME_US + 16); // only needed on re-acquisition
    ADCON0bits.GO_DONE = 1;
    __delay_us(PROJ_ADC_TIME_US);
    while(ADCON0bits.GO_DONE) {
        // w8
    }
    return ADRESH;
}

// runs adc on PROJ_ADC_CH_VOL, result is in ADRESH (30); relies on bank 0 being selected
void fast_RunADC_8() {
    asm("movlw " AUX_STRINGIFY((1 | (PROJ_ADC_CH_VOL << 2))));
    asm("movwf 31"); // ADCON0 = w
    asm("bsf 31,1"); // ADCON0bits.GO_DONE = 1;
    //asm("nop"); asm("nop"); // wait PROJ_ADC_TIME_US
    asm("labe_fast_RunADC_8__cycle:");
    asm("btfsc 31,1"); // while(ADCON0bits.GO_DONE)
    asm("goto labe_fast_RunADC_8__cycle");
    // asm("return"); // auto generated
}
//*/

#define PROJ_ADC_SETUP__WHEN_BANK0() \
    asm("movlw " AUX_STRINGIFY((1 | (PROJ_ADC_CH_VOL << 2)))); \
    asm("movwf 31"); // ADCON0 = w

#define PROJ_ADC_START() \
    asm("bsf 31,1"); // ADCON0bits.GO_DONE = 1;

#define PROJ_ADC_WAIT(lbl_name) \
    asm(""lbl_name":"); \
    asm("btfsc 31,1"); /* while(ADCON0bits.GO_DONE) */ \
    asm("goto "lbl_name);

// similar to fast_RunADC_8; erases tmp_ctr and tmp_ctr2 vars
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
    // 'return' is auto-geerated
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
    asm("goto labe_apply_pwr_level__d3_c1"); // 2
    asm("goto labe_apply_pwr_level__d2c1__d3c1"); // 3
    asm("goto labe_apply_pwr_level__d3c1__d3c1"); // 4
    asm("goto labe_apply_pwr_level__d3_c2"); // 5
    asm("goto labe_apply_pwr_level__d3_c3"); // 6
    asm("goto labe_apply_pwr_level__d3_c4"); // 7
    asm("goto labe_apply_pwr_level__d3_c5"); // 8
    asm("goto labe_apply_pwr_level__d3_c7"); // 9
    asm("goto labe_apply_pwr_level__d3_c9"); // 10
    asm("goto labe_apply_pwr_level__d3_c12"); // 11
    asm("goto labe_apply_pwr_level__d3_c15"); // 12
    asm("goto labe_apply_pwr_level__d3_c19"); // 13
    asm("goto labe_apply_pwr_level__d3_c24"); // 14
    asm("goto labe_apply_pwr_level__d4_c18"); // 15
    asm("goto labe_apply_pwr_level__d5_c15"); // 16
    asm("goto labe_apply_pwr_level__d6_c12"); // 17
    asm("goto labe_apply_pwr_level__d7_c10"); // 18
    asm("goto labe_apply_pwr_level__d9_c8"); // 19
    asm("goto labe_apply_pwr_level__d12_c6"); // 20
    asm("goto labe_apply_pwr_level__d15_c5"); // 21
    asm("goto labe_apply_pwr_level__d18_c4"); // 22
    asm("goto labe_apply_pwr_level__d21_c3"); // 23
    asm("goto labe_apply_pwr_level__max"); // 24
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
    asm("movlw 2");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c3:");
    asm("movlw 3");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c4:");
    asm("movlw 4");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c5:");
    asm("movlw 5");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c7:");
    asm("movlw 7");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c9:");
    asm("movlw 9");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c12:");
    asm("movlw 12");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c15:");
    asm("movlw 15");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c19:");
    asm("movlw 19");
    asm("goto labe_apply_pwr_level__d3_cycle_preinit");

    asm("labe_apply_pwr_level__d3_c24:");
    asm("movlw 24");
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
            asm("movwf _gp_shadow");
            asm("return");

    asm("labe_apply_pwr_level__d4_c18:");
    asm("movlw 18");
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
            asm("movwf _gp_shadow");
            asm("return");

    asm("labe_apply_pwr_level__d5_c15:");
    asm("movlw 15");
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
            asm("movwf _gp_shadow");
            asm("return");

    asm("labe_apply_pwr_level__d6_c12:");
    asm("movlw 12");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 6
    asm("labe_apply_pwr_level__d6_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_4
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_2
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d6_cycle");
            asm("movwf _gp_shadow");
            asm("return");

    asm("labe_apply_pwr_level__d7_c10:");
    asm("movlw 10");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 7
    asm("labe_apply_pwr_level__d7_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_5
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_3
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d7_cycle");
            asm("movwf _gp_shadow");
            asm("return");

    asm("labe_apply_pwr_level__d9_c8:");
    asm("movlw 8");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 9
    asm("labe_apply_pwr_level__d9_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_7
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_5
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d9_cycle");
            asm("movwf _gp_shadow");
            asm("return");

    asm("labe_apply_pwr_level__d12_c6:");
    asm("movlw 6");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 12
    asm("labe_apply_pwr_level__d12_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_10
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_8
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d12_cycle");
            asm("movwf _gp_shadow");
            asm("return");

    asm("labe_apply_pwr_level__d15_c5:");
    asm("movlw 5");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 15
    asm("labe_apply_pwr_level__d15_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_13
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_11
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d15_cycle");
            asm("movwf _gp_shadow");
            asm("return");

    asm("labe_apply_pwr_level__d18_c4:");
    asm("movlw 4");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 18
    asm("labe_apply_pwr_level__d18_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_16
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_14
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d18_cycle");
            asm("movwf _gp_shadow");
            asm("return");

    asm("labe_apply_pwr_level__d21_c3:");
    asm("movlw 3");
    asm("movwf _tmp_ctr");
    asm("movf _gp_shadow,w");
    // ----- generic 21
    asm("labe_apply_pwr_level__d21_cycle:");
        asm("iorlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_MASK));
        asm("movwf 5"); /* assign GPIO: output = 1 */
            AUX_DELAY_19
        asm("andlw " AUX_STRINGIFY(PROJ_OUT_FET_PINS_NEG_MASK));
        asm("movwf 5"); /* assign GPIO: output = 0 */
            AUX_DELAY_17
        asm("decfsz _tmp_ctr,f");
        asm("goto labe_apply_pwr_level__d21_cycle");
            asm("movwf _gp_shadow");
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
    asm("btfss _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_1000MS));
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
            PROJ_ADC_WAIT("labe_main__adc")
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
                asm("movlw " AUX_STRINGIFY(PROJ_PWR_RISE_DELAY)); // w = PROJ_PWR_RISE_DELAY
                asm("movwf _app_rise_counter"); // app_rise_counter = PROJ_PWR_RISE_DELAY

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
            asm("labe_main__ge_thr_hi:");
            // adcval >= thr_hi
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
            asm("btfss _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_1000MS));
            asm("goto labe_main__1000ms_non_dirty");
                // do these steps first...
                PROJ_ASSIGN_OUT_0 // ensure load is detached first
                asm("bcf _gp_shadow," AUX_STRINGIFY(PROJ_BIT_NUM_OUT_CAP));
                asm("movf _gp_shadow,w");
                asm("movwf 5"); // detach cap, give some time to establish raw voltage
                // now flags
                asm("bcf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_1000MS));
                asm("movlw " AUX_STRINGIFY(PROJ_PWR_RISE_DELAY)); // w = PROJ_PWR_RISE_DELAY
                asm("movwf _app_rise_counter"); // app_rise_counter = PROJ_PWR_RISE_DELAY
                // .. gonna put some dirt in your flags
                asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_THRS));
                asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_NOP1));
                asm("bsf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_NOP2));

                // and run adc and bring cap back
                asm("fcall _fast_RunADC_8_rising");
                asm("movf 30,w"); // w = ADRESH (ADC result)
                asm("movwf _last_raw_adc_val");

                asm("bsf _gp_shadow," AUX_STRINGIFY(PROJ_BIT_NUM_OUT_CAP));
                asm("movf _gp_shadow,w");
                asm("movwf 5"); // attach capacitor
                asm("goto labe_main__cycle"); // enough delaying work - to cycle start
            asm("labe_main__1000ms_non_dirty:");


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
                asm("movlw " AUX_STRINGIFY(PROJ_PWR_RISE_DELAY)); // w = PROJ_PWR_RISE_DELAY
                asm("movwf _app_rise_counter"); // app_rise_counter = PROJ_PWR_RISE_DELAY

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
                asm("goto labe_main__cycle"); // enough delaying work - to cycle start
            asm("labe_main__thr_non_dirty:");


            asm("btfss _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_150MS));
            asm("goto labe_main__150ms_non_dirty");
                asm("bcf _dirty_flags," AUX_STRINGIFY(APP_DIRTY_FLAG_150MS));
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
            asm("labe_main__150ms_non_dirty:");
            asm("bcf 3,5"); // ensure bank0 selected
        asm("goto labe_main__cycle");
} //
