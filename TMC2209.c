#include "pico/stdlib.h"
#include "stdio.h"
#include "hardware/uart.h"

#define STEP_PIN    23
#define DIR_PIN     22
#define EN_PIN      21
#define TX_PIN      24
#define RX_PIN      25

#define TMC2209_UART    uart1
#define BAUDRATE        115200

#define DATA_BITS   8
#define STOP_BIT    1

#define TMC2209_SYNC_BIT    0x05
#define TMC2209_WRITE_BIT   0x80

#define ADDRESS_GCONF       0x00
#define ADRESS_IHOLD_IRUN   0x10
#define ADRESS_VACTUAL      0x22
#define ADRESS_CHOPCONF     0x6C

//TMC2209 adresses enum
enum slave_adress
{
    DRIVER_1, DRIVER_2, DRIVER_3, DRIVER_4
};

//TMC2209 microstep resolution enum (0-256,1-128, etc)
enum microstep_resolution
{
    MICROSTEP_RESOLUTION_256,
    MICROSTEP_RESOLUTION_128,
    MICROSTEP_RESOLUTION_64,
    MICROSTEP_RESOLUTION_32,
    MICROSTEP_RESOLUTION_16,
    MICROSTEP_RESOLUTION_8,
    MICROSTEP_RESOLUTION_4,
    MICROSTEP_RESOLUTION_2,
    MICROSTEP_RESOLUTION_FULL_STEP
};

//ADRESS_IHOLD_IRUN register bytes
union DriverCurrent
{
    struct
    {
        uint32_t ihold : 5;         //Bit 4..0 - Standstill current (0=1/32 … 31=32/32)
        uint32_t reserved_0 : 3;    //Empty
        uint32_t irun : 5;          //Bit 12..8 - Motor run current (0=1/32 … 31=32/32)
        uint32_t reserved_1 : 3;    //Empty
        uint32_t iholddelay : 4;    //Bit 19..16 - Controls the number of clock cycles for motor power down after standstill is detected
        uint32_t reserved_2 : 12;   //Empty
    };
    uint32_t bytes;
};
static union DriverCurrent driver_current;

//ADDRESS_GCONF register bytes
union GlobalConfig
{
    struct
    {
        uint32_t i_scale_analog : 1;
        uint32_t internal_rsense : 1;
        uint32_t enable_spread_cycle : 1;
        uint32_t shaft : 1;                 // Inverse motor direction
        uint32_t index_otpw : 1;
        uint32_t index_step : 1;
        uint32_t pdn_disable : 1;
        uint32_t mstep_reg_select : 1;      // Microstep resolution selection (0 - by MS1&MS2 pins, 1 - by MSTEP register)
        uint32_t multistep_filt : 1;
        uint32_t test_mode : 1;
        uint32_t reserved : 22;
    };
    uint32_t bytes;
};
static union GlobalConfig driver_config;

//ADRESS_CHOPCONF register bytes
union ChopperConfig
{
    struct
    {
        uint32_t toff : 4;
        uint32_t hstrt : 3;
        uint32_t hend : 4;
        uint32_t reserved_0 : 4;
        uint32_t tbl : 2;
        uint32_t vsense : 1;
        uint32_t reserved_1 : 6;
        uint32_t mres : 4;              //Microstep resolution (0-256,1-128, etc)
        uint32_t interpolation : 1;
        uint32_t double_edge : 1;
        uint32_t diss2g : 1;
        uint32_t diss2vs : 1;
    };
    uint32_t bytes;
};
static union ChopperConfig chopper_config;

void gpioInit();
void gpioUartInit();
void uartInit();
void tmc_write(uint8_t regAdress, uint32_t value);
uint8_t calcCRC(uint8_t datagram[], uint8_t len);
void set_vactual(int32_t value);
void set_ihold_irun(int8_t i_hold, uint8_t i_run, uint8_t i_hold_dela);
void set_direction(bool direction);
void set_microstep_input(bool input);
void set_microstep_resolution(uint8_t resolution);

void vact_gconf_control();
void only_vact_control();
void linear_acc_control();
void s_curve_acc_control();

//TMC2209 STEP, DIR, EN gpio initialization
void gpioInit(){
    gpio_init(STEP_PIN);
    gpio_init(DIR_PIN);
    gpio_init(EN_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_set_dir(EN_PIN, GPIO_OUT);
}

//TMC2209 UART pins initialization
void gpioUartInit(){
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
}

//TMC2209 UART initialization
void uartInit(){
    uart_init(TMC2209_UART, BAUDRATE);
    uart_set_translate_crlf((uart_inst_t *) TMC2209_UART, false);
    uart_set_format(TMC2209_UART, DATA_BITS, STOP_BIT, UART_PARITY_NONE);
    uart_set_fifo_enabled(TMC2209_UART, true);
}

//TMC2209 write bytes function
void tmc_write(uint8_t regAdress, uint32_t value){
    uint8_t len = 7;
    uint8_t frame[8];
    frame[0] = TMC2209_SYNC_BIT;
    frame[1] = DRIVER_1;
    frame[2] = regAdress | TMC2209_WRITE_BIT;
    frame[3] = (uint8_t)(value >> 24);
    frame[4] = (uint8_t)(value >> 16);
    frame[5] = (uint8_t)(value >> 8);
    frame[6] = (uint8_t)(value >> 0);
    frame[7] = calcCRC(frame, len);
    for(uint8_t i=0; i<=len; i++) {
        uart_putc(TMC2209_UART, frame[i]);
    }
    busy_wait_ms(3);
}

//8 bit CRC polynomial calculation
uint8_t calcCRC(uint8_t datagram[], uint8_t len){
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t currentByte = datagram[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (currentByte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            crc &= 0xff;
            currentByte = currentByte >> 1;
        }
    }
    return crc;
}

// Allows moving the motor by UART control
// The motor direction is controlled by the sign of VACTUAL
void set_vactual(int32_t value){
    tmc_write(ADRESS_VACTUAL, value);
}

// Set standstill current(0-31), motor run current(0-31) and motor power down cycles(1-15)
void set_ihold_irun(int8_t i_hold, uint8_t i_run, uint8_t i_hold_delay){
    driver_current.ihold = i_hold;
    driver_current.irun  = i_run;
    driver_current.iholddelay = i_hold_delay;
    tmc_write(ADRESS_IHOLD_IRUN, driver_current.bytes);
}

// Allows to inverse motor direction via GCONF register
void set_direction(bool direction){
    driver_config.shaft = direction;
    tmc_write(ADDRESS_GCONF, driver_config.bytes);
}

// Don't work
// Allows to choose microstep resolution input (0 - by MS1&MS2 pins, 1 - by MSTEP register)
void set_microstep_input(bool input){
    driver_config.mstep_reg_select = input;
    tmc_write(ADDRESS_GCONF, driver_config.bytes);
}

// Don't work
// Allows to choose microstep resolution (0-256,1-128, etc)
void set_microstep_resolution(uint8_t resolution){
    chopper_config.mres = resolution;
    tmc_write(ADRESS_CHOPCONF, chopper_config.bytes);
}


//Motor control via VACTUAL and GCONF register
void vact_gconf_control(){
    bool direction = true;
    while (true)
    {
        set_direction(direction);
        set_vactual(1000);
        sleep_ms(1000);
        direction = !direction;
    }
}

//Motor control only via VACTUAL register
void only_vact_control(){
    while (true)
    {
        set_vactual(1000);
        sleep_ms(1000);
        set_vactual(-1000);
        sleep_ms(1000);
    }
}

//Motor control with acceleration via VACTUAL register
void linear_acc_control(){
    while (true)
    {
        for(uint16_t i = 0; i <= 5000; i++){
            set_vactual(i);
            printf("%d\n", i);
            sleep_us(100);
        }
        for(uint16_t i = 5000; i > 0; i--){
            set_vactual(i);
            printf("%d\n", i);
            sleep_us(100);
        }
    }
}

// Still in progress
void s_curve_acc_control(){

}

int main(){
    stdio_init_all();
    gpioInit();
    gpioUartInit();
    uartInit();
    set_ihold_irun(16, 16, 8);
    //set_microstep_input(true);
    //set_microstep_resolution(MICROSTEP_RESOLUTION_64);

    //Motor control via VACTUAL and CONFIG register
    //vact_gconf_control();

    //Motor control only via VACTUAL register
    //only_vact_control();
    
    //Motor control with linear acceleration via VACTUAL register
    linear_acc_control();

    //Motor control with s-curve acceleration via VACTUAL register
    //s_curve_acc_control();

    /*
    while(true){
        printf("Motor direction - %d\n", direction);
        for(uint16_t i = 0; i <= 1600; i++){
            printf("Step number - %d\n", i);

            gpio_put(STEP_PIN, 1);
            sleep_us(250);
            gpio_put(STEP_PIN, 0);
            sleep_us(250);
        }
        direction = !direction;
        gpio_put(EN_PIN, direction);
    }
     */
}