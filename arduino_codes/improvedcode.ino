// Removing floating point operations with fixed point one using scaling factor and optimizing the 
// code for faster and better response.

// SCALING FACTOR = 10000;

#include <avr/io.h>
#include <avr/interrupt.h>

// Define constants
#define F_CPU 16000000UL // Assuming a 16 MHz clock
#define VREF 360000       // Voltage reference in volts
#define KP1 00288        // Proportional gain for first PI controller
#define KI1 65613          // Integral gain for first PI controller
#define KP2 12030          // Proportional gain for second PI controller
#define KI2 16151          // Integral gain for second PI controller
#define PWM_FREQ 8000   // PWM frequency in Hz

// Flag to indicate when to start PID control
volatile bool startPID = false;

// Variables for PI controllers
volatile uint32_t voltageError = 00000;
volatile uint32_t currentError = 00000;
volatile uint32_t integral1 = 00000;
volatile uint32_t integral2 = 00000;
volatile uint32_t piOutput1 = 00000;
volatile uint32_t piOutput2 = 00000;
volatile uint32_t currentReference = 00000;
volatile uint32_t dutyCycle = 00000;

// Initialize ADC
void adc_init() {
    ADMUX = (1 << REFS0); // AVcc with external capacitor at AREF pin
    ADCSRA = (1 << ADEN)  // Enable ADC
           | (1 << ADPS2)| (1 << ADPS0); // Prescaler of 128(each conversion will take 104us, infeasable)
           // so modified to prescalar of 32(conversion will take 26us each);
}

// Read ADC value from specified channel
uint16_t adc_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Selecting ADC channel
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Waiting for conversion to complete
    return ADC;
}

// Initialize Timer1 for PWM of 10kHz
void timer1_init() {
    TCCR1A = (1 << COM1A1) | (1 << WGM11);  // Non-inverted PWM, Mode 14 (Fast PWM)
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // No prescaler
    ICR1 = (F_CPU / PWM_FREQ) - 1; // Set top value for 10kHz PWM
    DDRB |= (1 << PB5); // Setting PB5 or DDB5 => OC1A pin as output, so we will use OCR1A reg as 
                        // the output compare reg. for generating compare match event.
}

// Initializing Timer0 for periodic sampling interrupt
void timer0_init() {
    TCCR0A = (1 << WGM01); // CTC mode
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64
    OCR0A = 30; // Compare match value for 8kHz sampling rate
    TIMSK0 = (1 << OCIE0A); // Enable compare match interrupt with OCR0A register
}

ISR(TIMER0_COMPA_vect) {

    static uint32_t count = 0;
    if (count < 50000) { // 10kHz * 5 seconds = 50000
        count++;
        // Set constant duty cycle
        OCR1A = (5 * ICR1) / 10; // 50% duty cycle
        // if((count % 100) == 0)
        // {
        // Serial.println("DutyCycle = 0.5");
        // }
        return;
    } else {
        startPID = true;
    }

    if (startPID) {
      
    // Read voltage and current
    uint32_t R1 = 195600; 
    uint32_t Req = 10295600;
    uint32_t sensedVoltage = adc_read(0) * (50000UL * Req) / (1023UL * R1); //Can Adjust scaling as needed
    uint32_t rawVoltage = (adc_read(1) * 50000UL) / 1023;  // THIS IS FOR THE CURRENT SENSOR //Can Adjust scaling as needed
    uint32_t sensedCurrent = (rawVoltage-25000)*10; 


    // Serial.print(sensedCurrent);
    // Serial.println(" A");
    // Serial.print(sensedVoltage);
    // Serial.println(" V");

    // First PI controller for voltage
    voltageError = VREF - sensedVoltage;
    integral1 += voltageError; 
    piOutput1 = (KP1 * voltageError) + (KI1 * integral1);

    // Set current reference
    currentReference = piOutput1;

    // Second PI controller for current
    currentError = currentReference - sensedCurrent;
    integral2 += currentError;
    piOutput2 = (KP2 * currentError) + (KI2 * integral2);

    // Calculate duty cycle
    dutyCycle = piOutput2;
    if (dutyCycle > 9800) dutyCycle = 9800;
    if (dutyCycle < 200) dutyCycle = 200;

    // Set PWM duty cycle
    OCR1A = dutyCycle * ICR1 / 10000;
    // Serial.println(dutyCycle);
  }
    }

int main(void) {
    adc_init();
    timer1_init();
    timer0_init();

    // Serial.begin(9600);

    sei(); // Enable global interrupts

    while (1) {
        // Main loop does nothing, control is in ISR
    }
}