#include <avr/io.h>
#include <avr/interrupt.h>

// Define constants
#define F_CPU 16000000UL // Assuming a 16 MHz clock
#define VREF 36.0000       // Voltage reference in volts
#define KP1 0.028847         // Proportional gain for first PI controller
#define KI1 6.5613          // Integral gain for first PI controller
#define KP2 1.203          // Proportional gain for second PI controller
#define KI2 1.6151          // Integral gain for second PI controller
#define PWM_FREQ 10000   // PWM frequency in Hz

// Flag to indicate when to start PID control
volatile bool startPID = false;

// Variables for PI controllers
volatile float voltageError = 0.0000;
volatile float currentError = 0.0000;
volatile float integral1 = 0.0000;
volatile float integral2 = 0.0000;
volatile float piOutput1 = 0.0000;
volatile float piOutput2 = 0.0000;
volatile float currentReference = 0.0000;
volatile float dutyCycle = 0.0000;

// Initialize ADC
void adc_init() {
    ADMUX = (1 << REFS0); // AVcc with external capacitor at AREF pin
    ADCSRA = (1 << ADEN)  // Enable ADC
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler of 128
}

// Read ADC value from specified channel
uint16_t adc_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Select ADC channel
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
    return ADC;
}

// Initialize Timer1 for PWM
void timer1_init() {
    TCCR1A = (1 << COM1A1) | (1 << WGM11);  // Non-inverted PWM, Mode 14 (Fast PWM)
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // No prescaler
    ICR1 = (F_CPU / PWM_FREQ) - 1; // Set top value for 10kHz PWM
    DDRB |= (1 << PB5); // Set PB5/OC1A as output
}

// Initialize Timer0 for periodic sampling interrupt
void timer0_init() {
    TCCR0A = (1 << WGM01); // CTC mode
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64
    OCR0A = (F_CPU / (64 * 10000)) - 1; // Compare match value for 10kHz sampling rate
    TIMSK0 = (1 << OCIE0A); // Enable compare match interrupt
}

ISR(TIMER0_COMPA_vect) {

    static uint32_t count = 0;
    if (count < 50000) { // 10kHz * 5 seconds = 50000
        count++;
        // Set constant duty cycle
        OCR1A = 0.2 * ICR1; // 50% duty cycle
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
    float R1 = 19.5600; 
    float Req = 1029.5600;
    float sensedVoltage = adc_read(0) * (5.0000 / 1023.0000) * (Req/R1); // Adjust scaling as needed
    float rawVoltage = adc_read(1) * (5.0000 / 1023.0000);  // THIS IS FOR THE CURRENT SENSOR // Adjust scaling as needed
    float sensedCurrent = ((rawVoltage-2.5000)/0.1000); 


    // Serial.print(sensedCurrent);
    // Serial.println(" A");
    // Serial.print(sensedVoltage);
    // Serial.println(" V");

    // First PI controller for voltage
    voltageError = VREF - sensedVoltage;
    integral1 += voltageError * 0.0001; // 100us sampling time
    piOutput1 = (KP1 * voltageError) + (KI1 * integral1);

    // Set current reference
    currentReference = piOutput1;

    // Second PI controller for current
    currentError = currentReference - sensedCurrent;
    integral2 += currentError * 0.0001; // 100us sampling time
    piOutput2 = (KP2 * currentError) + (KI2 * integral2);

    // Calculate duty cycle
    dutyCycle = piOutput2;
    if (dutyCycle > 0.98) dutyCycle = 0.98;
    if (dutyCycle < 0.02) dutyCycle = 0.02;

    // Set PWM duty cycle
    OCR1A = dutyCycle * ICR1;
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
