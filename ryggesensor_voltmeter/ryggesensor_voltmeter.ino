

#include <LiquidCrystal.h>
#define DIST_SENSOR_ECHO_PIN 2
#define DIST_SENSOR_TRIGGER_PIN 4

#define LCD_VCC_PIN 3
#define LCD_RS_PIN 4
#define LCD_EN_PIN 5
#define LCD_D4_PIN 8
#define LCD_D5_PIN 9
#define LCD_D6_PIN 10
#define LCD_D7_PIN 11

#define ADC_PIN A0
#define ADC_AVERAGE_BUFFER_SIZE 5
unsigned int adc_buffer[ADC_AVERAGE_BUFFER_SIZE] = { 0 };
unsigned int adc_buffer_index = 0;
const float voltage_divider_factor = 0.02510f;
const unsigned long adc_sampling_delay_ms = 100;
unsigned long prev_sampling_time = 0;
float voltage_measurement = 0;

#define DISTANCE_AVERAGE_BUFFER_SIZE 3
unsigned int average_buffer[DISTANCE_AVERAGE_BUFFER_SIZE] = { 0 };
unsigned int average_index = 0;

const uint8_t time_to_cm_factor = 58;
unsigned long send_time_us = 0;
const unsigned int max_distance = 100; // cm
const unsigned int min_distance = 5; // cm
unsigned int distance = 0;
unsigned int average_distance = 0;

LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

void distance_sensor_callback();
void sample_adc();
void print_voltage();


void setup() 
{
	// Disable interrupts
	cli(); 

	// Setting up timer unterrupt to trigger distance sensor
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;
	TCCR1B |= (1 << WGM12);
	TCCR1B |= (1 << CS12) | (1 << CS10);
	OCR1A = 1561; // Around 10 Hz
	TIMSK1 |= (1 << OCIE1A);

	DDRB |= (1 << DIST_SENSOR_TRIGGER_PIN); // Sets pin D12 to output
	PORTB &= ~(1 << DIST_SENSOR_TRIGGER_PIN); // Sets D12 to 0

	DDRD &= ~(1 << DIST_SENSOR_ECHO_PIN); // Setting pin D2 to input 
	attachInterrupt(digitalPinToInterrupt(DIST_SENSOR_ECHO_PIN), distance_sensor_callback, CHANGE);

	DDRD |= (1 << LCD_VCC_PIN); // Setting pin D3 to output
	PORTD |= (1 << LCD_VCC_PIN); // Setting pin D3 high

	lcd.begin(16, 2); // Setting the size to 16x2 characters
	lcd.print("Voltage:");

	sei(); // Enable interrupts
}

void loop() 
{
	if(millis() - prev_sampling_time >= adc_sampling_delay_ms)
	{
		prev_sampling_time = millis();

		sample_adc();
		print_voltage();
	}
}

ISR(TIMER1_COMPA_vect)
{
	PORTB |= (1 << DIST_SENSOR_TRIGGER_PIN);

	delayMicroseconds(11); // A delay to trigger the distance sensor according to data sheet

	PORTB &= ~(1 << DIST_SENSOR_TRIGGER_PIN);
}

// A callback to calculate the distance gicen from the sensor
void distance_sensor_callback()
{
	// Checking if interrupt was triggered by positive edge
	if(PIND & (1 << DIST_SENSOR_ECHO_PIN))
	{
		send_time_us = micros();
		return;
	}

	unsigned long time_diff = micros() - send_time_us;
	distance = time_diff / time_to_cm_factor;	// Rounding to cm
	if(distance < min_distance)
	{
		distance = min_distance;
	}
	else if(distance > max_distance)
	{
		distance = max_distance;
	}
	
	average_buffer[average_index++] = distance;
	if(average_index >= DISTANCE_AVERAGE_BUFFER_SIZE)
	{
		average_index = 0;
	}

	average_distance = 0;
	for(uint8_t i = 0; i < DISTANCE_AVERAGE_BUFFER_SIZE; ++i)
	{
		average_distance += average_buffer[i];
	}
	average_distance /= DISTANCE_AVERAGE_BUFFER_SIZE;
	
	//lcd.setCursor(0, 1);
	//lcd.print("    ");
	//lcd.setCursor(0, 1);
	//lcd.print(average_distance);
	//lcd.print("cm");
}

void sample_adc()
{
	adc_buffer[adc_buffer_index++] = analogRead(ADC_PIN);
	if(adc_buffer_index >= ADC_AVERAGE_BUFFER_SIZE)
	{
		adc_buffer_index = 0;
	}

	voltage_measurement = 0;
	for(uint8_t i = 0; i < ADC_AVERAGE_BUFFER_SIZE; ++i)
	{
		voltage_measurement += (float)adc_buffer[i];
	}

	voltage_measurement /= ADC_AVERAGE_BUFFER_SIZE;
	voltage_measurement *= voltage_divider_factor;
}

void print_voltage()
{
	lcd.setCursor(0, 1);
	char measurement[6];
	//snprintf(measurement, 6, "%0.2fV", voltage_measurement);
	lcd.print("       ");
	lcd.setCursor(0, 1);
	lcd.print(voltage_measurement, 2);
	lcd.print("V");
}