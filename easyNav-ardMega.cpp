
// FreeRTOS header files
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "croutine.h"

// Arduino library header files
#include "Arduino.h"
#include "HardwareSerial.h"

// Protocol and other header files
#include "sprotapi.h"
#include "SensorReading.h"
#include "myutil.h"


#define SONAR_COUNT					2
#define SONAR_LEFT_ID				0
#define SONAR_RIGHT_ID				1

// How often to read each sonar in Hz (Take note : EZ2 and EZ4 sonar maximum frequency is 20 Hz)
#define SONAR_READING_FREQUENCY		2
#define SONAR_READING_PERIOD_MS		1000 / SONAR_READING_FREQUENCY
#define SONAR_WAIT_PERIOD_MS		50

// ADC-Parameters
#define MICROVOLTS_PER_CM			3900
#define MICROVOLTS_PER_UNIT			4900
//#define DISTANCE_SCALE_FACTOR		MICROVOLTS_PER_UNIT / MICROVOLTS_PER_CM
#define DISTANCE_SCALE_FACTOR		13

// ADC channel to Sonar Mappings
#define SONAR_LEFT_ADC_CHANNEL		A0
#define SONAR_RIGHT_ADC_CHANNEL		A7
#define SONAR3_ADC_CHANNEL			A13

// Sonar output enable pins
#define SONAR_LEFT_ENABLE_PIN		13
#define SONAR_RIGHT_ENABLE_PIN		12
#define SONAR3_ENABLE_PIN			11

// Sonar power pins
#define SONAR_LEFT_POWER_PIN		7
#define SONAR_RIGHT_POWER_PIN		6

// Task parameters
#define STACK_SIZE					150
#define TASK_PRIORITY				tskIDLE_PRIORITY + 1

// Queue parameters
#define QUEUE_LENGTH				1
#define QUEUE_ITEM_SIZE				sizeof(SensorReading)
#define QUEUE_WAIT_TIME				portMAX_DELAY

// Compass parameters
#define COMPASS_MAX_READING_SIZE		4
#define COMPASS_SERIAL_PORT				Serial3
#define	COMPASS_READING_PERIOD_MS		500

#define FOOTSENS_READING_SIZE			7
#define FOOTSENS_MAX_READING_SIZE		10
#define FOOTSENS_SERIAL_PORT			Serial2
#define	FOOTSENS_READING_PERIOD_MS		10

// Serial port parameters
#define RPI_SERIAL_PORT_BAUDRATE		115200
#define COMP_SERIAL_PORT_BAUDRATE		57600
#define FOOTSENS_SERIAL_PORT_BAUDRATE	57600

// Averaging filter settings
#define AVG_FILTER_SAMPLE_SIZE		3
#define DEFAULT_BUFFER_SIZE			20

// For delay functions
const TickType_t SONAR_WAIT_PERIOD_MS_TICK_INCREMENT = SONAR_WAIT_PERIOD_MS;
TickType_t lastTickValue;

// Semaphore for maintaining mutual exclusion of ultrasonic transmission
SemaphoreHandle_t SEMA_SONAR;

// Queue for containing sonar readings
QueueHandle_t ARR_SONAR_QUEUE[SONAR_COUNT];
QueueHandle_t compassQueue;
QueueHandle_t footsensQueue;

// Stores the last read value from each sonar
volatile int ARR_SONAR_SAMPLES[SONAR_COUNT][AVG_FILTER_SAMPLE_SIZE] = {{0},{0}};
volatile int ARR_SONAR_READ_COUNT[SONAR_COUNT] = {0};

// Temporary buffers for serial despatcher
char itemBuffer1[DEFAULT_BUFFER_SIZE] = {0};
char itemBuffer2[DEFAULT_BUFFER_SIZE] = {0};

// Sensor readings structure for each sensor
SensorReading srLeft;
SensorReading srRight;
SensorReading compassReading;
SensorReading footsensReading;

// Arduino setup function : init(), serial port configuration etc.
void setup(void);

// The tasks to be run
void sonarLeft(void * args);

void sonarRight(void * args);

void sonar3(void * args);

// Reads from the sonar readings queue and sends the data over serial communications
void serialDespatcher(void * args);

void readAndEnqueueSonarReading(uint8_t sonarId, SensorReading * sr, uint8_t sonarADCChannel, uint8_t sonarEnablePin);

void readAndEnqueueCompassReading(void * args);

void readAndEnqueueFootsensReading(void * args);

// Averaging filter to smooth out erratic sonar readings
int applyAveragingFilter(uint8_t sonarId, int sonarReading);


int main(void)
{
	setup();
	
	xTaskCreate(sonarLeft, "snlft", STACK_SIZE, NULL, TASK_PRIORITY, NULL);
	
	xTaskCreate(sonarRight, "snrgt", STACK_SIZE, NULL, TASK_PRIORITY, NULL);
	
	xTaskCreate(readAndEnqueueCompassReading, "cmprd", STACK_SIZE, NULL, TASK_PRIORITY, NULL);
	
	xTaskCreate(readAndEnqueueFootsensReading, "ftsrd", STACK_SIZE, NULL, TASK_PRIORITY, NULL);
	
	xTaskCreate(serialDespatcher, "srdsp", STACK_SIZE, NULL, TASK_PRIORITY, NULL);
	
	vTaskStartScheduler();
	
	return 0;
}


void setup(void)
{
	// Arduino initialization for some common functions
	init();
	
	// Select the voltage source to be used as the ADC reference
	analogReference(DEFAULT);
	
	// Set analog pins to input mode
	pinMode(SONAR_LEFT_ADC_CHANNEL, INPUT);
	pinMode(SONAR_RIGHT_ADC_CHANNEL, INPUT);
	pinMode(SONAR3_ADC_CHANNEL, INPUT);
	
	// Set sonar enable pins to output
	pinMode(SONAR_LEFT_ENABLE_PIN, OUTPUT);
	pinMode(SONAR_RIGHT_ENABLE_PIN, OUTPUT);
	pinMode(SONAR3_ENABLE_PIN, OUTPUT);
	
	// Set all power outputs to output and HIGH
	pinMode(SONAR_LEFT_POWER_PIN, OUTPUT);
	pinMode(SONAR_RIGHT_POWER_PIN, OUTPUT);
	digitalWrite(SONAR_LEFT_POWER_PIN, HIGH);
	digitalWrite(SONAR_RIGHT_POWER_PIN, HIGH);
	
	
	// Make all sonar enable lines low to disable all sonars
	digitalWrite(SONAR_LEFT_ENABLE_PIN, LOW);
	digitalWrite(SONAR_RIGHT_ENABLE_PIN, LOW);
	//digitalWrite(SONAR3_ENABLE_PIN, HIGH );
	
	// Serial port and protocol initialization
	COMPASS_SERIAL_PORT.begin(COMP_SERIAL_PORT_BAUDRATE, SERIAL_8N1);
	FOOTSENS_SERIAL_PORT.begin(FOOTSENS_SERIAL_PORT_BAUDRATE, SERIAL_8N1);
	SPROTInit(RPI_SERIAL_PORT_BAUDRATE);
	Serial.begin(115200, SERIAL_8N1);
	
	// Create a queue for each sonar
	for(int i=0; i<SONAR_COUNT; i++)
	{
		ARR_SONAR_QUEUE[i] = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
	}
	
	// Create a queue for compass readings
	compassQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
	// Create a queue for foot sensor readings
	footsensQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
	
	SEMA_SONAR = xSemaphoreCreateBinary();
	xSemaphoreGive(SEMA_SONAR);
}


void readAndEnqueueCompassReading(void * args)
{
	char rxBuffer[COMPASS_MAX_READING_SIZE];
	char readValue;
	int bytesRead;
	bool validReading;
	
	compassReading.name = 'C';
	
	while(true)
	{
		// Wait for a serial read to be aligned with a compass reading
		do
		{
			readValue = (char) COMPASS_SERIAL_PORT.read();
		} while (readValue != '\n');
		
		// Double check if data was indeed in correct format
		bytesRead = COMPASS_SERIAL_PORT.readBytes(rxBuffer, COMPASS_MAX_READING_SIZE);
		
		validReading = (bytesRead == COMPASS_MAX_READING_SIZE) &&
		((rxBuffer[0] >= '0') && (rxBuffer[0] <= '9')) &&
		((rxBuffer[1] >= '0') && (rxBuffer[1] <= '9')) &&
		((rxBuffer[2] >= '0') && (rxBuffer[2] <= '9')) &&
		 (rxBuffer[3]  == '\r');
		
		if(!validReading)
		{
			Serial.println("error");
			continue;
		}
		else
		{
			memcpy(compassReading.data, rxBuffer, 3);
			xQueueOverwrite(compassQueue, &compassReading);
			//vTaskDelay(COMPASS_READING_PERIOD_MS);
		}
	}
}


void readAndEnqueueFootsensReading(void * args)
{
	char rxBuffer[FOOTSENS_MAX_READING_SIZE];
	char readValue;
	int bytesRead;
	bool validReading;
	
	footsensReading.name = 'F';
	
	while(true)
	{
		// Wait for a serial read to be aligned with a footsensor reading
		do
		{
			readValue = (char) FOOTSENS_SERIAL_PORT.read();
		} while (readValue != ',');
				
		bytesRead = FOOTSENS_SERIAL_PORT.readBytes(rxBuffer, FOOTSENS_MAX_READING_SIZE-1);
		//Serial.println(rxBuffer);
		// Double check if data was indeed in correct format
		validReading = (bytesRead == FOOTSENS_MAX_READING_SIZE-1) && (rxBuffer[FOOTSENS_MAX_READING_SIZE-2]  == '\n');
		
		validReading = true;
		if(!validReading)
		{
			Serial.println("error");
			continue;
		}
		else
		{
			memcpy(footsensReading.data, rxBuffer, FOOTSENS_READING_SIZE);
			xQueueOverwrite(footsensQueue, &footsensReading);
			//vTaskDelay(FOOTSENS_READING_PERIOD_MS);
		}
	}
}


void readAndEnqueueSonarReading(uint8_t sonarId, SensorReading * sr, uint8_t sonarADCChannel, uint8_t sonarEnablePin)
{
	int sonarReading;

	// Maintain exclusivity of sonar enable lines - only 1 sonar at a time may be active
	xSemaphoreTake(SEMA_SONAR, portMAX_DELAY);
	
	// Enable only this sonar, wait for the next sonar wave transmission cycle
	digitalWrite(sonarEnablePin, HIGH);
	lastTickValue = xTaskGetTickCount();
	vTaskDelayUntil(&lastTickValue, SONAR_WAIT_PERIOD_MS_TICK_INCREMENT);
	
	// Read sonar from corresponding ADC channel, perform value adjustments as needed
	sonarReading = (analogRead(sonarADCChannel) * DISTANCE_SCALE_FACTOR) / 10;
	
	// Disable sonar immediately after reading
	digitalWrite(sonarEnablePin, LOW);
	
	xSemaphoreGive(SEMA_SONAR);
	
	sonarReading = applyAveragingFilter(sonarId, sonarReading);
	convertToDecimalString((char *)sr->data, sonarReading);
	
	// Enqueue sonar reading for transmission over serial line
	xQueueOverwrite(ARR_SONAR_QUEUE[sonarId], sr);
}


void serialDespatcher(void * args)
{
	BaseType_t q1Result, q2Result;
	
	while(true)
	{
		// Read sonar1 and sonar2 data, don't wait for queue if they are empty
		q1Result = xQueueReceive(ARR_SONAR_QUEUE[0], itemBuffer1, 0);
		q2Result = xQueueReceive(ARR_SONAR_QUEUE[1], itemBuffer2, 0);
		
		if(q1Result == pdTRUE)
		{
			// Send sonar data to RPi
			SPROTSend((byte_t *)itemBuffer1, 0, QUEUE_ITEM_SIZE, SPROT_SEND_TIMEOUT);
		}
		
		if(q2Result == pdTRUE)
		{
			SPROTSend((byte_t *)itemBuffer2, 0, QUEUE_ITEM_SIZE, SPROT_SEND_TIMEOUT);
		}
		
		
		// Send compass data
		if(xQueueReceive(compassQueue, itemBuffer1, 0) == pdTRUE)
		{
			SPROTSend((byte_t *)itemBuffer1, 0, QUEUE_ITEM_SIZE, SPROT_SEND_TIMEOUT);
			//Serial.println(itemBuffer1);
		}
		
		
		// Send foot sensor data
		if(xQueueReceive(footsensQueue, itemBuffer1, 0) == pdTRUE)
		{
			SPROTSend((byte_t *)itemBuffer1, 0, QUEUE_ITEM_SIZE, SPROT_SEND_TIMEOUT);
			//Serial.println(itemBuffer1);
		}
		
	}
}


void sonarLeft(void * args)
{
	srLeft.name = '1';
	
	while(1)
	{
		readAndEnqueueSonarReading(SONAR_LEFT_ID, &srLeft, SONAR_LEFT_ADC_CHANNEL, SONAR_LEFT_ENABLE_PIN);
		vTaskDelay(SONAR_READING_PERIOD_MS);
	}
}

void sonarRight(void * args)
{
	srRight.name = '2';
	
	while(1)
	{
		readAndEnqueueSonarReading(SONAR_RIGHT_ID, &srRight, SONAR_RIGHT_ADC_CHANNEL, SONAR_RIGHT_ENABLE_PIN);
		vTaskDelay(SONAR_READING_PERIOD_MS);
	}
}


int applyAveragingFilter(uint8_t sonarId, int sonarReading)
{
	int readCount = ARR_SONAR_READ_COUNT[sonarId];
	int averagedSonarReading = 0;
	
	// Store the current sample
	ARR_SONAR_SAMPLES[sonarId][readCount] = sonarReading;
	readCount++;
	
	// Reset the index of the next sample to be overwritten (the oldest sample)
	if(readCount == AVG_FILTER_SAMPLE_SIZE)
	{
		ARR_SONAR_READ_COUNT[sonarId] = 0;
	}
	else
	{
		ARR_SONAR_READ_COUNT[sonarId] = readCount;
	}
	
	// Perform averaging over latest N samples
	for(int i=0; i<AVG_FILTER_SAMPLE_SIZE; i++)
	{
		averagedSonarReading += ARR_SONAR_SAMPLES[sonarId][i];
	}
	averagedSonarReading /= AVG_FILTER_SAMPLE_SIZE;
	
	return averagedSonarReading;
}
