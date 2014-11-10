
#ifndef SONARREADING_H_
#define SENSORREADING_H_


// How many characters to hold the decimal value
#define SENSOR_READING_SIZE				16		
#define SENSOR_READING_DELIMITER		':'

// Structure for holding sonar readings
typedef struct
{
	char name;
	char delimiter = SENSOR_READING_DELIMITER;
	char data[SENSOR_READING_SIZE];
} SensorReading;


#endif
