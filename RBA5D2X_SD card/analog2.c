#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
//#include <wiringPi.h>

#define ANALOG_PIN 1  // Replace with the actual analog pin number on the micro bus

float analogToTemperature(int analogValue) {
    float R1 = 10000.0;
    float logR2, R2, T, K;
    float c1 = 0.001129148, c2 = 0.000234125, c3 = 0.0000000876741;

    R2 = R1 * (1023.0 / (float)analogValue - 1.0);
    logR2 = log(R2);
    K = 1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2);
    T = K - 273.15; // Convert to Celsius

    return T;
}

int main() {
    if (wiringPiSetup() == -1) {
        fprintf(stderr, "Unable to initialize wiringPi\n");
        return 1;
    }

    while (1) {
        // Read analog value from the analog pin on the micro bus
        int analogValue = analogRead(ANALOG_PIN);

        float temperature = analogToTemperature(analogValue);
        printf("Analog Value: %d, Temperature: %.2f\n", analogValue, temperature);

        // Adjust the sleep duration based on your sensor's update rate
        sleep(1);  // Sleep for 1 second between readings
    }

    return 0;
}

