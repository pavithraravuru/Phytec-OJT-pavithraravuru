#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#define DIGITAL_PIN "/sys/class/gpio/gpio23/value"  // Replace with the actual GPIO pin for the ball switch

void error(const char *msg) {
    perror(msg);
    exit(1);
}

bool readDigitalValue() {
    FILE *fp = fopen(DIGITAL_PIN, "r");
    if (fp == NULL) {
        error("Error opening digital pin");
    }

    char value;
    fscanf(fp, " %c", &value);  // Leading space to skip any leading whitespace
    fclose(fp);

    // '1' indicates the presence of a signal, '0' indicates no signal
    return (value == '1');
}

int main() {
    while (1) {
        bool ballSwitchState = readDigitalValue();

        if (ballSwitchState) {
            // If the ball switch is tilted, turn on the onboard LED (assuming it's GPIO 23)
            FILE *ledFile = fopen("/sys/class/gpio/gpio23/value", "w");
            if (ledFile != NULL) {
                fprintf(ledFile, "1");
                fclose(ledFile);
            } else {
                error("Error opening LED pin");
            }
        } else {
            // If the ball switch is not tilted, turn off the onboard LED
            FILE *ledFile = fopen("/sys/class/gpio/gpio23/value", "w");
            if (ledFile != NULL) {
                fprintf(ledFile, "0");
                fclose(ledFile);
            } else {
                error("Error opening LED pin");
            }
        }

        // Adjust the sleep duration based on your sensor's update rate
        sleep(1);  // Sleep for 1 second between readings
    }

    return 0;
}

