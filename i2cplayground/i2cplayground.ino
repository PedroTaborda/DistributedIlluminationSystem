#include <Wire.h>

#include <pico/unique_id.h>

#define NUM_BUFFERS 1000
#define BUFFER_SIZE 100

char buffer[NUM_BUFFERS][BUFFER_SIZE];
volatile bool newValues;
int counter;
int current_buffer, previous_written_buffer;
pico_unique_board_id_t id;

void onRecv(int len) {
    for(int i = 0; i < len; i++)
        buffer[current_buffer][i] = Wire1.read();

    buffer[current_buffer][len] = '\0';
    current_buffer = (current_buffer + 1) % NUM_BUFFERS;
    newValues = true;
}

void setup() {
    Serial.begin(115200);

    counter = 0;
    current_buffer = 0;
    previous_written_buffer = -1;
    newValues = false;
    pico_get_unique_board_id(&id);

    delay(1000);

    Wire.setSDA(0);
    Wire.setSCL(1);
    Wire1.setSDA(2);
    Wire1.setSCL(3);
    Wire.begin();
    Wire1.begin(0x40);
    Wire1.onReceive(onRecv);
}

void loop() {
    Wire.beginTransmission(0x0);
    Wire.write(id.id, 8);
    Wire.write(counter);

    int ret = Wire.endTransmission();
    if(ret == 0)
        counter++;

    if(newValues) {
        while(previous_written_buffer != current_buffer - 1) {
            previous_written_buffer = (previous_written_buffer + 1) % NUM_BUFFERS;
            uint64_t *unique;
            int *aux;
            unique = (uint64_t*)&buffer[previous_written_buffer][0];
            aux = (int *)&buffer[previous_written_buffer][8];
            Serial.print(*unique);
            Serial.print(' ');
            Serial.println(*aux);
        }
        newValues = false;
    }
}
