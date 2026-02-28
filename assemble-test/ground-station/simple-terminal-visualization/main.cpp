#include <stdio.h>
#include "data-receive-and-decomposition/data_receive_and_decompose.h"
#include "data-receive-and-decomposition/serialib.h"

serialib serial;

int main(){
    if (serial.openDevice("COM1", 115200)!=1) return 1;
    
}