#define TinyFOC_DISABLE_DEBUG
#include "TinyFOC.h"

int main(void)
{
    FOCMotor mymotor;
    BLDCMotor_load_default(&mymotor);

    CurrentSense mycs;
    CurrentSense_load_default(&mycs);

    FOCDriver mydriver;
    FOCDriver_load_default(&mydriver);

    Sensor mysensor;
    Sensor_load_default(&mysensor);

    mymotor.linkCurrentSense(&mymotor, &mycs);
    mymotor.linkDriver(&mymotor, &mydriver);
    mymotor.linkSensor(&mymotor, &mysensor);
}