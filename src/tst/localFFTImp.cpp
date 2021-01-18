#include "localFFTImp.hpp"
#include "mbed.h"


// Indice de parcours
int i = 0;
// Entrée du micro
AnalogIn micro_bee(A0);
//Tick pour capture sonore
LowPowerTicker sampler;

//Mutex pour assurer l'unicité de la lecture et sauvegarde des valeurs
Mutex read_mutex;

void microRead()
{
    // Function to protect read() of AnalogIn with thread and mutex
    while(1) {
        read_mutex.lock();
        samples[i] = 1024*micro_bee.read(); // 1024 : ADC of the sound sensor
        read_mutex.unlock();
        ThisThread::sleep_for(1/SAMPLING_FREQ);    // in seconds
    }
}


void sampling_interrup()
{
    i++;
    if (i >= FFT_LEN*2) {
        sampler.detach();
    }
}

void samplingBegin()
{
    // Reset sample buffer position and start callback at the sampling frequency
    i = 0;
    sampler.attach_us(&sampling_interrup, 1000000/SAMPLING_FREQ);   // ttes les 1 sec
}

bool samplingDone()
{
    return i >= FFT_LEN*2;
}
