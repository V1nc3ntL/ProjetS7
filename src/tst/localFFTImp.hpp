#ifndef __LOCAL_FFT_IMP_HH__
#define __LOCAL_FFT_IMP_HH__
#include "rtos.h"

// Longueur de FFT
#define FFT_LEN 256

/*Fréquence d'échantillonage
  Fréquence max à échantillonner 2kHz*/
#define SAMPLING_FREQ 4000


// Échantillons sonores
static float samples[FFT_LEN*2];
// Thread pour capture sonore          
static Thread thread1;

// Fonction d'interruption pour l'échantillonnage
void sampling_interrup();
/* Associe la fonction d'interruption et 
 * initialise les différentes variables nécessaires */
void samplingBegin();
/* Indique l'état de l'échantillonnage */
bool samplingDone();
/* Fonction d'échantillonnage */
void microRead();

#endif