// headers de bibliothèques C
#include "DS1820.h"
#include "mbed.h"
#include "DHT.h"
#include "WakeUp.h" 
#include "rtos.h"   // lib thread & mutex
#include "HX711.h"   // lib pour le capteur de poids

// headers de bibliothèques C++
#include <LowPowerTicker.h>
#include <ratio>
#include "fftReal.hpp"  // lib FFT

// header spécifiques à l'implémentation
#include "localFFTImp.hpp" 
#include "localSensors.hh"

//Temps minimum pour garantir l'envoi de données par Sigfox
#define LPWAN_LIMIT 6000

// Liaison sigfox
Serial sigfox(D1, D0); // tx, rx

// Liaison SERIE pour debug
#if DEBUG
Serial pc(USBTX, USBRX); // tx, rx
#endif

// Bus OneWire pour DS1820
OneWire     oneWire(D2);
// Sondes de température étanches
DS1820*     ds1820[SENSORS_NR]; 
// DHT 22 intérieur
DHT dhtI(D3, DHT22);
//DHT 22 intérieur
DHT dhtE(D4, DHT22);
// Pin pour PM
DigitalOut done(D5);
// Capteur de poids
HX711 Balance(D12,D11);

/* Variables représentant
 * expAmp : exposant 10 de l'amplification de la fréquence mesurée
 * expF : exposant 10 de la hauteur fréquence
 */
uint8_t expAmp = 0, expF =0;

int main()
{
    // ~~~~~~ Variables Température ~~~~~~~
    float tmp = 0,mod;
    int sensors_found = 0,  result = 0;
    
    int i = 0,j = 0;
    uint8_t  
    // Températures 
        tcI = 0, tcE = 0,
    // Humidités
        thI = 0, thE = 0;

    // Résultats de mesures de température
    float sonde[SENSORS_NR];

    // ~~~~~~ Variables Poids ~~~~~~~
    float valeur_poids;

    // ~~~~~~ Variables FFT ~~~~~~~
   // float tabFFT[5] = {0};    // 5 frequencies, init to 0
    int idxFFT = 0;
    bool fill_FFT_tab = 0;   // set to false
    int max1 = 1;   // avoid DC value (0 Hz)
    float valHz = 0;
    // CONFIG FFT
    Mikami::Complex fft_bins[FFT_LEN];
    Mikami::FftReal fft((int16_t)FFT_LEN);

#if DEBUG

    // D"tection des sondes  pour obtention adresses
    // mettre debug à 1 dans DS1820.cpp
    for(i = 0; i < SENSORS_NR; i++) {
        ThisThread::sleep_for(1000);
        if(!ds1820[i]->begin()) {
            delete ds1820[i];
            break;
        }
    }
    sensors_found = i;
    pc.printf("Found %d sensors.\r\n", sensors_found);

#else
    // Connaissance des sondes en dur
    for(i = 0 ; i < SENSORS_NR ; i ++ ) {
        ds1820[i] = new DS1820(&oneWire);
        for(j=0; j<8; j++) {
            ds1820[i]->addr[j] = addrs[i][j];
            ds1820[i]->present = true;
        }
    }
    sensors_found = SENSORS_NR;
#endif

    // Lance l'échantillonage
    thread1.start(microRead);
    while(1) {
        samplingBegin();
        // Récupération des données extérieures
        if(dhtE.readData() == 0) {
            tmp   = dhtE.ReadTemperature(CELCIUS)*2;
            tcI = (uint8_t) tmp;
            tmp   = dhtE.ReadHumidity()*2;
            thI = (uint8_t) tmp;
        }
        // ~~~~~~~ PARTIE CAPTEUR DE POIDS ~~~~~~~
        Balance.powerUp();
        valeur_poids = Balance.getGram();      // on récupère la valeur
        valeur_poids = valeur_poids *(-1.000);
        #if DEBUG
            pc.printf("\nPoids :%.2f\r\n", valeur_poids);        // Affichage du poids sur Putty
        #endif
        Balance.powerDown();
        
        // ~~~~~ PARTIE FFT ~~~~~

        for(i = 0; i < sensors_found; i++) {
            ds1820[i]->startConversion();   // start temperature conversion from analog to digital
            ThisThread::sleep_for(200);        // let DS1820s complete the temperature conversion
            result = ds1820[i]->read(sonde[i]);
          #if DEBUG
            pc.printf("temp[%d] = %3d%cC\r\n", i,  (int)(100*sonde[i]), 176);     // read temperature
          #endif
        }
        // Récupère température et humidité intérieures

        if(dhtI.readData() == 0) {
            tmp   = dhtI.ReadTemperature(CELCIUS)*2;
            tcI = (uint8_t) tmp;
            tmp   = dhtI.ReadHumidity()*2;
            thI = (uint8_t) tmp;
            #if DEBUG
                pc.printf("temp DHT Extérieur = %3.1f (int) = %d , hum = %3.1f (int) = %d\r\n", c, tcE, h, thE);     // read temperature
            #endif
        }

        // La FFT est prête
        if (samplingDone()) {
            
            /*453 
            359-345
            1k
            515Hz*/
            fft.Execute(samples,fft_bins);
            mod = 0;
            valHz = 29 * SAMPLING_FREQ / (FFT_LEN);
            mod = std::abs(fft_bins[29]);
            #if DEBUG
            pc.printf("\nMesures : ");
            pc.printf("\r\nAmplitude = %.2f\r\n", mod);
            pc.printf("Frequency = %.4f Hz\r\n\n", valHz);
            pc.printf("tabFFT[%d] = %.4f Hz\n", idxFFT, tabFFT[idxFFT]);
            #endif
            // Initialisation des exposants
            expAmp = 0 ;
            expF = 0;
            
            
            if(mod /10 >= 100) {
                expAmp = 1;
                while((mod /= 10) > 100)
                    expAmp++;
            }
            
            if(valHz /10 >= 100) {
                expF  = 1;
                while((valHz /= 10)> 100)
                    expF++;
            }
            
            // 
            if (idxFFT == 5)  { // wait for stabilization after switch on
                    fill_FFT_tab = 1;
                    idxFFT = 0; // reset idxFFT
                }
                idxFFT++;
            }
            
        // Concaténation des exposants et résultats
        expAmp <<= 4;
        expAmp += expF;
        result  = ((int) mod )<< 8 ;
        result+= (uint16_t) valHz;
        
        // Envoi des données
        sigfox.printf(  "AT$SF=%02X%02X%02X%02X%04X%04X%02X%02X%04X\r\n",
                        tcE,tcI,thE,thI,  (int)(100*sonde[GAUCHE]),
                        (int)(100*sonde[DROITE]),(uint8_t)valeur_poids*2,
                        expAmp,
                        (uint16_t)result
                     );

        //Attends 6 min      */
        thread1.terminate();
        hal_deepsleep();
        ThisThread::sleep_for(LPWAN_LIMIT);
        done = 1 ;
        // This code shall never be reached
        ThisThread::sleep_for(360000);
        done = 0;
    }
}
