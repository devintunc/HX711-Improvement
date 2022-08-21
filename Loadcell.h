#pragma once

#include "BaseDataSndr.h"
#include "HX711.h"
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

#ifndef SampleCount
#define SampleCount 30
#endif

extern const int LOADCELL_DOUT_PIN;
extern const int LOADCELL_SCK_PIN;

static HX711 scale;

// Loadcell object only sends data ModeWeight and ModePullup.
class Loadcell
{
private:
    bool isBegin;

    long isOnloadThreshold;
    // tare parameters       
    float tareMax_C;
    float tareMin_C;

    float safetyFactor;
   
    bool tareMaxMin_once;
    int i = 1;

    // calibration factor calculations parameters
    float KnownWeight;
    float calfac;
 
    // measurement parameters
    long Measurements[SampleCount];
    long weight;
    long *MinMax; // first element min second element max value.
    // timing paramaters
    unsigned long Last_time = 0;
    bool FirstTime = false;
    ;
    // filtering paramaters

    float avg = 0;
    unsigned long total;

    // control paramaters
    bool samplingActive;
    bool tareActive;
    bool calActive;
    bool isOnLoad;
    bool isCleanable;
    bool CalMode;
    // Non-Blocking Sampling control paramaters
    bool isSampled;    
    // Internal methods

    long RegulatedRead();
    long *Detect_MinMax(long *, int);
    float DVN_Correction(long *arr, int arrsize, long minMeasurement, long maxMeasurement);

    void EEpromWrite(int StartAdress, int Lenght, long val);


long EEpromRead(int StartAdress, int Lenght);


public:
    // timing paramaters
    uint8_t SamplingPeriod;
    
        long tareMin; // 286885;
    long tareMax ; // 374522;
    //non-blocking functions result parameter
     bool isTared;
     bool isCalFacCalculated;
    Loadcell(/* args */);
    ~Loadcell();
    static Loadcell &GetInstance()
    {
        static Loadcell Instance;
        return Instance;
    }
    void Init();
    bool onLoad();

    void Tare();  
    void Tare_MaxMin();
    void Tare_Init();
    void Tare_Complete();
   
    void Sample();
    void Sampler();
    void SamplerComplete();
    bool isSamplingDone();
    float CalculateWeight();

    void CalculateCalFactor();

    void SamplingActive(bool activity, bool calMode);
    void TareActive();
    void CalculateCalFactorActive();

    bool isCalibrated();
    void CleanParameters();

    void MainLoopOperations();
};
