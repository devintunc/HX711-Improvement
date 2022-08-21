#include "Loadcell.h"
#include "ModeWeight.h"
#include "ModePullUp.h"
#include "NonBlocking.h"
// eeprom adresses : calfac 6, taremin 7,8,9,10 taremax 11,12,13,14, isTared 15
const int LOADCELL_DOUT_PIN = 10;
const int LOADCELL_SCK_PIN = 11;

Loadcell::Loadcell(/* args */)
{
    isBegin = false;
    isCleanable = false;
    ////tare parameters
    tareActive = false;
    tareMaxMin_once = true;
    weight = 0;
    safetyFactor = 0.02;
    //       EEpromWrite(7,sizeof(tareMin),0);
    // EEpromWrite(11,sizeof(tareMax),0);
    tareMin = EEpromRead(7, sizeof(tareMin));
    tareMax = EEpromRead(11, sizeof(tareMax));
    isSampled = false;
    isTared = EEPROM.read(15);
    //// calibration factor calculations parameters
    KnownWeight = 5020;
    calfac = EEPROM.read(6);
    CalMode = false; // Default : calibration mode off.
    isCalFacCalculated = false;
    calActive = false;
    //// sampling
    samplingActive = false;
    SamplingPeriod = 90;

    isOnloadThreshold=80000;
}

Loadcell::~Loadcell()
{
}
void Loadcell::Init()
{
    if (!isBegin)
    {
        scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
        isBegin = true;
    }
}

// This function returns if reading value at tare max and min interval.Removes the tare min from raw data.
long Loadcell::RegulatedRead()
{
    long reading = scale.read();
    if (reading < tareMax && reading > tareMin)
    {
        reading = 0;
    }
    if (reading < tareMin)
    {
        // tareMin = reading;
        reading = 0;
    }

    if (reading != 0)
        return (reading - tareMin); // cropping the min tare value.
    else
        return -1;
}
long *Loadcell::Detect_MinMax(long *arr, int arrSize)
{
    bool Flock;
    Flock = true;
    static long MinMax_ARR[2];

    ///////////Calculate min and max reading reading.//////////////////
    for (int i = 0; i < arrSize; i++)
    {
        if (Flock)
        {
            MinMax_ARR[0] = arr[i];
            MinMax_ARR[1] = arr[i];
            Flock = false;
        }

        if (arr[i] > MinMax_ARR[1])
            MinMax_ARR[1] = arr[i];
        if (arr[i] < MinMax_ARR[0])
            MinMax_ARR[0] = arr[i];
    }
    Serial.print(" minW and maxW : ");
    Serial.print(MinMax_ARR[0]);
    Serial.print("  ");
    Serial.println(MinMax_ARR[1]);
    return MinMax_ARR;
    ///////////Calculate min and max reading reading.//////////////////
}
// This function correcting regulated data via removing tare interval proportionally to measurement interval respect to tare interval.
// It returns avarage value of the given array.
float Loadcell::DVN_Correction(long *arr, int arrsize, long minMeasurement, long maxMeasurement)
{

    long wDiff = maxMeasurement - minMeasurement;
    long tareDiff = tareMax - tareMin;
    float ripCoef = (float)tareDiff / (float)wDiff;
    float _total = 0;

    Serial.print(" wdiff: ");
    Serial.print(wDiff);
    Serial.print(" tareDiff: ");
    Serial.print(tareDiff);
    Serial.print(" tareMin: ");
    Serial.print(tareMin);
    Serial.print(" tareMax: ");
    Serial.print(tareMax);
    Serial.print(" ripCoef: ");
    Serial.println(ripCoef);

    for (int i = 0; i < arrsize; i++)
    {
        Measurements[i] = Measurements[i] - ((tareDiff)*ripCoef); //((Measurements[a] - minW) * ripCoef);

        _total += Measurements[i]; /// calfac;

        // Serial.print(Measurements[i]);
        // Serial.println(",");
    }
    return _total / arrsize;
}
// Calls Sample function repeatedly from non-blocking run function until sampling done.
void Loadcell::Sampler()
{
    NonBlockRun<Loadcell>(&Sample, &SamplerComplete, this, samplingActive, Last_time, SamplingPeriod, SampleCount, i);
}
// Push data that acquired with regulated read into the defined measurement array. Non-Blocking function.
void Loadcell::Sample()
{

    if (scale.is_ready())
    {
        weight = RegulatedRead();
        if (weight > 0)
        {
            Measurements[i] = weight;
            Serial.println(weight);
        }

        else
            i--;
    }
    else
        i--;
}
void Loadcell::SamplerComplete()
{
    isSampled = true;
    samplingActive = false;
}
bool Loadcell::isSamplingDone()
{
    return isSampled;
}

// This function measuring the weight with internal raw data cleaning functions. It is non-blocking and should be use in main loop.
//  Normally deactive and when the process done deactived automatically. It can be activated with calling MeasurePreciseActivate() function.
//  It is non-blocking when reading.
// Raw_CFI is raw or calibration factor included selection.True is CFI, false is raw.
float Loadcell::CalculateWeight()
{
    if (isSampled)
    {
        MinMax = Detect_MinMax(Measurements, SampleCount);
        avg = DVN_Correction(Measurements, SampleCount, *MinMax, *(MinMax + 1));

        samplingActive = false;
        i = 0;
        Last_time = 0;

        if (CalMode)
            return avg;
        else
        {
            Serial.print(" Weight is : ");
            Serial.print(avg / calfac);
            Serial.println("  gram.");
            return avg / calfac;
        }
    }
    else
        return -1;
}
void Loadcell::CleanParameters()
{
    Serial.println("Paramaters cleaned.");
    if (isCleanable)
    {

        weight = 0;
        avg = 0;
        total = 0;
    }
}
bool Loadcell::onLoad()
{
    bool _isOnload = false;
    delay(100);
    if (scale.is_ready())
    {
        weight = RegulatedRead();
        
        if (weight > isOnloadThreshold)
            _isOnload = true;
        else
            _isOnload = false;
    }

    return _isOnload;
}
// Non-Blocking function.
void Loadcell::Tare()
{
    NonBlockRun<Loadcell>(&Tare_MaxMin, &Tare_Complete, this, tareActive, Last_time, SamplingPeriod, SampleCount, i);
}
void Loadcell::Tare_Init()
{
    tareMin_C = tareMax_C;
    Serial.print("tareMin_c and tareMax_C  : ");
    Serial.print(tareMin_C);
    Serial.print("  ");
    Serial.println(tareMax_C);
}
void Loadcell::Tare_MaxMin()
{
    if (scale.is_ready())
    {
        float reading = scale.read();

        if (reading > tareMax_C)
            tareMax_C = reading;
        if (reading < tareMin_C)
            tareMin_C = reading;

        Once<Loadcell>(&Tare_Init, tareMaxMin_once, this);
    }
    else
    {
        i--;
    }
}

void Loadcell::Tare_Complete()
{
    ////////////////////////////////////////////Safety Factor Calculations////////////////////////////////////////
    // Safety factor is expands minimum and maximum tare interval for reducing sample rate to fastening the calibration process.
    float tareMaxRatio;
    float tareMinRatio;
    // Checks if tareMax is zero or not to avoid infinity result problem.
    // Tare  max and min values multipling by safety factor so that sequential calibration can expand the values despite that is undesirable.
    if (tareMax != 0)
        tareMaxRatio = (abs(tareMax - tareMax_C) / tareMax) * 100;
    else
        tareMaxRatio = 4;

    if (tareMaxRatio > 3)
        tareMax = tareMax_C * (1 + safetyFactor);

    if (tareMin != 0)
        tareMinRatio = (abs(tareMin - tareMin_C) / tareMin) * 100;
    else
        tareMinRatio = 4;

    if (tareMinRatio > 3)
        tareMin = tareMin_C / (1 - safetyFactor);

    ////////////////////////////////////////////Safety Factor Calculations////////////////////////////////////////
    tareMaxMin_once = true;

    Serial.print("TareMin and _TareMin : ");
    Serial.print(tareMin);
    Serial.print("  ");
    Serial.println(tareMin_C);
    Serial.print("TareMax Ratio and TareMin Ratio : ");
    Serial.print(tareMaxRatio);
    Serial.print("  ");
    Serial.println(tareMinRatio);

    Serial.print("TareMax and TareMin : ");
    Serial.print(tareMax);
    Serial.print(" , ");
    Serial.println(tareMin);

    isTared = true;
    // EEPROM one adress just a byte
    // Taremin start adress 7 (4)
    // Taremax start adress 11 (4)

    EEpromWrite(7, sizeof(tareMin), tareMin);
    EEpromWrite(11, sizeof(tareMax), tareMax);

    EEPROM.write(15, isTared);
}
void Loadcell::EEpromWrite(int StartAdress, int Lenght, long val)
{
    Serial.println(val);
    byte buf[Lenght];
    for (int i = 0; i < Lenght; i++)
    {
        buf[i] = (byte)(val >> (i * 8));
        EEPROM.write(StartAdress + i, buf[i]);
    }
}

long Loadcell::EEpromRead(int StartAdress, int Lenght)
{
    long RetVal = 0;
    for (int i = 0; i < Lenght; i++)
    {
        long x = EEPROM.read(StartAdress + i);
        RetVal |= (x << (i * 8));
    }
   
    return RetVal;
}
void Loadcell::CalculateCalFactor()
{
    if (!calActive)
        return;
    if (!isSamplingDone())
        return;

    calfac = CalculateWeight() / KnownWeight;

    EEPROM.write(6, calfac);

    Serial.print("Calibration factor is : ");
    Serial.println(calfac);
    calActive = false;
    isCalFacCalculated = true;
}
bool Loadcell::isCalibrated()
{
    if (isTared && calfac > 0)
        return true;
    else
        return false;
}

void Loadcell::CalculateCalFactorActive()
{
    calActive = true;
    isCalFacCalculated = false;
    SamplingActive(true, true);
}
void Loadcell::TareActive()
{
    tareMin_C = 0;
    tareMax_C = 0;
    tareActive = true;
    isTared = false;
}
void Loadcell::SamplingActive(bool activity, bool calMode)
{
    isSampled = false;
    samplingActive = activity;
    CalMode = calMode;
}

void Loadcell::MainLoopOperations()
{
    Tare();
    Sampler();
    CalculateCalFactor();
}
