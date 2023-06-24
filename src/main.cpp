#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>

// Initialise LCD
U8X8_SSD1306_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE);

const int debugState = 1;
const int noResidual = 1; // WARNING THIS NEEDS TO BE UNSET AFTER TESTING ONCE WE HAVE EEPROM
const int testPressure = 1.013; //BAR
const float eulers =  2.7182818284590452353602874;
float currFactor = 70; // TESTING VALUE;
// Define variables
// O2 Cells
const int  pinCell1 = A0;
const int pinCell2 = A1;
const int pinCell3 = A2;
// Pressure Sensors
const int pinPressExt = A3;
const int pinPressInt = A4;
// Thermistors
//int pinTherm1 = A11;
//int pinTherm2 = A12;
//int pinTherm3 = A13;
//int pinTherm4 = A14;
//int pinTherm5 = A15;

// Gas in use
int dil[2];
// Pressure readings
float pExt;
float pInt;
// Temperature readings
float t1;
float t2;
float t3;
float t4;
float t5;
// Cell readings
float cell1Volt;
float cell2Volt;
float cell3Volt;

// Times
int lastTime;
int readTime;

//Depths
float lastPress;
float absPress;
// Define ZH-L16C values - METRIC
const float tissueFactors[16][6] =
{
//{hHe,    hN2,    aHe,    bHe,    aN2,    bN2}
{1.88,    5.0,    16.189, 0.4770, 11.696, 0.5578}, //C1
{3.02,    8.0,    13.83,  0.5747, 10.0,   0.6514}, //C2
{4.72,    12.5,   11.919, 0.6527, 8.618,  0.7222}, //C3
{6.99,    18.5,   10.458, 0.7223, 7.562,  0.7825}, //C4
{10.21,   27.0,   9.220,  0.7582, 6.200,  0.8126}, //C5
{14.48,   38.3,   8.205,  0.7957, 5.043,  0.8434}, //C6
{20.53,   54.3,   7.305,  0.8279, 4.410,  0.8693}, //C7
{29.11,   77.0,   6.502,  0.8553, 4.000,  0.8910}, //C8
{41.20,   109.0,  5.950,  0.8757, 3.750,  0.9092}, //C9
{55.19,   146.0,  5.545,  0.8903, 3.500,  0.9222}, //C10
{70.69,   187.0,  5.333,  0.8997, 3.295,  0.9319}, //C11
{90.34,   239.0,  5.189,  0.9073, 3.065,  0.9403}, //C12
{115.29,  305.0,  5.181,  0.9122, 2.835,  0.9477}, //C13
{147.42,  390.0,  5.176,  0.9171, 2.610,  0.9544}, //C14
{188.24,  498.0,  5.172,  0.9217, 2.480,  0.9602}, //C15
{240.03,  635.0,  5.119,  0.9267, 2.327,  0.9653}  //C16
};

// Better to define this once instead of each time the loop is called
float pressVapor = 0.0627;

float surfLoad = 0.7902 * (testPressure - pressVapor);
float tissueLoadN2[16] = {surfLoad,surfLoad,surfLoad,surfLoad,surfLoad,surfLoad,surfLoad,surfLoad,surfLoad,surfLoad,surfLoad,surfLoad,surfLoad,surfLoad,surfLoad,surfLoad};
float tissueLoadHe[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


// Provide absolute pressure in BAR
// THIS IS THE CORE PART OF UPDATING HOW SATURATED A TISSUE IS; BE VERY CAREFUL MODIFYING
// Inputs
// tissueNumber: Index of tissue to check, valid range 0-16
// absPressue: What the absolute pressure is in BAR [Q: will this be loop pressure or external pressure?]
// cTissue: The P_i value, what the current partial pressure in the tissue is
// cGasO2: Percentage O2 in the loop
// cGasHe: Percentage Helium in the loop
// coeffToUse: What coefficent to use (use value from table, means we only write one loop for both He and N2)
// rocPress : Rate of change in pressure, as BAR per min
// exposureTime : Exposure time in mins (will need to time the main loop for this [Have ordered RTC's])
double update_tissue (float absPress2,double cTissue,float coeffToUse,float cGasO2, float cGasHe, float rocPress, float exposureTime) {
    double newTissueLoad; // What we plan to output
    double pressAlv; // What the inspired pressure in the alvioli are
    float fracGas = 1 - (cGasO2/ 100); // Fraction of inert gas (i.e nitrogen)
     // BAR, this is a constant, water vapor pressure in lungs
    float rocInert = cTissue * rocPress;
    pressAlv = fracGas * (absPress2 - pressVapor);
    double k = log(2)/coeffToUse;
    newTissueLoad = pressAlv + (rocInert * (exposureTime - 1/k)) - ((pressAlv - cTissue - rocInert/k)*pow(eulers,(-k*exposureTime)));
    return newTissueLoad;
}

float findCompartmentLimit (int tissueNumber, float tissueN2, float tissueHe, float gf) {
    float compLimit;
    float tissueHalfLifeHe = tissueFactors[tissueNumber][0];
    float tissueHalfLifeN2 = tissueFactors[tissueNumber][1]; 
    float HeA = tissueFactors[tissueNumber][2];
    float HeB = tissueFactors[tissueNumber][3];
    float N2A = tissueFactors[tissueNumber][4];
    float N2B = tissueFactors[tissueNumber][5];
    float pInert = tissueN2 + tissueHe;
    float aCombined = (HeA * tissueHe + N2A * tissueN2);
    float bCombined = (HeB * tissueHe + N2B * tissueN2);
    compLimit = (pInert - aCombined*gf)/(gf/bCombined + 1 - gf);
    return compLimit;

}

void setup() {
    Serial.begin(9600);
    display.begin();
    display.setPowerSave(0);
    display.setFont(u8x8_font_pxplusibmcgathin_f);
    display.clearDisplay();
    display.drawString(2,1,"CCaRduino");
    if(debugState == 1){
     display.drawString(2,3,"DEBUG");
    }
    else{
      display.drawString(2,3,"RELEASE");
    }
    int lastTime = millis();
    float currentDepth = analogRead(pinPressExt);
    float lastPress = currentDepth/1024 * 5; // Pressure in BAR - TESTING VALS
    delay(200);
};

void loop () {
    float currentDepth = analogRead(pinPressExt);
    float absPress = currentDepth/1024 * 5; // Pressure in BAR - TESTING VALS
    float trueDepth = currentDepth/1024 * 50; // FOR TESTING GIVE DUMMY DEPTH VALUES

    cell1Volt = analogRead(pinCell1)/1024.0;
    cell2Volt = analogRead(pinCell2)/1024.0;
    cell3Volt = analogRead(pinCell3)/1024.0;

    float cellAvg = (cell1Volt+cell2Volt+cell3Volt)/3; // FOR TESTING we assume 1 == 100% O2;


    Serial.print("Cell Avg:");
    Serial.print(cellAvg,2);
    Serial.print(",");
    Serial.print("Depth:");
    Serial.println(trueDepth,2);
    delay(200);

    // Pre test
    // Calibrate Dil
    // Calibrate O2
    // Cell Coefficents
    // Loop integrity

    // Calculate tissue cells;
    int readTime = millis();
    int timeDelta = readTime-lastTime;

    double pressChange = 60000*((absPress - lastPress)/(readTime-lastTime));
    for(int i = 0; i = 15; i++){
        float newTissueLoadN2[i] = {update_tissue(trueDepth,i,tissueFactors[i][1],cellAvg,0,pressChange,(readTime-lastPress)/60000)};
        float newTissueLoadHe[i] = {update_tissue(trueDepth,i,tissueFactors[i][0],cellAvg,0,pressChange,(readTime-lastPress)/60000)};
        float newTissueLimit[i] = {findCompartmentLimit(i,newTissueLoadN2[i],newTissueLoadN2[i],currFactor)};
    }
}