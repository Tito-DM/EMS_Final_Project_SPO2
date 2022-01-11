#include <Arduino.h>
#include <Wire.h>

// a liquid crystal displays BPM
//LiquidCrystal_I2C lcd(0x3F, 16, 2);

//temp sensor variable
#define AnalogPin A10
float R1 = 5000.0;
float R6 = R1;
//float R4=1000.0;
//float R3=2200.0;
//float R5=5800.0;
float Vth = 1.57;
float Rth = 6504.0;
float Vcc = 5.0;
float R_NTC = 0.0;
float Vo = 0.0;
//float Ro=10000;
//float To=25+273.15;
float T = 0.0;
float A = 0.0015936558304898885;
float B = 0.00017023091250123744;
float C = 2.4634823231312137 * pow(10, -7);
float X = 0;
/*----------------end-----------*/

// spo2 sensor variables

#define maxperiod_siz 80 // max number of samples in a period
#define measures 10      // number of periods stored
#define samp_siz 4       // number of samples for average
#define rise_threshold 3 // number of rising measures to determine a peak
int T = 20;              // slot milliseconds to read a value from the sensor
int sensorPin = A11;
int REDLed = 3;
int IRLed = 4;

/*----------------end-----------*/

float T_SH(float R)
{

  float A = 0.0015936558304898885;
  float B = 0.00017023091250123744;
  float C = 2.4634823231312137 * pow(10, -7);

  float Y = A + B * log(R) + C * pow(log(R), 3);
  float X = (1 / Y - 273.15);
  return X;
}

//temp sensor task
void tempSensorTask(void *params)
{
  //loop infinito
  while (1)
  {
    Vo = analogRead(AnalogPin) * 5.0 / 1023.0;
    R_NTC = (float)R1 * (Vcc * (1 + R6 / Rth) / (Vo + R6 * Vth / Rth) - 1);

    Serial.println("-----------");
    Serial.print("Vo:");
    Serial.println(Vo);
    Serial.print("R_NTC:");
    Serial.println(R_NTC);

    //Steihart-Hart Model
    T = T_SH(R_NTC);
    Serial.print("T: ");
    Serial.println(T);
    delay(1000);

    //delay o task por 1s
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

//spo2 sensor task

void spo2SensorTask(void *parmas)
{
  bool finger_status = true;

  float readsIR[samp_siz], sumIR, lastIR, reader, start;
  float readsRED[samp_siz], sumRED, lastRED;

  int period, samples;
  period = 0;
  samples = 0;
  int samplesCounter = 0;
  float readsIRMM[maxperiod_siz], readsREDMM[maxperiod_siz];
  int ptrMM = 0;
  for (int i = 0; i < maxperiod_siz; i++)
  {
    readsIRMM[i] = 0;
    readsREDMM[i] = 0;
  }
  float IRmax = 0;
  float IRmin = 0;
  float REDmax = 0;
  float REDmin = 0;
  double R = 0;

  float measuresR[measures];
  int measuresPeriods[measures];
  int m = 0;
  for (int i = 0; i < measures; i++)
  {
    measuresPeriods[i] = 0;
    measuresR[i] = 0;
  }

  int ptr;

  float beforeIR;

  bool rising;
  int rise_count;
  int n;
  long int last_beat;
  for (int i = 0; i < samp_siz; i++)
  {
    readsIR[i] = 0;
    readsRED[i] = 0;
  }
  sumIR = 0;
  sumRED = 0;
  ptr = 0;

  while (1)
  {

    //
    // turn on IR LED
    digitalWrite(REDLed, LOW);
    digitalWrite(IRLed, HIGH);

    // calculate an average of the sensor
    // during a 20 ms (T) period (this will eliminate
    // the 50 Hz noise caused by electric light
    n = 0;
    start = millis();
    reader = 0.;
    do
    {
      reader += analogRead(sensorPin);
      n++;
    } while (millis() < start + T);
    reader /= n; // we got an average
    // Add the newest measurement to an array
    // and subtract the oldest measurement from the array
    // to maintain a sum of last measurements
    sumIR -= readsIR[ptr];
    sumIR += reader;
    readsIR[ptr] = reader;
    lastIR = sumIR / samp_siz;

    //
    // TURN ON RED LED and do the same

    digitalWrite(REDLed, HIGH);
    digitalWrite(IRLed, LOW);

    n = 0;
    start = millis();
    reader = 0.;
    do
    {
      reader += analogRead(sensorPin);
      n++;
    } while (millis() < start + T);
    reader /= n; // we got an average
    // Add the newest measurement to an array
    // and subtract the oldest measurement from the array
    // to maintain a sum of last measurements
    sumRED -= readsRED[ptr];
    sumRED += reader;
    readsRED[ptr] = reader;
    lastRED = sumRED / samp_siz;

    //
    // R CALCULATION
    // save all the samples of a period both for IR and for RED
    readsIRMM[ptrMM] = lastIR;
    readsREDMM[ptrMM] = lastRED;
    ptrMM++;
    ptrMM %= maxperiod_siz;
    samplesCounter++;
    //
    // if I've saved all the samples of a period, look to find
    // max and min values and calculate R parameter
    if (samplesCounter >= samples)
    {
      samplesCounter = 0;
      IRmax = 0;
      IRmin = 1023;
      REDmax = 0;
      REDmin = 1023;
      for (int i = 0; i < maxperiod_siz; i++)
      {
        if (readsIRMM[i] > IRmax)
          IRmax = readsIRMM[i];
        if (readsIRMM[i] > 0 && readsIRMM[i] < IRmin)
          IRmin = readsIRMM[i];
        readsIRMM[i] = 0;
        if (readsREDMM[i] > REDmax)
          REDmax = readsREDMM[i];
        if (readsREDMM[i] > 0 && readsREDMM[i] < REDmin)
          REDmin = readsREDMM[i];
        readsREDMM[i] = 0;
      }
      R = ((REDmax - REDmin) / REDmin) / ((IRmax - IRmin) / IRmin);
    }

    // check that the finger is placed inside
    // the sensor. If the finger is missing
    // RED curve is under the IR.
    //
    if (lastRED < lastIR)
    {
      if (finger_status == true)
      {
        finger_status = false;
        //lcd.clear();
        //lcd.setCursor(0,0);
        //lcd.print("No finger?");
      }
    }
    else
    {
      if (finger_status == false)
      {
        //lcd.clear();
        finger_status = true;

        //lcd.setCursor(10,0);
        //lcd.print("c=");
        /*
            lcd.setCursor(0,0);
            lcd.print("bpm");
            lcd.setCursor(0,1);
            lcd.print("SpO"); lcd.write(1);  //2            
            lcd.setCursor(10,1);
            lcd.print("R=");
*/
      }
    }

    float avR = 0;
    int avBPM = 0;

    if (finger_status == true)
    {

      // lastIR holds the average of the values in the array
      // check for a rising curve (= a heart beat)
      if (lastIR > beforeIR)
      {

        rise_count++; // count the number of samples that are rising
        if (!rising && rise_count > rise_threshold)
        {
          //lcd.setCursor(3,0);
          //lcd.write( 0 );       // <3
          // Ok, we have detected a rising curve, which implies a heartbeat.
          // Record the time since last beat, keep track of the 10 previous
          // peaks to get an average value.
          // The rising flag prevents us from detecting the same rise
          // more than once.
          rising = true;

          measuresR[m] = R;
          measuresPeriods[m] = millis() - last_beat;
          last_beat = millis();
          int period = 0;
          for (int i = 0; i < measures; i++)
            period += measuresPeriods[i];

          // calculate average period and number of samples
          // to store to find min and max values
          period = period / measures;
          samples = period / (2 * T);

          int avPeriod = 0;

          int c = 0;
          // c stores the number of good measures (not floating more than 10%),
          // in the last 10 peaks
          for (int i = 1; i < measures; i++)
          {
            if ((measuresPeriods[i] < measuresPeriods[i - 1] * 1.1) &&
                (measuresPeriods[i] > measuresPeriods[i - 1] / 1.1))
            {

              c++;
              avPeriod += measuresPeriods[i];
              avR += measuresR[i];
            }
          }

          m++;
          m %= measures;

          //lcd.setCursor(12,0);
          //lcd.print(String(c)+"  ");

          // bpm and R shown are calculated as the
          // average of at least 5 good peaks
          avBPM = 60000 / (avPeriod / c);
          avR = avR / c;

          // if there are at last 5 measures
          //lcd.setCursor(12,1);
          // if(c==0) lcd.print("    "); else lcd.print(String(avR) + " ");

          // if there are at least 5 good measures...
          if (c > 4)
          {

            //
            // SATURTION IS A FUNCTION OF R (calibration)
            // Y = k*x + m
            // k and m are calculated with another oximeter
            int SpO2 = -19 * R + 112;

            //lcd.setCursor(4,0);
            //if(avBPM > 40 && avBPM <220) lcd.print(String(avBPM)+" "); //else lcd.print("---");

            // lcd.setCursor(4,1);
            //if(SpO2 > 70 && SpO2 <150) lcd.print( " " + String(SpO2) +"% "); //else lcd.print("--% ");
          }
          else
          {
            if (c < 3)
            {
              // if less then 2 measures add ?
              //lcd.setCursor(3,0); lcd.write( 2 ); //bpm ?
              //lcd.setCursor(4,1); lcd.write( 2 ); //SpO2 ?
            }
          }
        }
      }
      else
      {
        // Ok, the curve is falling
        rising = false;
        rise_count = 0;
        //lcd.setCursor(3,0);lcd.print(" ");
      }

      // to compare it with the new value and find peaks
      beforeIR = lastIR;

    } // finger is inside

    // PLOT everything
    Serial.print(lastIR);
    Serial.print(",");
    Serial.print(lastRED);

    Serial.print(",");
    Serial.print(R);
    Serial.print(",");
    Serial.print(IRmax);
    Serial.print(",");
    Serial.print(IRmin);
    Serial.print(",");
    Serial.print(REDmax);
    Serial.print(",");
    Serial.print(REDmin);
    Serial.print(",");
    Serial.print(avR);
    Serial.print(",");
    Serial.print(avBPM);
    Serial.println();

    // handle the arrays
    ptr++;
    ptr %= samp_siz;

  } // loop while 1
}

void setup()
{
  Serial.begin(9600);
  Serial.flush();
  pinMode(sensorPin, INPUT);
  pinMode(REDLed, OUTPUT);
  pinMode(IRLed, OUTPUT);

  // initialize the LCD
  //lcd.init();
  // lcd.backlight();
  // turn off leds
  digitalWrite(REDLed, LOW);
  digitalWrite(IRLed, LOW);

  //create spo2 task
  xTaskCreate(
      spo2SensorTask, //nome da função
      "spo2",         //nome da tarefa
      10000,          //stack size
      NULL,           // parametros do tarefa
      1,              //prioridade da tarefa
      NULL);

  //create temp task
  xTaskCreate(
      tempSensorTask, //nome da função
      "temp",         //nome da tarefa
      1000,           //stack size
      NULL,           // parametros do tarefa
      1,              //prioridade da tarefa
      NULL);
}

void loop()
{
  //no code here
}