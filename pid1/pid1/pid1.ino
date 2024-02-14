//********7seg*****************
//********7seg*****************
#include <HCMAX7219.h>
#include "SPI.h"
#define LOAD 10
HCMAX7219 HCMAX7219(LOAD);

//**********sensor**************
const int    SAMPLE_NUMBER      = 50;
const double BALANCE_RESISTOR   = 9710.0;
const double MAX_ADC            = 1023.0;
const double BETA               = 3974.0;
const double ROOM_TEMP          = 298.15;   // room temperature in Kelvin
const double RESISTOR_ROOM_TEMP = 100000.0;
double currentTemperature = 0;
int thermistorPin = A5;  // Where the ADC samples the resistor divider's output

#include <EEPROMex.h>
#include "Arduino.h"
//************PIN**********
int firing_pin = 3;
int zero_cross = 8;
const int ledred =  6;
const int ledgreen =  7;
int k1 = A4;
int k2 = A3;

int kelid = 5;

int temp_;

//***************PID*****************
//Variables
int last_CH1_state = 0;
bool zero_cross_detected = false;
int firing_delay = 7400;

//////////////////////////////////////////////////////
int maximum_firing_delay = 7400;
/*Later in the code you will se that the maximum delay after the zero detection
   is 7400. Why? Well, we know that the 220V AC voltage has a frequency of around 50-60HZ so
   the period is between 20ms and 16ms, depending on the country. We control the firing
   delay each half period so each 10ms or 8 ms. To amke sure we wont pass thsoe 10ms, I've made tests
   and the 7400us or 7.4ms was a good value. Measure your frequency and chande that value later */
//////////////////////////////////////////////////////

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
int temp_read_Delay = 250;
int real_temperature = 0;
int setpoint = 150;

//PID variables
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
long PID_value = 0;
//PID constants
long kp = 500;   int ki = 1.6;   int kd = 1.01;
long PID_p = 0;    int PID_i = 0;    int PID_d = 0;

int low_temp;
int high_temp;
int error_temp_high;
int error_temp_low;
int start_count_led;

int dama;


void setup() {
  Serial.begin(9600);
  //  dama = 150;
  //  EEPROM.writeFloat(0, dama);

  dama = EEPROM.readFloat(0);
  setpoint = dama;
  //*******************PID********************
  pinMode (firing_pin, OUTPUT);
  pinMode (zero_cross, INPUT);
  pinMode (zero_cross, INPUT_PULLUP);
  pinMode (k1, INPUT_PULLUP);
  pinMode (k2, INPUT_PULLUP);
  pinMode (kelid, INPUT_PULLUP);
  pinMode(ledred, OUTPUT);
  pinMode(ledgreen, OUTPUT);
  int i;
  digitalWrite(ledred, LOW);
  digitalWrite(ledgreen, HIGH);
  for (i = 0; i < 10; ++i) {
    digitalWrite(ledred, !digitalRead(ledred));
    digitalWrite(ledgreen, !digitalRead(ledgreen));
    delay(50);
  }

  digitalWrite(ledred, LOW);
  digitalWrite(ledgreen, HIGH);
  delay(100);
  PCICR |= (1 << PCIE0);    //enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  //Set pin D8 (zero cross input) trigger an interrupt on state change.
  Serial.println("start");

}

void loop() {
  static unsigned long timer = millis();
  static int deciSeconds = 0;
  static int deciSeconds1 = 0;

  if (millis() >= timer) {
    deciSeconds++; // 100 milliSeconds is equal to 1 deciSecond
    timer += 100;

    if (deciSeconds > 10) {
      deciSeconds = 0;
      //delay(5);
    }

  }

  if (digitalRead(kelid) == HIGH) pid();

  if (digitalRead(kelid) == LOW) {
    digitalWrite(ledred, LOW);
    digitalWrite(ledgreen, HIGH);
    HCMAX7219.Clear();
    HCMAX7219.print7Seg("stop", 24);
    HCMAX7219.Refresh();
  }

}



void pid() {
  currentMillis = millis();           //Save the value of time before the loop
  /*  We create this if so we will read the temperature and change values each "temp_read_Delay"
      value. Change that value above iv you want. The MAX6675 read is slow. Tha will affect the
      PID control. I've tried reading the temp each 100ms but it didn't work. With 500ms worked ok.*/
  if (currentMillis - previousMillis >= temp_read_Delay) {
    previousMillis += temp_read_Delay;              //Increase the previous time for next loop
    currentTemperature = readThermistor();
    real_temperature = currentTemperature;  //get the real temperature in Celsius degrees


    PID_error = setpoint - real_temperature;        //Calculate the pid ERROR

    if (PID_error > 10)                             //integral constant will only affect errors below 30ºC
    {
      PID_i = 0;
    }

    ////pid-p
    PID_p = kp * PID_error; //Calculate the P value
    // if (PID_p < 0)PID_p = 0;

    /////pid-i
    PID_i = PID_i + (ki * PID_error);  //Calculate the I value
    //if (PID_i > 7400)PID_i = 7400;
    if (PID_i < 0)PID_i = 0;

    ////pid-d
    timePrev = Time;                    // the previous time is stored before the actual time read
    Time = millis();                    // actual time read
    elapsedTime = (Time - timePrev) / 1000;
    PID_d = kd * ((PID_error - previous_error) / elapsedTime); //Calculate the D value
    PID_value = PID_p + PID_i + PID_d;                      //Calculate total PID value

    //We define firing delay range between 0 and 7400. Read above why 7400!!!!!!!
    if (PID_value < 0)
    {
      PID_value = 0;
    }
    if (PID_value > 7400)
    {
      PID_value = 7400;
    }

    low_temp = setpoint - 3;
    high_temp = setpoint + 3;
    error_temp_low = setpoint - 15;
    error_temp_high = setpoint + 15;

    if (real_temperature < low_temp || real_temperature > high_temp) {
      start_count_led = 0;
    }

    if (real_temperature == low_temp || real_temperature == high_temp || real_temperature == setpoint) {
      ++ start_count_led;
    }

    if (start_count_led > 100) {
      start_count_led = 0;
      digitalWrite(ledred, HIGH);
      digitalWrite(ledgreen, LOW);
    }

    if (error_temp_low >= real_temperature || error_temp_high <= real_temperature) {
      start_count_led = 0;
      digitalWrite(ledred, LOW);
      digitalWrite(ledgreen, HIGH);
    }

    if (digitalRead(k1) == LOW)  {
      ++setpoint;
      dama = setpoint;
      EEPROM.writeFloat(0, dama);
      delay(30);
    }
    if (digitalRead(k2) == LOW)  {
      --setpoint;
      dama = setpoint;
      EEPROM.writeFloat(0, dama);
      delay(30);
    }

    //if (second >= 1)
    show_temp();


    previous_error = PID_error; //Remember to store the previous error.
  }


  //If the zero cross interruption was detected we create the 100us firing pulse
  if (temp_ > -5) {
    if (zero_cross_detected)
    {
      delayMicroseconds(maximum_firing_delay - PID_value); //This delay controls the power
      digitalWrite(firing_pin, HIGH);
      delayMicroseconds(100);
      digitalWrite(firing_pin, LOW);
      zero_cross_detected = false;
    }
  }


}

ISR(PCINT0_vect) {
  ///////////////////////////////////////Input from optocoupler
  if (PINB & B00000001) {          //We make an AND with the state register, We verify if pin D8 is HIGH???
    if (last_CH1_state == 0) {     //If the last state was 0, then we have a state change...
      zero_cross_detected = true;  //We have detected a state change! We need both falling and rising edges
    }
  }
  else if (last_CH1_state == 1) {  //If pin 8 is LOW and the last state was HIGH then we have a state change
    zero_cross_detected = true;    //We haev detected a state change!  We need both falling and rising edges.
    last_CH1_state = 0;            //Store the current state into the last state for the next loop
  }

}

double readThermistor()
{
  double rThermistor = 0;            // Holds thermistor resistance value
  double tKelvin     = 0;            // Holds calculated temperature
  double tCelsius    = 0;            // Hold temperature in celsius
  double adcAverage  = 0;            // Holds the average voltage measurement
  int    adcSamples[SAMPLE_NUMBER];  // Array to hold each voltage measurement

  for (int i = 0; i < SAMPLE_NUMBER; i++)
  {
    adcSamples[i] = analogRead(thermistorPin);  // read from pin and store
    delay(2);        // wait 10 milliseconds
  }
  for (int i = 0; i < SAMPLE_NUMBER; i++)
  {
    adcAverage += adcSamples[i];      // add all samples up . . .
  }
  adcAverage /= SAMPLE_NUMBER;        // . . . average it w/ divide
  rThermistor = BALANCE_RESISTOR * ( (MAX_ADC / adcAverage) - 1);
  tKelvin = (BETA * ROOM_TEMP) /
            (BETA + (ROOM_TEMP * log(rThermistor / RESISTOR_ROOM_TEMP)));
  tCelsius = tKelvin - 273.15;  // convert kelvin to celsius

  return tCelsius;    // Return the temperature in Celsius

}



void show_temp() {
  int volt = map(PID_value, 0, 7400, 0, 220);
  HCMAX7219.Clear();
  //HCMAX7219.print7Seg(volt, 16);
  //HCMAX7219.print7Seg("uolt", 12);
  //HCMAX7219.print7Seg(PID_p, 12);

  temp_ = real_temperature;
  if (temp_ > -5) {
    HCMAX7219.print7Seg(setpoint, 20);
    HCMAX7219.print7Seg(temp_, 24);
    // String ss = "Temp:" + String(temp_) + " /Set point:" + setpoint+ " /voltage:" + volt;
    //Serial.println(ss);
  }
  if (temp_ < -5) HCMAX7219.print7Seg("ERROR1", 24);
  HCMAX7219.Refresh();
}

