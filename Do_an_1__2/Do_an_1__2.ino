
#define setpoint_pin A0
#define sensor_pin A1
#define pwm_pin 3

double kp, ki, kd;
// double p, i, d;

const int min_control = 0;
const int max_control = 255;
const int T = 1;

double setpoint, sensor;
double pot_adc, sensed;
double xn1 = 0, yn1 = 0, xn2 = 0, yn2 = 0;

double error;
double last_error = 0;
double total_error = 0;
double delta_error;

unsigned long current_time;
unsigned long last_time = 0;

int pwm;

void setup() {
  
  kp = 0.8;
  ki = 0.004;
  kd = 0.07;  

  Serial.begin(9600);
  for(int j = 0; j<50; j++)
  {
    Serial.println(0);
    delay(20);
  }
}

void loop() {
  pot_adc = analogRead(setpoint_pin) / 4;
  setpoint = 0.969 * yn1 + 0.0155 * pot_adc + 0.0155 * xn1;
  xn1 = pot_adc;
  yn1 = setpoint;

  // setpoint = analogRead(setpoint_pin);

  current_time = millis();
  int delta_time = current_time - last_time;
  if(delta_time >= T)
  {
    sensed = analogRead(sensor_pin) / 4;
    sensor = 0.969 * yn2 + 0.0155 * sensed + 0.0155 * xn2;
    xn2 = sensed;
    yn2 = sensor;

    // sensor = analogRead(sensor_pin);

    error = sensor - setpoint;
    total_error += error; 

    if(total_error >= max_control) total_error = max_control;
    else if(total_error <= min_control) total_error = min_control;

    delta_error = error - last_error;

    pwm = kp * error + (ki * T) * total_error + (kd / T) * delta_error;

    if(pwm >= max_control) pwm = max_control;
    else if(pwm <= min_control) pwm = min_control;    

    analogWrite(pwm_pin, pwm);

    last_error = error;
    last_time = current_time;

    Serial.print(setpoint);
    Serial.print(' ');  
    Serial.print(sensor,3);
    Serial.print(',');  
    Serial.print(error,3);
    Serial.print(' ');  
    Serial.print(pwm,3);
    Serial.println();
  }
}
