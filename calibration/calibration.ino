
/*
 Example of a State Feedback Controler (SFC)
 University of Technology Sydney (UTS)
 Board: MKR1000
 Based: bAC18
 Code:  v3

 Created by Ricardo P. Aguiilera, 
            Manh (Danny) Duong Phung,

 Date: 14/04/2019
 

 Hardward tools: Extended Arduino Board for MKR1000
 Software tools: It requires the following libraries:
    Timer5
    MatrixMath
    PWM_MKR1000_AdvCtrl_UTS
 */


/******************************
 ******************************
 Libraries
 */ 
#include <Arduino.h>
#include <Timer5.h>
#include <MatrixMath.h>
#include <PWM_MKR1000_AdvCtrl_UTS.h>
#include <math.h>

/******************************
 ******************************
 Definitions (constant parameters) and Variales
 */ 


//Discrete-Time LTI System To Be Controlled                              \
  x(k+1)=A·x(k)+B·u(k)                    \
    y(k)=C·x(k)                             \
                                            \
  where, x \in R^n, u \in R^m, y \in R^p    \

#define fs 100  // Sampling frequency [Hz]
                // Sampling Time, Ts=1/fs [s]


//Inputs
float in1 = 0;
float in2 = 0;
float in3 = 0;
float in4 = 0;

//Outputs
float out1 = 0;
float out2 = 0;
float out3 = 0;
float out4 = 0;

int count = 0;

/******************************
 ******************************
 Initializations
 */
void setup()
{  
  //Initialize Serial Buse
    Serial.begin(9600); 

  //Sampling Time Ts=1/fs [s]
    float Ts=1/float(fs); //in seconds      
  
  //Reference

  //Initialize I/O pins to measure execution time
    pinMode(LED_BUILTIN,OUTPUT);
    pinMode(A3,OUTPUT);
    digitalWrite(A3,LOW); 

  //ADC Resolution                                                      \
    The Due, Zero and MKR Family boards have 12-bit ADC capabilities    \
    that can be accessed by changing the resolution to 12.              \
    12 bits will return values from analogRead() between 0 and 4095.    \
    11 bits will return values from analogRead() between 0 and 2047.    \
    10 bits will return values from analogRead() between 0 and 1023.    \
    Default resolution if not used is 10bits.

    int res=12;
    analogReadResolution(res); //If commented, default resolution is 10bits

   
  // Configure PWM for outputs
    init_PWM_MKR1000_UTS();
  
  // define timer5 interruption freq (sampling time)
    MyTimer5.begin(fs);   //Argument is freq in Hz

  // define the interrupt callback function
    MyTimer5.attachInterrupt(Controller);
  
  // start the timer
    MyTimer5.start();   //Always start timer at the end
    
}





void Controller(void) {
/******************************
 ******************************
 Timer Interruption
 The code inside this section will be run at every Ts
 */

  //Start measuring execution time
  digitalWrite(A3,HIGH);  

 
  
/*
______________________
Board Inputs
______________________
*/
    //It is possible to adjust offset and gain of
    //the measurements. Also, disp = 1 will display
    //individual input in serial monitor
   
    //read_inputx(offset, gain, disp)
    float alpha = 0.7;
    in1 = exp_filter(alpha, in1, read_input1(0, 1, 0));  // 0 -> 12v
    in2 = exp_filter(alpha, in2, read_input2(0, 1, 0));  // 0 -> 12v
    in3 = exp_filter(alpha, in3, read_input3(0, 1, 0));  // -12v -> 12v
    in4 = exp_filter(alpha, in4, read_input4(0, 1, 0));  // -12v -> 12v

    //disp_inputs_all();
    disp_outputs_all();

    float outval = 10.0;
    // Calibrated Input Values
    //out1=calibrate(in1, 0.9642, -0.0536);
    //out2=calibrate(in2, 0.9612, 0.0045);
    //out3=calibrate(in3, 0.9719, -0.9162);  // correct
    //out4=calibrate(in4, 0.9703, -1.014);   // correct

    // Calibrated Output Values
    //out1=calibrate(outval, 0.9945, -0.0525);
    //out2=calibrate(outval, 0.9975, -0.0288);
    //out3=calibrate(outval, 0.9886, 0.0603);  // correct
    //out4=calibrate(outval, 0.9891, 0.0954);   // correct



    read_calibrated_1();
    read_calibrated_2();
    read_calibrated_3();
    read_calibrated_4();
    write_out_calibrated_1(outval);
    write_out_calibrated_2(outval);
    write_out_calibrated_3(outval);
    write_out_calibrated_4(outval);



    
    
/*
______________________
Board Outputs
______________________
*/
    //It is possible to adjust offset and gain of
    //each output. Also, disp = 1 will display
    //individual output in serial monitor
     
  //write_outx(value, offset, gain, disp) 
    write_out1(out1,  0,  1, 0);   // Pin 5  -12V to 12V
    write_out2(out2,  0,  1, 0);   // Pin 4  -12V to 12V
    write_out3(out3,  0,  1, 0);   // Pin 7    0V to 12V
    write_out4(out4,  0,  1, 0);   // Pin 6    0V to 12V

    //disp_outputs_all();
//Only display outputs for calibration. 
//Do not display them when running the controller   

  //Stop measuring calculation time
  digitalWrite(A3,LOW);   
}






//Main loop does nothing
void loop() { 

  //Left intentionally empty

  }






/*
______________________
Functions
______________________
*/

/*
 Display Inputs
 */
 void disp_inputs_all()
{
      Serial.print("In1: ");
      Serial.print(in1);
      Serial.print(" [V]  ");
      Serial.print("In2: ");
      Serial.print(in2);
      Serial.print(" [V]  ");
      Serial.print("In3: ");
      Serial.print(in3);
      Serial.print(" [V]  ");
      Serial.print("In4: ");
      Serial.print(in4);
      Serial.println(" [V]");
}

/*
 Display Outputs
 */
 void disp_outputs_all()
{
      Serial.print("Out1: ");
      Serial.print(out1);
      Serial.print(" [V]  ");
      Serial.print("Out2: ");
      Serial.print(out2);
      Serial.print(" [V]  ");
      Serial.print("Out3: ");
      Serial.print(out3);
      Serial.print(" [V]  ");
      Serial.print("Out4: ");
      Serial.print(out4);
      Serial.println(" [V]");
}

/*
 Read Inputs
 */
float read_input1(float offset, float gain, int disp) {
  float in_float = 0;
  int in = analogRead(A1); // Read input 1 (0 –> 12V)
  in_float = (float)(in)*0.00287-0.11;
  in_float = in_float*gain + offset;
  //in_float = (float)(in);

  if (disp==1){
      Serial.print("In1: ");
      Serial.print(in1);
      Serial.println(" [V]");
  }
  
  return in_float; 
}

float read_input2(float offset, float gain, int disp) {
  float in_float = 0;
  int in = analogRead(A2); // Read input 2 (0 –> 12V)
  in_float = (float)(in)*0.00287-0.11;
  in_float = in_float*gain + offset;
  //in_float = (float)(in);

  if (disp==1){
      Serial.print("In2: ");
      Serial.print(in2);
      Serial.println(" [V]");
  }

  return in_float; 
}

float read_input3(float offset, float gain, int disp) {
  float in_float = 0;
  int in = analogRead(A5); // Read input 3 (0 -> +/-12)
  
  in_float = (float)(in)-2044;
  in_float = in_float*0.00617;
  in_float = (in_float+ offset)*gain ;

    if (disp==1){
      Serial.print("In3: ");
      Serial.print(in3);
      Serial.println(" [V]");
  }

  return in_float; 
}

float read_input4(float offset, float gain, int disp) {
  float in_float = 0;
  int in = analogRead(A6); // Read input 4 (0 -> +/-12)
  in_float = (float)(in)-2044;
  in_float = in_float*0.00617;
  in_float = (in_float+ offset)*gain ;

    if (disp==1){
      Serial.print("In4: ");
      Serial.print(in4);
      Serial.println(" [V]");
  }
  
 return in_float;
}



/*
 Write Outputs
 */

void write_out1(float out, float offset, float gain, int disp)
{
  //Pin 5
  float d=(out-0.3533+offset)*0.0394*gain+0.5;
    if (d<0)
      d=0;
    REG_TCC0_CCB1= (int)((REG_TCC0_PER+1)*d);   
    if (disp==1){
      Serial.print("Out1 : ");
      Serial.print(out1);
      Serial.println(" [V]");
  }
 }

void write_out2(float out, float offset, float gain, int disp)
{
   //Pin 4
    float d=(out-0.3933+offset)*0.0394*gain+0.5;
    if (d<0)
      d=0;
    REG_TCC0_CCB0= (int)((REG_TCC0_PER+1)*d); 
    if (disp==1){
      Serial.print("Out2 : ");
      Serial.print(out2);
      Serial.println(" [V]");
  }

}

void write_out3(float out, float offset, float gain, int disp)
{
   //Pin 7
    float d = (out+offset)*0.076*gain;
    if (d<0)
      d=0;
    REG_TCC0_CCB3 = (int)((REG_TCC0_PER+1)*d); 
    if (disp==1){
      Serial.print("Out3 : ");
      Serial.print(out3);
      Serial.println(" [V]");
  }

}

void write_out4(float out, float offset, float gain, int disp)
{
   //Pin 6
    float d = (out+offset)*0.076*gain;
    if (d<0)
      d=0;
    REG_TCC0_CCB2 = (int)((REG_TCC0_PER+1)*d); 
    if (disp==1){
      Serial.print("Out4 : ");
      Serial.print(out4);
      Serial.println(" [V]");
  }
 
}

float exp_filter(float alpha, float old, float new_value)
{
  return alpha*old + (1.0-alpha)*new_value;
}

/*PWM Explanation
The sawtooth carrier incrases from 0 to Tmax

Tmax = REG_TCC0_PER + 1

Tmax    _____________________
             /|     /|     /|
            / |    / |----/-|
CMP_i   ---/--|   /  |   /  |
          /   |  /   |  /   |
         /    |-/----| /    |
    0   /     |/     |/     └───────> t
       |      |      |      |
       |      |      |      |
PWM i Output  |      |      |
       ┌──┐   ┌┐     ┌────┐ 
       │  │   ││     │    │ 
       │  │   ││     │    │ 
       ┘  └───┘└─────┘    └───────> t
*/

float calibrate(float input, float m, float b)
{
  return m*input + b;
}

    
float read_calibrated_1(void)
{
    // Calibrated reading
    return calibrate(read_input1(0, 1, 0), 0.9642, -0.0536);
}

float read_calibrated_2(void)
{
    // Calibrated reading
    return calibrate(read_input2(0, 1, 0), 0.9612, 0.0045);
}

float read_calibrated_3(void)
{
    // Calibrated reading
    return calibrate(read_input3(0, 1, 0), 0.9719, -0.9162);  // correct
} 

  
float read_calibrated_4(void)
{
    // Calibrated reading
    return calibrate(read_input4(0, 1, 0), 0.9703, -1.014);   // correct  
}

void write_out_calibrated_1(float value)
{
    write_out1(calibrate(value, 0.9945, -0.0525),  0,  1, 0);
}

void write_out_calibrated_2(float value)
{   
    write_out2(calibrate(value, 0.9975, -0.0288),  0,  1, 0);
}

void write_out_calibrated_3(float value)
{    
    write_out3(calibrate(value, 0.9886, 0.0603),  0,  1, 0);
}

void write_out_calibrated_4(float value)
{
    
    write_out4(calibrate(value, 0.9891, 0.0954),  0,  1, 0);
}
