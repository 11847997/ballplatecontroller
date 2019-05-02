
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

//Number of States, Inputs, and Outputs
#define n 4   //number of states 
#define m 1   //number of inputs 
#define p 1   //number of outputs

//Discrete- Time System Vectors
float x_k[n][1];  //State vector  x(k)
float u_k[m][1];  //Input vector  u(k)
float y_k[p][1];  //Output vector y(k)

//Discrete-Time System Matrices
float A[n][n];
float B[n][m];
float C[p][n];

//Intermediate Multiplications
float Ax_k[n][1];     //Intermediate vector A·x(k)
float Bu_k[n][1];     //Intermediate vector B·u(k)

//SFC u(k)=-F*x(k)+r
float F[m][n];      //Designed offline
float r[m][1];      //Computed online
float M[m][p];      // r=M*y_ref
float Fx_k[m][1];   //Intermediate vector F·x(k)
float y_ref[p][1];  //Output Reference y*

// Observer
float L[n][1];      //Designed offline
float L_error[n][1];      //Designed offline


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
    y_ref[0][0]=0;
    
  //Initialize matrices  
    A[0][0]=0.9577;     A[0][1]=0.0000;    A[0][2]=0.0000;    A[0][3]=0.0000; 
    A[1][0]=0.0196;     A[1][1]=1.0000;     A[1][2]=0.0000;    A[1][3]=0.0000; 
    A[2][0]=0.0002;     A[2][1]=0.0200;     A[2][2]=1.0000;    A[2][3]=0.0000; 
    A[3][0]=0.0000;     A[3][1]=0.0002;     A[3][2]=0.0200;    A[3][3]=1.0000; 
  
    B[0][0]=0.0196;
    B[1][0]=0.0002;
    B[2][0]=0.0000;
    B[3][0]=0.0000;
  
    C[0][0]=0;  C[0][1]=0; C[0][2]=0; C[0][3]=-0.0151;
    
    F[0][0]=15.8584;  F[0][1]=120.9189;  F[0][2]=338.7070;  F[0][3]=321.885;
     
    M[0][0]=0;

    L[0][0] =-7.3416*powf(10.0, 4.0);
    L[0][0] =-2.4237*powf(10.0, 4.0);
    L[0][0] =-0.2267*powf(10.0, 4.0);
    L[0][0] =-0.0098*powf(10.0, 4.0);
    
    u_k[0][0]=0;

    for (int i = 0; i < n; ++i)
    {
        x_k[0][i]=0;
    }
    
    x_k[0][3]=0.0001;

    
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

  if (count==300)
    y_ref[0][0] -= 0.04;

  count++;
  if(count==600)
  {
    count=0;
    y_ref[0][0] += 0.04;
  }
  
/*
______________________
Board Inputs
______________________
*/
    //It is possible to adjust offset and gain of
    //the measurements. Also, disp = 1 will display
    //individual input in serial monitor
    
    //read_inputx(offset, gain, disp)
    //in1 = read_input1(0, 1, 0);  // 0 -> 12v
    //in2 = read_input2(0, 1, 0);       // 0 -> 12v
    //in3 = read_input3(0, 1, 0);     // -12v -> 12v
    //in4 = read_input4(0, 1, 0);   // -12v -> 12v

    //disp_inputs_all();


    
//Only display inputs for calibration. 
//Do not display them when running the controller

   

/*___________________________
State Feedback Controller
___________________________
*/

    //read_calibrated_1();
    //read_calibrated_2();
    //read_calibrated_3();
    //read_calibrated_4();
    //write_out_calibrated_1(outval);
    //write_out_calibrated_2(outval);
    //write_out_calibrated_3(outval);
    //write_out_calibrated_4(outval);
    // States Measurement
    x_k[3][0]=read_calibrated_1() - 0.813; //x1(k)

    
    //r=M·y_ref
    r[0][0] = F[0][3]*y_ref[0][0];
    //Matrix.Multiply((float*)M, (float*)y_ref, m, p, 1, (float*)r);

    //F·x(k);
    Matrix.Multiply((float*)F, (float*)x_k, m, n, 1, (float*)Fx_k);    
    
    //   u(k)=-F·x(k)+r 
    //or u(k)=r-F·x(k)

    Matrix.Subtract((float*)r, (float*)Fx_k, m,  1, (float*)u_k);
    

    //Finally, assign each controller output to one board output
    out1=-u_k[0][0];
    out2=0;//x_k[3][0]; // reference
    out3=0;
    out4=0;

    // reset all your x_k values
    float error = x_k[3][0] - y_ref[0][0];
    for (int i = 0; i < n; ++i)
    {
      x_k[i][0] = 0;
      L_error[i][0] = L[i][0]*error;
    }

    Matrix.Multiply((float*)A, (float*)x_k, n, n, 1, (float*)Ax_k);    
    Matrix.Multiply((float*)B, (float*)u_k, n, 1, 1, (float*)Bu_k);    
    Matrix.Multiply((float*)F, (float*)x_k, m, n, 1, (float*)Fx_k);    

    Matrix.Add((float*)Fx_k, (float*)r, m,  1, (float*)u_k);
    Matrix.Add((float*)Ax_k, (float*)x_k, n,  1, (float*)x_k);
    Matrix.Add((float*)Bu_k, (float*)x_k, n,  1, (float*)x_k);
    Matrix.Add((float*)L_error, (float*)x_k, n,  1, (float*)x_k);


    //x_hat_k1=A*x_hat_k+B*u_k+L*(y_k-y_hat_k);



    
    
/*
______________________
Board Outputs
______________________
*/
    //It is possible to adjust offset and gain of
    //each output. Also, disp = 1 will display
    //individual output in serial monitor
     
  //write_outx(value, offset, gain, disp) 
    //write_out1(out1,  0,  1, 0);   // Pin 5  -12V to 12V
    //write_out2(out2,  0,  1, 0);   // Pin 4  -12V to 12V
    //write_out3(out3,  0,  1, 0);   // Pin 7    0V to 12V
    //write_out4(out4,  0,  1, 0);   // Pin 6    0V to 12V

    write_out_calibrated_1(out1);
    write_out_calibrated_2(out2);
    write_out_calibrated_3(out3);
    write_out_calibrated_4(out4);
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

float calibrate(float input, float mx, float b)
{
  return mx*input + b;
}

    
float read_calibrated_1(void)
{
    // Calibrated reading
    return calibrate(read_input1(0, 1, 0), 0.9642, -0.0536); // correct
}

float read_calibrated_2(void)
{
    // Calibrated reading
    return calibrate(read_input2(0, 1, 0), 0.9612, 0.0045); // correct
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

float saturate(float saturation_limit, float value)
{
    float saturated_output = value;

    if (saturated_output > saturation_limit)
    {
      saturated_output = saturation_limit;
    }
    if (saturated_output < -saturation_limit) 
    {
      saturated_output = -saturation_limit;
    }

    return saturated_output;
}

void write_out_calibrated_1(float value)
{
    float saturation_limit = 7.00;
    float output = saturate(saturation_limit, calibrate(value, 0.9945, -0.0525));

    write_out1(output,  0,  1, 0);
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
