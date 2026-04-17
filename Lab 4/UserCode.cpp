#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"

#include <stdio.h> //for printf

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;


//We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

//Some constants that we may use:
const float mass = 32e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]
float const l = 33e-3f;
float const kappa = 0.01f;
Vec3f desTorque = Vec3f(0,0,0);


const float dt = 1.0f / 500.0f; //[s] period between successive calls to MainLoop

Vec3f estGyroBias = Vec3f(0,0,0);
Vec3f rateGyro_corr = Vec3f(0,0,0);
float estRoll = 0;
float estPitch = 0;
float estYaw = 0;


MainLoopOutput MainLoop(MainLoopInput const &in) {
  //Your code goes here!
  // The function input (named "in") is a struct of type
  // "MainLoopInput". You can understand what values it
  // contains by going to its definition (click on "MainLoopInput",
  // and then hit <F3> -- this should take you to the definition).
  // For example, "in.joystickInput.buttonBlue" is true if the
  // joystick's blue button is pushed, false otherwise.



  //Define the output numbers (in the struct outVals):
  MainLoopOutput outVals;
//  motorCommand1 -> located at body +x +y
//  motorCommand2 -> located at body +x -y
//  motorCommand3 -> located at body -x -y
//  motorCommand4 -> located at body -x +y

  // Gyroscope Correction Stuff
  if (in.currentTime < 1.0f) {
    estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 500.0f);
  }
  Vec3f rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;

  //Executing the Integrator
  float g = float(9.81);
  float rho = float(0.01);
  float measRoll = 0;
  float measPitch = 0;
  measRoll = in.imuMeasurement.accelerometer.y / g;
  measPitch = -in.imuMeasurement.accelerometer.x / g;

  float dt = float(1)/float(500);
  estRoll = (1- rho) * (estRoll + dt * rateGyro_corr.x) + rho * measRoll;
  estPitch = (1 - rho) * (estPitch + dt* rateGyro_corr.y) + rho * measPitch;
  estYaw = estYaw + dt * rateGyro_corr.z;

  // Outputing values for log
  outVals.telemetryOutputs_plusMinus100[0] = estRoll;
  outVals.telemetryOutputs_plusMinus100[1] = estPitch;
  outVals.telemetryOutputs_plusMinus100[2] = estYaw;

  //Set Constants
  float const timeConstant_rollRate = 0.04f;
  float const timeConstant_pitchRate = timeConstant_rollRate;
  float const timeConstant_yawRate = 0.5f;

  float const timeConstant_rollAngle = 0.4f ;
  float const timeConstant_pitchAngle = timeConstant_rollAngle;
  float const timeConstant_yawAngle = 1.0f;

  float desNormalizedAcceleration = 8.0f; //Normalized Total Thrust in [m/s^2]

  Vec3f cmdAngAcc= Vec3f(0,0,0);
  Vec3f desAngVel = Vec3f(0,0,0);

  Vec3f cmdAngVel= Vec3f(0,0,0);
  Vec3f desAng = Vec3f(0,0,0);

  Vec3f desAngularAcceleration = Vec3f(0,0,0); // Desired Angular Acceleration [rad/s^2]

  if (lastMainLoopInputs.joystickInput.buttonBlue){

    desAng = Vec3f(0, 0.5236, 0);

    }
  else{

    desAng = Vec3f(0,0,0);

    }
  
  // Outer loop cascaded controller
  cmdAngVel.x = (-1/timeConstant_rollAngle) * (estRoll - desAng.x);
  cmdAngVel.y = (-1/timeConstant_pitchAngle) * (estPitch - desAng.y);
  cmdAngVel.z = (-1/timeConstant_yawAngle) * (estYaw - desAng.z);
  desAngVel = cmdAngVel;
  // Inner loop cascaded controller
  cmdAngAcc.x = (-1/timeConstant_rollRate) * (rateGyro_corr.x - desAngVel.x);
  cmdAngAcc.y = (-1/timeConstant_pitchRate) * (rateGyro_corr.y - desAngVel.y);
  cmdAngAcc.z = (-1/timeConstant_yawRate) * (rateGyro_corr.z - desAngVel.z);

  // Outputing values for log
  outVals.telemetryOutputs_plusMinus100[3] = cmdAngAcc.x;
  outVals.telemetryOutputs_plusMinus100[4] = cmdAngAcc.y;
  outVals.telemetryOutputs_plusMinus100[5] = cmdAngAcc.z;

  outVals.telemetryOutputs_plusMinus100[6] = cmdAngVel.x;
  outVals.telemetryOutputs_plusMinus100[7] = cmdAngVel.y;
  outVals.telemetryOutputs_plusMinus100[8] = cmdAngVel.z;
  outVals.telemetryOutputs_plusMinus100[9] = desAng[1];

  desAngularAcceleration = cmdAngAcc;

  float desTotalForce = desNormalizedAcceleration * mass; // Desired Total Force [N]
  desTorque.x = desAngularAcceleration[0] * inertia_xx;
  desTorque.y = desAngularAcceleration[1] * inertia_yy;
  desTorque.z = desAngularAcceleration[2] * inertia_zz;

  // Mixer Matrix Equations
  float c1 = 0.25f * (desTotalForce + desTorque.x / l - desTorque.y / l + desTorque.z / kappa);
  float c2 = 0.25f * (desTotalForce - desTorque.x / l - desTorque.y / l - desTorque.z / kappa);
  float c3 = 0.25f * (desTotalForce - desTorque.x / l + desTorque.y / l + desTorque.z / kappa);
  float c4 = 0.25f * (desTotalForce + desTorque.x / l + desTorque.y / l - desTorque.z / kappa);
  
  // Calculating Motor Commands
  float s1 = speedFromForce(c1);
  float s2 = speedFromForce(c2);
  float s3 = speedFromForce(c3);
  float s4 = speedFromForce(c4);
  outVals.motorCommand1 = pwmCommandFromSpeed(s1);
  outVals.motorCommand2 = pwmCommandFromSpeed(s2);
  outVals.motorCommand3 = pwmCommandFromSpeed(s3);
  outVals.motorCommand4 = pwmCommandFromSpeed(s4);

  //copy the inputs and outputs:
  lastMainLoopInputs = in;
  lastMainLoopOutputs = outVals;
  return outVals;
}

void PrintStatus() {
  //For a quick reference on the printf function, see: http://www.cplusplus.com/reference/cstdio/printf/
  // Note that \n is a "new line" character.
  // Also, note that to print a `float` variable, you have to explicitly cast it to
  //  `double` in the printf function, and explicitly specify precision using something
  //  like %6.3f (six significant digits, three after the period). Example:
  //   printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //Accelerometer measurement
  printf("Acc: ");
  printf("x=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.x));
  printf("y=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.y));
  printf("z=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.z));
  printf("\n");  //new line

  //Gyroscope measurements
  printf("Gyro: ");
  printf("x=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x));
  printf("y=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.y));
  printf("z=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.z));
  printf("\n");  //new line

  //Gyroscope Bias Calculations and Measurements
  printf("Estimated Gyro Bias = (%6.3f, %6.3f, %6.3f)\n",
         double(estGyroBias[0]),
         double(estGyroBias[1]),
         double(estGyroBias[2])
           );

  printf("Corrected Gyro Values = (%6.3f, %6.3f, %6.3f)\n",
         double(rateGyro_corr[0]),
         double(rateGyro_corr[1]),
         double(rateGyro_corr[2])
           );

  printf("Estimated Orientation Values = (%6.3f, %6.3f, %6.3f)\n",
           double(estRoll),
           double(estPitch),
           double(estYaw)
             );


  printf("Example variable values:\n");
  printf("  exampleVariable_int = %d\n", exampleVariable_int);
  //Note that it is somewhat annoying to print float variables.
  //  We need to cast the variable as double, and we need to specify
  //  the number of digits we want (if you used simply "%f", it would
  //  truncate to an integer.
  //  Here, we print 6 digits, with three digits after the period.
  printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //We print the Vec3f by printing it's three components independently:
  printf("  exampleVariable_Vec3f = (%6.3f, %6.3f, %6.3f)\n",
         double(exampleVariable_Vec3f.x), double(exampleVariable_Vec3f.y),
         double(exampleVariable_Vec3f.z));

  //just an example of how we would inspect the last main loop inputs and outputs:
  printf("Last main loop inputs:\n");
  printf("  batt voltage = %6.3f\n",
         double(lastMainLoopInputs.batteryVoltage.value));
  printf("  JS buttons: ");
  if (lastMainLoopInputs.joystickInput.buttonRed)
    printf("buttonRed ");
  if (lastMainLoopInputs.joystickInput.buttonGreen)
    printf("buttonGreen ");
  if (lastMainLoopInputs.joystickInput.buttonBlue)
    printf("buttonBlue ");
  if (lastMainLoopInputs.joystickInput.buttonYellow)
    printf("buttonYellow ");
  if (lastMainLoopInputs.joystickInput.buttonStart)
    printf("buttonStart ");
  if (lastMainLoopInputs.joystickInput.buttonSelect)
    printf("buttonSelect ");
  printf("\n");
  printf("Last main loop outputs:\n");
  printf("  motor command 1 = %6.3f\n",
         double(lastMainLoopOutputs.motorCommand1));
  printf("  motor command 2 = %6.3f\n",
           double(lastMainLoopOutputs.motorCommand2));
  printf("  motor command 3 = %6.3f\n",
           double(lastMainLoopOutputs.motorCommand3));
  printf("  motor command 4 = %6.3f\n",
           double(lastMainLoopOutputs.motorCommand4));
}
