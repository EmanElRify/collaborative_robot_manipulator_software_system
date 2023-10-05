#include <Servo.h>
#include <math.h>

// trajectory parameters
const int t_f = 6;
const int DOF = 2;
float q_0[DOF] = {0, 109};
float q_f[DOF];
float qdot_0[DOF] = {0, 0};
float qdot_f[DOF] = {0, 0};
int min_delay = 6000;
float qOutPut[DOF][t_f + 1];
float qdotOutPut[DOF][t_f + 1];
float qddotOutPut[DOF][t_f + 1];

// Global variables
int current_angle_1 = 0; // current position of joint 1 (stepper motor)
int current_angle_2 ; // current position of joint 2 (servo motor)

struct Trajectory {
  float q;
  float qdot;
  float qddot;
};

Trajectory cubic_polynomial_trajectory(float q_0[], float q_f[], float qdot_0[], float qdot_f[], int t_f, int DOF) {
  float a0[DOF];
  float a1[DOF];
  float a2[DOF];
  float a3[DOF];

  Trajectory traj[DOF][t_f];

  for (int i = 0; i < DOF; i++) {
    a0[i] = q_0[i];
    a1[i] = qdot_0[i];
    a2[i] = (3.0 / pow(t_f, 2)) * (q_f[i] - q_0[i]);
    a3[i] = -(2.0 / pow(t_f, 3)) * (q_f[i] - q_0[i]);
  }

  for (int i = 0; i < DOF; i++) {
    for (int t = 0; t < t_f + 1; t++) {
      traj[i][t].q = a0[i] + a2[i] * pow(t, 2) + a3[i] * pow(t, 3);
      traj[i][t].qdot = 2 * a2[i] * t + 3 * a3[i] * pow(t, 2);
      traj[i][t].qddot = 2 * a2[i] + 6 * a3[i] * t;
      qOutPut[i][t] = traj[i][t].q;
      qdotOutPut[i][t] = traj[i][t].qdot;
      qddotOutPut[i][t] = traj[i][t].qddot;
      // Print the trajectory values
      Serial.print("DOF ");
      Serial.print(i);
      Serial.print(", t = ");
      Serial.print(t);
      Serial.print(": q = ");
      Serial.print(traj[i][t].q);
      Serial.print(", qdot = ");
      Serial.print(traj[i][t].qdot);
      Serial.print(", qddot = ");
      Serial.println(traj[i][t].qddot);

    }
  }
   //return traj[DOF - 1][t_f - 1];  // Return the trajectory at the last DOF and last time step
}
//Trajectory traj[DOF][t_f]; // declare the traj array

// Defining the connecting ports
int step_1 = 5;
int dir_1 = 2;
int en_1 = 8;
int servo_1 = 9;
int cnt = 1;
int DL = 1000;
//defining servo motors
Servo joint_2;

// declaring the current positions of each joint
int current_pos_1 = 0;
int current_pos_2;


int home_pos_1 = 0;
int home_pos_2 = 109;
int micro_step=8;
//defining step
float step_ = 1.8/micro_step;

//defining limit angles
int lowerlimit_angle_1 = -90;
int upperlimit_angle_1 = 180;
int lowerlimit_angle_2 = -109;
int upperlimit_angle_2 = 71;

int lowerlimit_steps_1 = lowerlimit_angle_1 / step_;
int upperlimit_steps_1 = upperlimit_angle_1 / step_;

// defining the temporary variables
const unsigned int MAX_MESSAGE_LENGTH = 50;      // arbitrary length

// Parsing variables to capture the parameters
int8_t indexA, indexB;
String theta1, theta2;

// Converted final variables, used to run the motor
float angle1, angle2;
int steps_1;
float stepsRev;

/**
 * setup - runs once, containing the hardware configurations
 * 
 * Description: a function the sets up the Arduino I/O configuration, and starting up serial communication and stepper motors
 * Serial: starting the serial communication with baudrate = 9600
 * Stepper: configuring pins connected between Arduino and Stepper driver TB6600 (EN+, PUL+, DIR+) and starting up the motors (Off-line control enable)
 * Servo: configuring pins connected between Arduino and servo motors (PWM Signal)
 * Return: None
*/

void setup() {
  //Trajectory traj[DOF][t_f];

  Serial.begin(9600);
  /*
  * Hardware configurations
  */
 // stepper pins configuration
  pinMode(step_1, OUTPUT);
  pinMode(dir_1, OUTPUT);
  pinMode(en_1, OUTPUT);
  pinMode(servo_1, OUTPUT);

  // servo motors configurations
  joint_2.attach(servo_1);
  joint_2.write(109);
  //starting up stepper motors 
  digitalWrite(en_1, LOW);       //off-line enables
  }

void loop() 
{
  // Check to see if anything is available in the serial receive buffer
  while (Serial.available() > 0)
  {
    // Create a place to hold the incoming message
    static char motData[MAX_MESSAGE_LENGTH];            //created a character array named message
    static unsigned int index_pos = 0;                  //this variable will allow us to choose where in the array to put the incoming bites 

    // Read the next available byte in the serial receiver buffer
    char charData = Serial.read();

    if (charData != '\n')
    {
      motData[index_pos] = charData;
      index_pos++ ;  
    }
    else
    {
      motData[index_pos] = '\0';

      index_pos = 0;

      Parse_the_Data(motData);
      Serial.println(angle1);
      Serial.println(angle2);
      //Trajectory traj[DOF][t_f];
      cubic_polynomial_trajectory(q_0, q_f, qdot_0, qdot_f, t_f, DOF);
      //Trajectory traj[DOF][t_f];  // Declare the traj array

      //float myArray[2][10]=cubic_polynomial_trajectory(q_0, q_f, qdot_0, qdot_f, t_f, DOF);

      //float q_at_t_4 = myArray[1][4];  // Access q at t = 4

       
      //Serial.println(qOutPut[1][4]);
      //joint_move_modified();
      moveOnTrajectoryWaypoints();
      delay(3000);
      home();
    }
  }
}

/**
 * Parse_the_Data - analyzing the messege into parts and extract important informations
 * @dataIn: a string arrays of data that is needed to be parsed
 * Return: joint_angle an array of the joint angles
*/
long Parse_the_Data(String dataIn){
  
      Serial.println(dataIn);
      indexA = dataIn.indexOf("A");
      indexB = dataIn.indexOf("B");

      theta1 = dataIn.substring(0, indexA);
      angle1 = theta1.toFloat();
      angle1 > upperlimit_angle_1? angle1 = upperlimit_angle_1 : angle1 < lowerlimit_angle_1? angle1 = lowerlimit_angle_1 : Serial.println("Angle 1 is reachable") ;
      steps_1 = angle1/step_;
      theta2 = dataIn.substring(indexA+1, indexB);
      angle2 = theta2.toFloat();
      angle2 > upperlimit_angle_2? angle2 = upperlimit_angle_2 : angle2 < lowerlimit_angle_2? angle2 = lowerlimit_angle_2 : Serial.println("Angle 2 is reachable");
      angle2 = map(angle2,-109, 71, 0, 180);
      q_f[0] = {angle1};
      q_f[1] = {angle2};
}
int convert_to_steps(float angle, float stepS)
{
  return angle / stepS;
}

int delay_velocity_stepper(float delta_theta, float omega_0, float acc)
{
  int delay_;
  if (acc != 0)
  {
    float a = 0.5 * acc;
    float b = omega_0;
    float c = -1 * delta_theta;
   // int dt = (-b + sqrt((pow(b,2) - 4 * a * c)) / 2 * a);
    int dt = (-b + sqrt((pow(b, 2) - 4 * a * c))) / (2 * a);

    delay_ = abs(0.5 * (dt / (delta_theta/step_)));
    return (delay_*1e+6);

  }
  else
  {
    delay_ = abs(135000 / omega_0);
    return(delay_);
  }

}

int delay_velocity_servo(float delta_theta, float omega_0, float acc)
{
  int delay_;
  if (acc != 0)
  {
    float a = abs(0.5 * acc);
    float b = omega_0;
    float c = -1 * delta_theta;
    int dt = (-b + sqrt((pow(b,2) - 4 * a * c))) / (2 * a);
    delay_ = abs(dt / (delta_theta));
    return(delay_*1e+3);
  }
  else
  {
    delay_ = abs(354 / omega_0);
    return (delay_);

  }
}

void moveOnTrajectoryWaypoints(){
  for (int t = 0; t <= t_f; t++) {
    float desired_pos_1 = qOutPut[0][t];
    float desired_vel_1 = qdotOutPut[0][t];
    float desired_acc_1 = qddotOutPut[0][t];
    float desired_acc_2 = qddotOutPut[1][t];
    float desired_pos_2 = qOutPut[1][t];
    float desired_vel_2 = qdotOutPut[1][t];
    current_angle_2 = joint_2.read();
    float angular_distance_1 = desired_pos_1 - current_angle_1;
    float angular_distance_2 = desired_pos_2 - current_angle_2;
    int delay_time_1 = delay_velocity_stepper(angular_distance_1, desired_vel_1, desired_acc_1);
    delay_time_1 < min_delay ? delay_time_1 = min_delay : Serial.println("Velocity is within range");
    int steps_1 = convert_to_steps(desired_pos_1, step_); // convert desired position to steps
    int delay_time_2 = delay_velocity_servo(angular_distance_2, desired_vel_2, desired_acc_2);
    joint_move(steps_1, desired_pos_2, delay_time_1, delay_time_2);
    current_angle_1 = current_angle_1 + desired_pos_1;
    current_angle_2 = joint_2.read();
    Serial.print("done waypoint ");
    Serial.println(t);
    
  }
}

void joint_move( int steps_1, float desired_pos_2, int delay_time_1, int delay_time_2) {
  float current_time = millis();
  //joint 1 (stepper motor)
  int current_1 = current_pos_1;
  int delay1 = delay_time_1;
  int delay2 = delay_time_2;
  if (steps_1 > current_1)
  {
    digitalWrite(dir_1, HIGH);
    for (int i = current_pos_1; i <  steps_1; i++)
    {
      // Check for incoming data and stop the motors if an interrupt is received
      if (Serial.available() > 0) {
        char incoming = Serial.read();
        if (incoming == 'I') {
          stopMotors();
          unsigned long startTime = millis();
          unsigned long duration = 10000; // 10 seconds duration
          while (millis() - startTime < duration) {
            if (Serial.available() > 0 && Serial.read() == 'I') {
              // Restart the loop from the beginning
              startTime = millis(); // Reset the start time
             continue; // Restart the loop
            }
          }
          // Resume motor movement
          continue;
        }
      }

      digitalWrite(step_1, HIGH);
      delayMicroseconds(delay1);
      digitalWrite(step_1, LOW);
      delayMicroseconds(delay1);
      current_pos_1 = current_pos_1 + 1;
      if (current_pos_1 > upperlimit_steps_1) break;
    }
  }
  else if (steps_1 < current_1)
  {
    digitalWrite(dir_1, LOW);
    for (int i = current_pos_1; i >  steps_1; i--)
    {
     // Check for incoming data and stop the motors if an interrupt is received
      if (Serial.available() > 0) {
        char incoming = Serial.read();
        if (incoming == 'I') {
          stopMotors();
          unsigned long startTime = millis();
          unsigned long duration = 10000; // 10 seconds duration
          while (millis() - startTime < duration) {
            if (Serial.available() > 0 && Serial.read() == 'I') {
              // Restart the loop from the beginning
              startTime = millis(); // Reset the start time
             continue; // Restart the loop
            }
          }
          // Resume motor movement
          continue;
        }
      }

      digitalWrite(step_1, HIGH);
      delayMicroseconds(delay1);
      digitalWrite(step_1, LOW);
      delayMicroseconds(delay1);
      current_pos_1 = current_pos_1 - 1;
      if (current_pos_1 < lowerlimit_steps_1) break;
    }
  }
  float time_stepper = millis() - current_time;
  Serial.print("Time for stepper: ");
  Serial.println(time_stepper);
  current_time = millis();
  //joint 2 (servo motor)
  current_pos_2 = joint_2.read();
  if (desired_pos_2 > current_pos_2)
  {
     for (int i = current_pos_2; i <= desired_pos_2; i++)
     {
        // Check for incoming data and stop the motors if an interrupt is received
      if (Serial.available() > 0) {
        char incoming = Serial.read();
        if (incoming == 'I') {
          stopMotors();
          unsigned long startTime = millis();
          unsigned long duration = 10000; // 10 seconds duration
          while (millis() - startTime < duration) {
            if (Serial.available() > 0 && Serial.read() == 'I') {
              // Restart the loop from the beginning
              startTime = millis(); // Reset the start time
             continue; // Restart the loop
            }
          }
          // Resume motor movement
          continue;
        }
      }

        joint_2.write(i);
        delay(delay2);
     }
  }
  else
  {
    for (int i = current_pos_2; i >= desired_pos_2; i--)
    {
      // Check for incoming data and stop the motors if an interrupt is received
      if (Serial.available() > 0) {
        char incoming = Serial.read();
        if (incoming == 'I') {
          stopMotors();
          unsigned long startTime = millis();
          unsigned long duration = 10000; // 10 seconds duration
          while (millis() - startTime < duration) {
            if (Serial.available() > 0 && Serial.read() == 'I') {
              // Restart the loop from the beginning
              startTime = millis(); // Reset the start time
             continue; // Restart the loop
            }
          }
          // Resume motor movement
          continue;
        }
      }

        joint_2.write(i);
        delay(delay2);
    }
  }
  float time_servo = millis() - current_time;
  Serial.print("Time for servo: ");
  Serial.println(time_servo);

}

/**
 * joint_move_modified() - moving the joints with desired angles
 * 
 * Description:
 * joint 1: checking whether the desired angle is negative or positive to send a dir impulse and rotating the rotor CW or CCW
 * joint 2: checking whether the desired angle is negative or positive to send a dir impulse and rotating the rotor CW or CCW
 * Return: None
*/


void stopMotors() {
  // Stop the motors by setting the step pins low
  digitalWrite(step_1, LOW);
  current_pos_2 = joint_2.read();
  joint_2.write(current_pos_2);
}


/**
 * home - returning motors to home position
 * Return: None
*/
void home()
{
  //homing joint 1
  current_pos_1 > 0? digitalWrite(dir_1, LOW) : digitalWrite(dir_1, HIGH);
  for (int i = 0; i <= abs(current_pos_1); i++)
  {
    // Check for incoming data and stop the motors if an interrupt is received
    if (Serial.available() > 0) {
      char incoming = Serial.read();
      if (incoming == 'I') {
        stopMotors();
        unsigned long startTime = millis();
        unsigned long duration = 10000; // 10 seconds duration
        while (millis() - startTime < duration) {
          if (Serial.available() > 0 && Serial.read() == 'I') {
            // Restart the loop from the beginning
            startTime = millis(); // Reset the start time
           continue; // Restart the loop
          }
        }
        // Resume motor movement
        continue;
      }
    }
    digitalWrite(step_1, HIGH);
    delayMicroseconds(10000);
     digitalWrite(step_1, LOW);
    delayMicroseconds(10000);
  }
  
  //homing joint 2
  //joint_2.write(home_pos_2);
  //current_pos_2 = joint_2.read();
  //joint 2 (servo motor)
  current_pos_2 = joint_2.read();
  if (home_pos_2 > current_pos_2)
  {
     for (int i = current_pos_2; i < home_pos_2; i++)
     {
        // Check for incoming data and stop the motors if an interrupt is received
      if (Serial.available() > 0) {
        char incoming = Serial.read();
        if (incoming == 'I') {
          stopMotors();
          unsigned long startTime = millis();
          unsigned long duration = 10000; // 10 seconds duration
          while (millis() - startTime < duration) {
            if (Serial.available() > 0 && Serial.read() == 'I') {
              // Restart the loop from the beginning
              startTime = millis(); // Reset the start time
             continue; // Restart the loop
            }
          }
          // Resume motor movement
          continue;
        }
      }

        joint_2.write(i);
        delay(15);
     }
  }
  else
  {
    for (int i = current_pos_2; i > home_pos_2; i--)
    {
      // Check for incoming data and stop the motors if an interrupt is received
      if (Serial.available() > 0) {
        char incoming = Serial.read();
        if (incoming == 'I') {
          stopMotors();
          unsigned long startTime = millis();
          unsigned long duration = 10000; // 10 seconds duration
          while (millis() - startTime < duration) {
            if (Serial.available() > 0 && Serial.read() == 'I') {
              // Restart the loop from the beginning
              startTime = millis(); // Reset the start time
             continue; // Restart the loop
            }
          }
          // Resume motor movement
          continue;
        }
      }

        joint_2.write(i);
        delay(15);
    }
  }

  current_pos_1 = home_pos_1;

}
