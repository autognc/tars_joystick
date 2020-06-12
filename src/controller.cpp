#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <tf/transform_datatypes.h>
#include <joyDrivers.h>
#include <stdio.h>
#include <SerialPort.h>
#include "sabertooth_2x25_driver.h"
#define pi 3.14159265


joyStruct joystick;
int leftw,rightw;
std_msgs::Int16MultiArray twist;
//ros::Publisher vel_pub_;
ros::Subscriber joy_sub_;
SerialPort ser("/dev/ttyS0");
int direction = 0;
ros::Time LastWrite,timeNow;
ros::Duration writeDelay(1);
void joyCallback(sensor_msgs::Joy joy)
{
  joystick = driverXbox360Wired(joy);
  // rightw = 200*joystick.RstickVer;
  // leftw = 200*joystick.RstickVer;
  // rightw+=50*joystick.LstickHor;
  // leftw-=50*joystick.LstickHor;
  timeNow = ros::Time::now();
  if(joystick.buttonX && timeNow-LastWrite>writeDelay)
  {
    control_motors_sep(6,100,7,100,128);
    control_motors_sep(6,64,7,64,129);
    direction = 1;
    LastWrite = timeNow;

  }
  else if(joystick.buttonY && timeNow-LastWrite>writeDelay)
  {
    control_motors_sep(6,64,7,28,128);
    control_motors_sep(6,64,7,100,129);
    direction = 1;
    LastWrite = timeNow;
  }
  else if(joystick.buttonB && timeNow-LastWrite>writeDelay)
  {
    control_motors_sep(6,28,7,64,128);
    control_motors_sep(6,64,7,28,129);
    direction = 1;
    LastWrite = timeNow;
  }
  else if(joystick.buttonA && timeNow-LastWrite>writeDelay)
  {
    control_motors_sep(6,28,7,100,128);
    control_motors_sep(6,64,7,100,129);
    direction = 1;
    LastWrite = timeNow;
  }
   
  // twist.data[1] = rightw;
  //vel_pub_.publish(twist);


}

void writeToPort(char data){
    bool written=false;
    char* write = &data;
    //ser.flush();
    while(!written){
        try{
            ser.Write(write);
            written=true;
            //control_motors_sep(0,0,4,0,128);
        }
        catch(std::exception& e){
            continue;
        }
    }
}

/**********************************************************************************************
 * Function:        static void send_command ( uint8_t command, uint8_t value, uint8_t address )
 *
 * Pre-Condition:   None
 * Input:           Receives the command data from the driver functions
 * Output:          Sends the three commands plus their checksum to the serial port, and through
 *                      that to the Sabertooth Motor Controller
 * Side Effects:    None
 * Overview:        None
 * Notes:           Static helper function, for use only by driver functions in this file
 *********************************************************************************************/

    /* Helper Command, for internal driver use only
     * Defining it here, in the .c file, instead of in the .h file, to prevent
     * compiler warning about it being declared "static", but never defined
     * This is correct, since it is not for the user, only the user's functions */

    

/*********************************************************************************************/
static void send_command ( uint8_t command, uint8_t value, uint8_t address ) {
    //assert  ( command < COMMAND_HIGH_LIMIT);
    // putchar ( address );
    // putchar ( command );
    // putchar ( value );
    // putchar ( ( address + command + value ) & CRC_MASK );

    writeToPort( (char) address );
    writeToPort( (char) command );
    writeToPort( (char) value );
    writeToPort( (char) (( address + command + value ) & CRC_MASK) );
}

/**********************************************************************************************
 * Function:        uint8_t control_motors_sep ( uint8_t command1, uint8_t speed1, \
 *                                               uint8_t command2, uint8_t speed2, \
 *                                               uint8_t address )
 *
 * Pre-Condition:   None
 * Input:           Receives command data from the application program
 * Output:          Sends the commands to the send_command function,
 *                      and then to the serial port
 * Side Effects:    None
 * Overview:        Checks commands for validity, and passes them to the serial port
 * Notes:           This function is valid for Sabertooth Commands 0, 1, 4 - 7
 *                  These commands are for controlling two motors with individual settings, a single motor at a time
 *
 *                      Individual Motor Commands:
 *                          0:  Drive Forward      Motor 1
 *                          1:  Drive Reverse      Motor 1
 *                          4:  Drive Forward      Motor 2
 *                          5:  Drive Reverse      Motor 2
 *                          6:  Drive 7-Bit        Motor 1
 *                          7:  Drive 7-Bit        Motor 2
 *
 *********************************************************************************************/

uint8_t control_motors_sep ( uint8_t command1, uint8_t speed1, \
                             uint8_t command2, uint8_t speed2, \
                             uint8_t address ) {
// = MOTOR_DRIVER_ADDRESS_1 ) { // If your compiler allows overloading, feel free to un-comment the assignment
// and move it immediately after the "address" variable
    if ( ( command1 < COMMAND_LOW_LIMIT || command1 > DRIVE_MOTOR_2_7_BIT ) || \
         ( command2 < COMMAND_LOW_LIMIT || command2 > DRIVE_MOTOR_2_7_BIT ) ) {

   /*     Set error code for invalid command
    *     Call a user error function to do whatever your application requires, such as:
    *         mSTOP_ALL_MOTORS;  */
        return FALSE;
    }

    else {
      
        send_command ( command1, speed1, address );
        send_command ( command2, speed2, address );
//      Set error code for no error
        return TRUE;
    }
}

uint8_t set_baudrate ( uint8_t desired_baudrate, uint8_t address ) {

    static uint8_t new_baudrate = DEFAULT_BAUDRATE;

    if ( desired_baudrate < BAUDRATE_2400 || \
         desired_baudrate > BAUDRATE_38400 ) {

            new_baudrate = DEFAULT_BAUDRATE;
            return FALSE;                                   // Set error code for error
    }

    else {
        new_baudrate = desired_baudrate;
    }

    send_command ( SET_BAUD_RATE, new_baudrate, address );
    return TRUE;                                            // Set error code for no error
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  ros::NodeHandle nh_;
  ros::Rate loop_rate(10);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);
  std::cout<<"Something"<<std::endl;

  ser.Open();
  //SerialPort ser;
   //ser.Open("/dev/ttyS0");
  ser.SetBaudRate(SerialPort::BAUD_9600);
//  set_baudrate(2,128);

  set_baudrate(2,128);
  set_baudrate(2,129);
  std::cout<<"Start"<<std::endl;
  //control_motors_sep(6,10,7,100,129);
  //control_motors_sep(6,10,7,100,128);
  // control_motors_sep(6,10,7,64,129);
  // control_motors_sep(6,64,7,64,128);
  while(ros::ok()){
  //control_motors_sep(0,1,4,1,128);
  //   std::cout<<"Start"<<std::endl;
  // control_motors_sep(6,10,7,100,129);
  // control_motors_sep(6,10,7,100,128);
  // ros::Duration(5).sleep();
  //  std::cout<<"Stop"<<std::endl;
  //  control_motors_sep(6,64,7,64,129);
  //  control_motors_sep(6,64,7,64,128);
  //   ros::Duration(5).sleep();
 //  if(direction==1)
   //{
    //std::cout<<"direc\n";
   // control_motors_sep(6,10,7,100,129);
  //control_motors_sep(6,10,7,100,128);
    //ros::Duration(5).sleep();
  // direction = 0;
   if(direction && ros::Time::now()-LastWrite>writeDelay)
   {
   control_motors_sep(6,64,7,64,129);
   control_motors_sep(6,64,7,64,128);
   //ros::Duration(1).sleep();
   direction = 0;
   //std::cout<<"Stop\n";
   }

  //vel_pub_ = nh_.advertise<std_msgs::Int16MultiArray>("/cmd", 10);
  
  ros::spinOnce();
  loop_rate.sleep();
}
control_motors_sep(6,64,7,64,129);
   control_motors_sep(6,64,7,64,128);
ser.Close();
return 0;
}
