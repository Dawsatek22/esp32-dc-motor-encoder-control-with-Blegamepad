// This is a code on how to control a dc motor with encoder(I use the:Motor CHR-775S-ABHL DC Magnetic Holzer Encoder Motor 24.0V8000RPM 12.0V4000RPM Robot Driving Motor (Size : 24V)),
// a gamepad(I used Xbox Wireless (model 1708, 2 buttons) for this code ),A esp32 microcontroller , and a cytron  MMD01A dc motor driver.
// I recommend to look into first how to install and use this library for gamepad control int link below.
// https://bluepad32.readthedocs.io/en/latest/


// link of the motor https://www.amazon.com/CHR-775S-ABHL-Magnetic-Encoder-24-0V8000RPM-12-0V4000RPM/dp/B0CPVFHD17?th=1
#include<Bluepad32.h> // Is the library we use to send gamepad signals. 

ControllerPtr myControllers[BP32_MAX_GAMEPADS]; // Is the constructor.

int  pos = 0; // is the position that get measured.
int max_left = -360; // The maximal value for the leftwards position.
int max_right = 360;
int lread  = 25; // Is the sensor pin for the left direction.Reads 1 when motor direction goes left.
int rread = 26; // Is the sensor pin for the right direction.Reads 1 when motor direction goes right.
int dir = 12; // Is the direction pin for the motor driver.
int pwm = 13; // Is the pwm pin for the motor drive


void onConnectedController(ControllerPtr ctl) {
bool foundEmptyslot = false;
for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {


  if (myControllers[i] == nullptr) {

    Serial.printf("Callback gamepad is not connected");
    ControllerProperties properties = ctl->getProperties();
    Serial.printf("controller model: %s, VID=0x%4x,PID=0x%04x\n",ctl->getModelName().c_str(), properties.vendor_id, properties.product_id );
    myControllers[i] = ctl;
    foundEmptyslot = true;
    break;
  }
}


    if (!foundEmptyslot) {
      Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}


void onDisconnectedController(ControllerPtr ctl ) {
bool foundController = false;
for (int i = 0; i <BP32_MAX_GAMEPADS; i++ ) {

  if (myControllers[i] == ctl ) {


Serial.printf("Callback: controller disconnected");
myControllers[i] = nullptr;
  
  foundController = true;
  break;
  }


}


if (!foundController) {

  Serial.printf("Callback: Controller disconnected but not found in my  my controllers");
}

}


// Below prints the  function joystick Values.
void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
  "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
  "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
  ctl->index(),        // Controller Index
  ctl->dpad(),         // D-pad
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(),        // (-511 - 512) left X Axis
  ctl->axisY(),        // (-511 - 512) left Y axis
  ctl->axisRX(),       // (-511 - 512) right X axis
  ctl->axisRY(),       // (-511 - 512) right Y axis
  ctl->brake(),        // (0 - 1023): brake button
  ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
  ctl->miscButtons(),  // bitmask of pressed "misc" buttons
  ctl->gyroX(),        // Gyro X
  ctl->gyroY(),        // Gyro Y
  ctl->gyroZ(),        // Gyro Z
  ctl->accelX(),       // Accelerometer X
  ctl->accelY(),       // Accelerometer Y
  ctl->accelZ()        // Accelerometer Z
  );
}




void processGamepad(ControllerPtr ctl) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...
 
  //== xbox one A button
  if (ctl->buttons() == 0x0001) {
  Serial.printf("A is pressesd");
  }


  //== Xbox one  x  button = 0x0004 ==//
  if (ctl->buttons() == 0x0004) {
Serial.printf("X is pressesd");
  }

  //== Xbox one Y button = 0x0008 ==//
  if (ctl->buttons() == 0x0008) {
 Serial.printf("Y is pressesd");
  }

  //== Xbox one B button = 0x0002 ==//
  if (ctl->buttons() == 0x0002) {
 Serial.printf("B is pressesd");
  }






  //== Xbox one R1 trigger button = 0x0020 ==//
  if (ctl->buttons() == 0x0020) {
   Serial.printf("RB is pressesd");
  }
 

  //== Xbox one  RT trigger button = 0x0080 ==//
  if (ctl->buttons() == 0x0080) {
    Serial.printf("RT is pressesd");
  }


  //== Xbox one LB trigger button = 0x0010 ==//
  if (ctl->buttons() == 0x0010) {
  Serial.printf("LB is pressesd");
  }

  //== Xbox one  LT trigger button = 0x0040 ==//
  if (ctl->buttons() == 0x0040) {
  Serial.printf("LT is pressesd");
  }

// Below are the commands to control the motor  direction.

  //== Xbox one LEFT JOYSTICK - UP ==//
  if (ctl->axisY() <= -36) {
  
 digitalWrite(pwm, 125);
  digitalWrite(dir, HIGH);
 
  Serial.println("direction is goes left  and position is");
 
delay(500);
 digitalWrite(pwm, LOW);
  digitalWrite(dir, HIGH);

    }

  //== Xbox one LEFT JOYSTICK - DOWN ==//
  if (ctl->axisY() >= 36) {
   digitalWrite(pwm, 125);
  digitalWrite(dir, LOW);
  
  Serial.println("direction is goes right and position is ");
  
delay(500);
 digitalWrite(pwm, LOW);
  digitalWrite(dir, HIGH);
  }


  //==  Xbox one  LEFT JOYSTICK DEADZONE ==//
  if (ctl->axisY() > -25 && ctl->axisY() < 25 && ctl->axisX() > -25 && ctl->axisX() < 25) {
    // code for when left joystick is at idle
  }
 //==  Xbox one RIGHT JOYSTICK - X AXIS ==//
  if (ctl->axisRX()) {
    // code for when right joystick moves along x-axis
  }

  //==  Xbox one RIGHT JOYSTICK - Y AXIS ==//
  if (ctl->axisRY()) {
  // code for when right joystick moves along y-axis
  }
  dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() ) {
      if (myController->isGamepad()) {
         processGamepad(myController);
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}
  void setup() {
  Serial.begin(115200);
  // below are the pin setup
  pinMode(lread,INPUT);
pinMode(lread,INPUT);
pinMode(pwm,OUTPUT);
pinMode(dir,OUTPUT);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.

}

void loop() {
 int l = digitalRead(lread);
int r = digitalRead(lread);
Serial.print("L reads ");
Serial.print("  ");
Serial.println(l);
Serial.print("  ");
Serial.print("R reads ");
Serial.print("  ");
Serial.println(r);
bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();


// This if statement prints when the motor direction goes left.
if (  l == 1 && r == 0) {
Serial.println("pos is going left");
pos -= 1;
Serial.print(pos);

}
// This if statement prints when the motor direction goes right.
if ( l == 0 && r == 1) {
Serial.println("pos is going right");
pos += 1;
Serial.print(pos);

}

// if statement prints postion if reached maximum position right.
  if ( pos >max_right) {
Serial.printf("direction hit right limit  ");
pos =  max_right;
 digitalWrite(pwm, LOW);
  digitalWrite(dir, HIGH);

   }

// if statement prints postion if reached maximum position left.
   if ( pos < max_left) {
Serial.printf("direction hit left limit  ");
pos =  max_left;

 digitalWrite(pwm, LOW);
  digitalWrite(dir, HIGH);
   }
   
    // vTaskDelay(1);
  delay(150);
}