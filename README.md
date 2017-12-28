# 86MIPCar

Mobile inverted pendulum with 86Duino.

This project is inspired by JJROBOTS' B-ROBOT, which is a self balanced Arduino robot with stepper motors.
One can refer to their project at http://jjrobots.com/b-robot

### Required hardware

1. 86Duino One
2. Prototype shield for Arduino Uno 
3. A4988 Stepper motor driver carrier x2 (https://www.pololu.com/product/1182)
4. NEMA17 Stepper motor x2 (http://www.motechmotor.com/products_detail.php?id=147&cid=70&page=1)
5. 2 channels RC transmitter and receiver

### Build environment

86Duino IDE, coding 315 (download page : http://www.86duino.com/?page_id=8918)

### Build procedure

1. Connect all wires by following the attached connection document.
2. Connect 86Duino One to your PC's usb port.
3. Open 86Duino IDE.
4. Open mip.ino from its corresponding folder.
5. Compile mip.ino and download the compiled binary file into the connected 86Duino One.

### Known issue

PID gains depend on the chosen stepper motors and other mechanical parts. They should be tuned carefully. 


