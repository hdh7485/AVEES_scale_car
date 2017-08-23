# AVEES Scale Car Project
This is Scale Car project at Kookmin University AVEES laboratory.
## Hardware
### Motor Drivers
- DC Motor Driver
  - MW-MDC24D500D
  - <http://www.ntrexgo.com/wp-content/uploads/2013/10/MoonWalker-Motor-Controllers-User-Manual1.pdf>
- Step Motor Driver
  - MD5-HD14 Series
  - <http://www.autonics.co.kr/products/products_detail.php?catecode=03/01/02&db_uid=122>
### Motors
- 48V DC Motor
- 24V DC Motor
  - my1020zxfh-450w-24v
- 24V Step Motor
### Sensors
- Encoder
- Mobis Angle Sensors (2)
  - CAN protocol
  - Little Endian
  
  |ID|DLC|0|1|2|3|4|
  |--|---|-|-|-|-|-|
  |0x2B0|5|angle low byte|angle high byte|speed low byte|speed high byte|checksum|
 
- Ultrasonic Sensors (2)
  - <http://www.das-co.com/bbs_form/Fckeditor/upload/UDS10%20catalog_KOR_150312_R.pdf>
### Arduino
- Arudino UNO
- Arduino CAN bus shield (2)
  - High-speed CAN
  - CAN-to-SPI
  - <http://wiki.seeed.cc/CAN-BUS_Shield_V1.2/> </br>
    - Top: CAN bus shield (CS:9)</br>
    - Middle: CAN bus shield (CS:10) </br>
    - Bottom: Arduino

### Etc.
- Use CAN Bus
- Use CANlink
