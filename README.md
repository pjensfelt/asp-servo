# asp_servo_api
EtherCat servo API utilizing the SOEM ethercat master

## Prerequisistes and dependencies

`sudo apt install build-essential cmake`

### tinysml2
```
git clone https://github.com/pjensfelt/tinyxml2.git
cd tinyxml2
mkdir build
cd build
cmake ..
make
sudo make install
```
### SOEM
```
git clone https://github.com/pjensfelt/SOEM.git
cd SOEM
mkdir build
cd build
cmake ..
make
sudo make install
```
Test the installation by running for example slaveinfo which should give you an output like this if you have no slaves attached
```
patric@PJMSI:~$ slaveinfo 
SOEM (Simple Open EtherCAT Master)
Slaveinfo
Usage: slaveinfo ifname [options]
ifname = eth0 for example
Options :
 -sdo : print SDO info
 -map : print mapping
Available adapters
Description : lo, Device to use for wpcap: lo
Description : enp3s0, Device to use for wpcap: enp3s0
Description : wlo1, Device to use for wpcap: wlo1
End program
```

### Update LD_LIBRARY_PATH
If you get errors like `slaveinfo: error while loading shared libraries: libsoem.so: cannot open shared object file: No such file or directory` you need to tell Linux where to find the newly installed libraries
```
sudo ldconfig
```

## Installation instructions

```
git clone https://github.com/pjensfelt/asp_servo_api.git
cd asp_servo_api
mkdir build
cd build
cmake ..
make
sudo make install
```

## How to use

There should be an XML file describing the system configuration (see for example)
```
<?xml version="1.0" encoding="utf-8"?>
<SystemConfiguration>
    <EthernetPort></EthernetPort>
    <CycleTime_ms></CycleTime_ms>
    <ServoCollection>
       <Servo position="" name="">
          <Type></Type>
          <StartupParameters>
            <!-- Items -->                        
          </StartupParameters>
          <PDOmapping>
            <Tx>
              <!-- Objects -->
            </Tx>
            <Rx>
              <!-- Objects -->
            </Rx>
          </PDOmapping>
       </Servo>
       <!-- More servos -->
    </ServoCollection>
</SystemConfiguration>
```
An object is as defined in the manual, e.g.:

```
<Object>
   <Name>Position</Name>
   <Index>0x6064</Index>
   <Subindex>0</Subindex>
   <Type>INT32</Type>
</Object>
```

The name is used when writing to and reading from the servo.

`servo_collection.write(servo_name,"Position",target_position);`

Be sure to use the correct data type for the target value.


In the StartupParameters tag of the XML file, an object is combined with a value to form an Item:

```
<Item>
   <Object>
      <Name>Mode of Operation</Name>
      <Index>0x6060</Index>
      <Subindex>0</Subindex>
      <Type>INT8</Type>
   </Object>
   <Value>10</Value>
</Item>  
```  
