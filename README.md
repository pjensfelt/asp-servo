# asp_servo_api
EtherCat servo API utilizing the SOEM ethercat master

## Prerequisistes

`sudo apt install build-essential cmake`

## Installation instructions

Clone this repository (including submodules)

```
git clone --recursive https://github.com/pjensfelt/asp_servo_api.git
cd aspservo_api
mkdir build
cd build
cmake ..
make
sudo make install
```

If the above failed you probably used autopilot when you cloned the repo, and forgot the `--recurvise` option, which means that the submodules where not checked out. In that case you need to got into the asp_servo_api directory and manually init the submodules

```
cd /some/path/to/asp_servo_api
git submodule update --init --recursive
```

## How to use

There should be an XML file describing the system configuration (see for example https://github.com/urban-eriksson/asp-servo-testapp).

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
