# IRT

## **InfraRed Thermometer Water/Heat Stress Sensing Project - 2019 Summary**  
Rebecca Hartman, Francis Santiago, Patricia Gonzalez-Medina

### Sensors
IRT Sensors from Melexis MLX90614 product line

![Canopy Sensor](https://github.com/precision-sustainable-ag/IRT/blob/master/Images/Canopy_Small.jpg)
![Leaf Sensor](https://github.com/precision-sustainable-ag/IRT/blob/master/Images/Leaf%20Sensor_Small.jpg)
![Soil Sensor](https://github.com/precision-sustainable-ag/IRT/blob/master/Images/Soil%20Sensor_Small.jpg)

**Canopy** - 90° FOV, **Leaf** - 35° FOV, **Soil** - 35° FOV

### Structure
PVC structure uses rotation instead of translation to adjust height of canopy sensor

![Structure](https://github.com/precision-sustainable-ag/IRT/blob/master/Images/Structure_Small.jpg)

### Boards
Components: MoteinoMEGA with LoRa transceiver, DS3231 RTC, microSD (Server only), custom PCBs

![Client & Server](https://github.com/precision-sustainable-ag/IRT/blob/master/Images/Client%20Server_Small.jpg)

**Client:** collects data from sensors, saves to Flash, send data to Server
**Server:** receives data from Servers, saves data to microSD

### Power

Components: 12V SLA Batteries, 12V 3W Solar panels, Charge controller, Battery box

![Battery & Controller](https://github.com/precision-sustainable-ag/IRT/blob/master/Images/Battery%20Charger_Small.jpg)
![Power Setup](https://github.com/precision-sustainable-ag/IRT/blob/master/Images/Power%20Setup_Small.jpg)



