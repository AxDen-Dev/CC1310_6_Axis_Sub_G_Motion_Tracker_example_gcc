# 6-Axis Sub-G LongRang Motion Tracker

<img src="./assets/axden_6_sub_g_motion_tracker.jpeg">
<br>

----

6-Axis Sub-G LongRang Motion Tracker collects essential information required for Motion Tracker using acceleration, gyro, and temperature sensors and transmits it using Sub-G communication.
<br>
<br>
This is an example that provides quick testing of various service scenarios that require long-distance communication of 1Km or more.
<br>

----

Device can be purchased from the Naver Smart Store.
<br>

[Purchase Link : naver smart store](https://smartstore.naver.com/axden)
<br>
<br>

You can purchase it by contacting sales@axden.io

----

### Key feature and functions

MCU | Description
:-------------------------:|:-------------------------:
CC1310 | Sub-G SoC

Sensors | Description
:-------------------------:|:-------------------------:
LSM6DSL | 6 Axis Accelerometer, Gyroscope Sensor
Si7201 | Hall Sensor
Si7501 | Temperature Sensor
Battery | 1/2 AA Battery 1200mAh

<br>

It is a motion tracker capable of Sub-G communication.
<br>

Sub-G wireless communication is performed using CC1310 SoC
<br>

It can be turned on and off using Si7201 hall sensor and magnet.
<br>

Use LSM6DSL to collect acceleration and gyro sensor values.
<br>

The temperature value is collected using the Si7051 sensor.
<br>

It operates for 5 years using a battery.
<br>

----

### Note

This program is not suitable for mass production and commercialization as an example program.
<br>

B2B customers should contact development@axden.io.
<br>

For B2B customers, we develop firmware optimized for customers' purposes, such as low power, stabilization, and communication with gateways, for free.
<br>

<table>
  <tr align="center">
    <td>Top</td>
    <td>Bottom</td>
  </tr>
  <tr align="center">
    <td><img src="./assets/axden_6_sub_g_motion_tracker_top.jpeg"></td>
    <td><img src="./assets/axden_6_sub_g_motion_tracker_bottom.jpeg"></td>
  </tr>
</table>

Pinmap can be found in the file ```board_define.h```
<br>

```

#define LED_RED_GPIO IOID_8
#define LED_BLUE_GPIO IOID_9

#define HALL_SENSOR_GPIO IOID_2

#define I2C_SDA IOID_6
#define I2C_SCL IOID_7


```

Sub-G communication-related settings can be found in the ```RadioTask.c``` file.
<br>

```

EasyLink_Params easyLink_params;
EasyLink_Params_init(&easyLink_params);
easyLink_params.ui32ModType = EasyLink_Phy_5kbpsSlLr;

if (EasyLink_init(&easyLink_params) != EasyLink_Status_Success)
{

    SysCtrlSystemReset();

}

EasyLink_getIeeeAddr(mac_address);

if (EasyLink_setFrequency(920000000) != EasyLink_Status_Success)
{

    SysCtrlSystemReset();

}

if (EasyLink_enableRxAddrFilter(mac_address, 8, 1)
        != EasyLink_Status_Success)
{

    SysCtrlSystemReset();

}

radio_packet_protocol.Packet.company_id[0] = COMPANY_ID >> 8;
radio_packet_protocol.Packet.company_id[1] = COMPANY_ID;

radio_packet_protocol.Packet.device_id[0] = DEVICE_TYPE >> 8;
radio_packet_protocol.Packet.device_id[1] = DEVICE_TYPE;

memcpy(radio_packet_protocol.Packet.mac_address, mac_address, 8);

radio_packet_protocol.Packet.control_number = 0;

SensorTask_registerPacketSendRequestCallback(sendPacketCallback);

while (1)
{

    Semaphore_pend(radioAccessSemHandle, BIOS_WAIT_FOREVER);

    EasyLink_setRfPower(TX_POWER);

    radio_sensor_data_packet_send(radio_packet_protocol);

    uint32_t events = Event_pend(radioEventHandle, 0,
    RADIO_TASK_EVENT_ALL,
                                 BIOS_WAIT_FOREVER);

    if (events == RADIO_TASK_EVENT_ACK)
    {

        recv_error_count = 0;

    }
    else if (events == RADIO_TASK_EVENT_ACK_TIMEOUT)
    {

        recv_error_count += 1;

        if (recv_error_count > RECV_ERROR_MAX_COUNT)
        {

            collection_cycle_timeout_count += 3;
            recv_error_count = 0;

        }

    }

}


```
