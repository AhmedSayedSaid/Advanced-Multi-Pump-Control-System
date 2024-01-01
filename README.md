# Advanced Multi-Pump Control System: Comprehensive Guide for Optimal Pressure Management and Efficient Operation

## Overview of System

**System Composition**: The system is composed of a sophisticated control board, designed to interface with Variable Frequency Drives (VFDs) or directly with pumps. Its primary function is to regulate the operation of these VFDs or pumps, aiming to maintain a specific pressure set point with maximum efficiency and fast response time.

**Hardware and Connectivity**: The hardware facilitates seamless connectivity, allowing users to interact with the board via Bluetooth Low Energy (BLE) through various platforms, including mobile phones, laptops, and desktop computers. This system supports a diverse range of applications, including mobile apps for iOS, web applications, and desktop software, offering users flexibility in monitoring and managing the system.

## User Interface and Control

Users can effortlessly access comprehensive system diagnostics, real-time operational statuses, and modify system Parameters through the user-friendly interface of the apps. The system employs a Proportional-Integral-Derivative (PID) controller for precise speed and operational control. Additionally, it features a dual scheduler system: one to ensure balanced working hours across all pumps, and another to enable user-defined operational schedules based on specific days, times, or date-triggered events.

## Communication Protocol and Customizability

Leveraging the Modbus communication protocol, the system offers compatibility with any Modbus-enabled machinery, thus providing users with the capability to transmit custom commands. Knowledgeable users can directly communicate with and control various devices connected to the system.

## Enhanced Accessibility and Monitoring

Accessibility is further enhanced through a bar-code scanning feature, allowing quick and easy connection to the system. The system archives a detailed history of alarms, facilitating effective troubleshooting and maintenance. Additionally, a dedicated monitoring panel is available for each pump, displaying vital data such as current speed, actual pressure, and target set points through interactive graphs and visualizations.


## High-Level System Diagram and Description of System Components and Their Interactions

## Features

### User Interface

- **Mobile App**: Provides on-the-go access and system control.
- **Web App**: Allows access via web browsers for a more comprehensive view.


### FEATURES THAT THE SYSTEM PROVIDE

- **Different Access Levels**: With each level having its own settings.

  ![Login Screen](https://github.com/AhmedSayedSaid/Advanced-Multi-Pump-Control-System/blob/main/Advanced%20Multi-Pump%20Control%20System/app-images/login.png)

- **Full System Condition Monitor and Manual Control Over Each Individual VF**: Through a dedicated control panel.

  ![Manual Control](https://github.com/AhmedSayedSaid/Advanced-Multi-Pump-Control-System/blob/main/Advanced%20Multi-Pump%20Control%20System/app-images/manual-control.png)

  
  ![Graph](https://github.com/AhmedSayedSaid/Advanced-Multi-Pump-Control-System/blob/main/Advanced%20Multi-Pump%20Control%20System/app-images/data-Graph.png)


- **System User Access to All System Parameters**:

  ![System Parameters](https://github.com/AhmedSayedSaid/Advanced-Multi-Pump-Control-System/blob/main/Advanced%20Multi-Pump%20Control%20System/app-images/system_parameters.png)

- **Scheduler to Set When the System PID Should Work on a Regular Basis or on Dated Basis**:

  ![Scheduler](https://github.com/AhmedSayedSaid/Advanced-Multi-Pump-Control-System/blob/main/Advanced%20Multi-Pump%20Control%20System/app-images/SCHEDULER.png)

- **Current System Alarm and Warnings and History of Each Event Happened on the System**:

  ![System Alarms](https://github.com/AhmedSayedSaid/Advanced-Multi-Pump-Control-System/blob/main/Advanced%20Multi-Pump%20Control%20System/app-images/system_alarm.png)

- **The Ability to Send Any Custom Order to Any Modbus Slave or Read Data from It**:

  ![Custom Order](https://github.com/AhmedSayedSaid/Advanced-Multi-Pump-Control-System/blob/main/Advanced%20Multi-Pump%20Control%20System/app-images/send_custom_order.png)

### Dedicated Mobile App

- **Android /IOS Mobile Applications for the system **:

  <p float="left">
  <img src="https://github.com/AhmedSayedSaid/Advanced-Multi-Pump-Control-System/blob/main/Advanced%20Multi-Pump%20Control%20System/app-images/system_app.png" width="250" />
  <img src="https://github.com/AhmedSayedSaid/Advanced-Multi-Pump-Control-System/blob/main/Advanced%20Multi-Pump%20Control%20System/app-images/system_app_pic_2.png" width="250" /> 
  <img src="https://github.com/AhmedSayedSaid/Advanced-Multi-Pump-Control-System/blob/main/Advanced%20Multi-Pump%20Control%20System/app-images/system_app_pic_3.png" width="250" />
</p>



### Communication Layer

- **Bluetooth Low Energy (BLE)**: Facilitates wireless connectivity for user commands and data visualization.
- **QR Code Scanning**: Provides a method for device identification and connection establishment using bar code scanning that connect to the board directly through Bluetooth Low Energy.
- **ESP Module**: Acts as the central communication hub, managing access levels and data transmission.

### Control Algorithm

- **STM32 Microcontroller**: Employs a PID algorithm for precise control of pressure levels, alongside a scheduling algorithm to balance pump operation hours. Up to 8 pumps can be managed, dynamically controlling their operation to maintain optimal pressure.

  ![system_Architecture](https://github.com/AhmedSayedSaid/Advanced-Multi-Pump-Control-System/blob/main/Advanced%20Multi-Pump%20Control%20System/app-images/system_Architecture.png)

## Workflow

1. **User Access**: Users initiate the system interaction through the mobile, web, or desktop application.
2. **Authentication**: Users are required to authenticate themselves for secure access.
3. **Access Levels**: General Access, Admin Access, and Factory Level.
4. **Data Transmission**: The ESP module communicates system readings through the STM32 microcontroller.
5. **Control and Efficiency Algorithms**: The STM32 executes PID and scheduling algorithms for optimal operation.
6. **User Interaction**: Depending on access level, users can monitor or control the system.

For more details , see the full documentation which is inside the project folder.
