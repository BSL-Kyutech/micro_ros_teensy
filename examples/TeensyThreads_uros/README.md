# micro-ROS on TeensyThreads for Teensy

A micro-ROS code using multithreading for Teensy 4.1.   
The multithreading function is implemented via "TeensyThreads", a official support library of Teensy, similar to "FreeRTOS".  
Note that, if you use "FreeRTOS", the following issues may occur:
    
1. **Error : Device detection failed.**  
    This issue was found in the situation using a multithreading library of "tsandmann/freertos-teensy".

1. **Error : Running callback function of ROS2 node failed.**  
    This issue was found in the situation using a multithreading library of "juliandesvignes/FreeRTOS-Teensy4".
