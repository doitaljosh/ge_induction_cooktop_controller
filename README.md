Arduino-based GE Induction Cooktop Controller

The Arduino code in this repository is designed to send commands to GE's induction cooktop control boards, and attempts to act as a "replacement" for the control panel and/or machine control.
It's still very much a work-in-progress, so expect it to behave inconsistently or not even work at all. This is just a fun little project that illustrates my efforts in reverse engineering GE appliances.


###Design tips:
- Use pull-down resistors and decoupling capacitors on the analog pot inputs to prevent erratic behavior.
- Tie the personality pin high if you are controlling a 36 inch cooktop with 5 coils and 3 generator boards.
- To connect to the single-wire half-duplex serial bus the generator boards use, connect a bus transceiver. Refer to the following for design tips: https://github.com/wfang2002/Full-Half-Duplex-Adapter

##Note: you MUST use an Arduino-compatible board that supports at least two hardware serial busses, or change Serial or Serial2 to use SoftwareSerial.
