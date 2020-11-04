# CarettaBMS

CarettaBMS is the best price to performance battery management system and it's also open-source.

# Overview

In recent years, there has been an increase in the use of ultralight electric vehicles. A fairly new segment requires the use of smaller battery packs for energy storage to power vehicles. For safe use of battery packs, BMS is very important. There is a big shortage in the field of quality battery management systems, which would be suitable for smaller battery packs. Prices of the systems currently on the market are too high. The price of a BMS can exceed the price of a battery pack. The use of such BMS does not make sense. The most difficult task was to make the system as inexpensive as possible for use with smaller battery packs. Despite the low cost, BMS still has all the necessary safety mechanisms.

By approaching the challenges in a creative way it was possible to find innovative solutions, which enabled a very low-cost implementation of the system. Analog to digital converter (ADC) is used with swapped inputs reference and ADC input. This makes it possible to measure the supply voltage without the use a of voltage divider and additional static energy consumption a voltage divider would entail. The type A standard uncertainty of cell voltage measurement is 0.9mV. The communication uses a built-in analog comparator and two external resistors. Despite the fact that series connected modules operate on different reference potentials, this does not affect the message transmission. Tests with artificially induced interfearence confirmed the communication reliability of an arbitrary number of modules connected in series.

At the time of this writing the production price for 1000 pieces is estimated at 1.37EUR per piece, which would allow the sell price between 6 and 10EUR. This is at least three times better than the current cheapest commercial offer.

The work was released as an open source project with the name CarettaBMS. The purpose is to allow access to better quality BMS suitable for smaller battery packs to a wider audience.

# System Arhitecure

![enter image description here](https://lh3.googleusercontent.com/pw/ACtC-3ddnmIlgAX7mJDV0arrn4f1sXazZcy3e4E-uQQENZFuSyTD-T8r-spXYUoo_znrnw3XfoDmRY36TqDYfhk3SqiQ9UDXjDD9U14ocJviiAtjifs4hR8bXS-7GtUJVUYnRgnQNe-SVxnXcvCURDpLlTw1Hg=w1280-h797-no?authuser=0)

# specifications

Operating voltage range: od 1.8V do 5.0V\
Operating temperture range: od 0°C do 80°C\
Voltage measurement accuracy: ±10mV\
Voltage measurement resolution: at least 3mV\
Temperature measurement accuracy: ±5°C\
PCB size: 65mm × 18mm × 5mm\
Pasive cell balancing current: 1A\
load characteristics: R = 3.6Ω, P~max~ = 4W\
Maximum of series connected cells: up to 256\
Temperature sensors: at least 3 sensors per module (1 internal, 2 external) or up to 32 I2C sensors\
Comunication intergace (cell modules with master): isolated UART 8E2 (documented protocol)\
Bootloader for future infield firmware upgrades\

# Cell Module
![enter image description here](https://lh3.googleusercontent.com/pw/ACtC-3c2fUZxI_gzC2w3IJYHB17Ieo5sSghVB1DCFiqprrKGjAsjlA3z2c1yILERjq1eG8U84jEvjWz5RBbkoycMlxdKpXT9QEH2yXEYr3CSb4LkigQ8I1HedWclGtvBs_9rG6EiEaH0F6gFOdXFeXLpDZOJ3Q=w245-h881-no?authuser=0)

Schematic: https://github.com/Hrastovc/CarettaBMS/blob/master/CellModule/Hardware/CarettaBMS_CellModule0v8.pdf

# License

CarettaBMS is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License version 3 as published by the Free Software Foundation.
