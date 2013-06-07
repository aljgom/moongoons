REM "C:/Program Files (x86)/CodeSourcery/Sourcery G++ Lite/bin/arm-none-linux-gnueabi-g++.exe" -lpthread -o ../bin/motorboard motorboard.c mot.c ../gpio/gpio.c ../util/util.c main_motorboard.c
REM "C:/Program Files (x86)/CodeSourcery/Sourcery G++ Lite/bin/arm-none-linux-gnueabi-g++.exe" -lpthread -o ../bin/sam_motorboard motorboard.c mot.c ../gpio/gpio.c ../util/util.c sam_motorboard.c

arm-none-linux-gnueabi-g++.exe -lpthread -o compiled/sam_motorboard motorboard.c mot.c ../gpio/gpio.c ../util/util.c ../udp/udp.c sam_motorboard.c
