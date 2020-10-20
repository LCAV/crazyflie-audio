EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 6
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:GND #PWR0102
U 1 1 5EF4F83A
P 1300 2400
F 0 "#PWR0102" H 1300 2150 50  0001 C CNN
F 1 "GND" H 1305 2227 50  0000 C CNN
F 2 "" H 1300 2400 50  0001 C CNN
F 3 "" H 1300 2400 50  0001 C CNN
	1    1300 2400
	-1   0    0    1   
$EndComp
Text Label 1200 2750 0    50   ~ 0
SWCLK
Text Label 1200 2850 0    50   ~ 0
SWDIO
Text Label 1200 2950 0    50   ~ 0
nRESET
Wire Wire Line
	1200 2950 1450 2950
Wire Wire Line
	1200 2850 1450 2850
Wire Wire Line
	1200 2750 1450 2750
Text Notes 1450 2100 0    50   ~ 0
microcontroller stm32f4\n
Text Label 1200 4350 0    50   ~ 0
RX1
Text Label 1200 4450 0    50   ~ 0
TX1
Text Label 1200 3300 0    50   ~ 0
SDA
Text Label 1200 3400 0    50   ~ 0
SCL
Wire Wire Line
	1450 4350 1200 4350
Wire Wire Line
	1450 4450 1200 4450
Wire Wire Line
	1450 3300 1200 3300
Wire Wire Line
	1200 3400 1450 3400
Text Notes 1700 800  0    50   ~ 0
TC2030 connector for debug
Wire Wire Line
	1800 1450 1950 1450
$Comp
L power:GND #PWR0114
U 1 1 5EFF1397
P 1800 1450
F 0 "#PWR0114" H 1800 1200 50  0001 C CNN
F 1 "GND" H 1805 1277 50  0000 C CNN
F 2 "" H 1800 1450 50  0001 C CNN
F 3 "" H 1800 1450 50  0001 C CNN
	1    1800 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 1050 1800 1050
$Comp
L power:VCC #PWR0113
U 1 1 5EFEDE82
P 1800 1050
F 0 "#PWR0113" H 1800 900 50  0001 C CNN
F 1 "VCC" H 1815 1223 50  0000 C CNN
F 2 "" H 1800 1050 50  0001 C CNN
F 3 "" H 1800 1050 50  0001 C CNN
	1    1800 1050
	1    0    0    -1  
$EndComp
Text Label 1700 1150 0    50   ~ 0
SWDIO
Wire Wire Line
	1700 1350 1950 1350
Wire Wire Line
	1700 1150 1950 1150
Wire Wire Line
	1700 1250 1950 1250
Text Label 1700 1250 0    50   ~ 0
nRESET
Text Label 1700 1350 0    50   ~ 0
SWCLK
$Comp
L audio_shield:SWD_TC2030 J9
U 1 1 5EFDB2F4
P 2000 950
F 0 "J9" H 2478 646 50  0000 L CNN
F 1 "SWD_TC2030" H 2478 555 50  0000 L CNN
F 2 "Connector:Tag-Connect_TC2030-IDC-FP_2x03_P1.27mm_Vertical" H 2000 950 50  0001 C CNN
F 3 "https://www.digikey.ch/htmldatasheets/production/2094258/0/0/1/tc2030-ctx-datasheet.html" H 2000 950 50  0001 C CNN
F 4 "Digikey" H 2000 950 50  0001 C CNN "Supplier"
F 5 "TC2030-CTX-ND " H 2000 950 50  0001 C CNN "Supplier ref."
F 6 "51.01" H 2000 950 50  0001 C CNN "Price"
	1    2000 950 
	1    0    0    -1  
$EndComp
Text Notes 6900 700  0    50   ~ 0
MEMS microphones
Text Label 2350 2350 0    50   ~ 0
I2S1_SD
Text Label 2350 2450 0    50   ~ 0
I2S1_CK
Text Label 2350 3400 0    50   ~ 0
I2S3_WS
Text Label 2350 3300 0    50   ~ 0
I2S3_CK
Text Label 2350 3200 0    50   ~ 0
I2S3_SD
Text Label 2350 2550 0    50   ~ 0
I2S1_WS
Text Label 5750 3550 0    50   ~ 0
I2S1_SD
Text Label 5750 3650 0    50   ~ 0
I2S1_CK
Text Label 5750 3750 0    50   ~ 0
I2S1_WS
Wire Wire Line
	5750 3550 6150 3550
Text Label 5800 4700 0    50   ~ 0
I2S1_CK
Text Label 5800 4800 0    50   ~ 0
I2S1_WS
Text Label 5800 1400 0    50   ~ 0
I2S3_SD
Text Label 5800 1500 0    50   ~ 0
I2S3_CK
Text Label 5800 1600 0    50   ~ 0
I2S3_WS
Text Label 5800 2450 0    50   ~ 0
I2S3_SD
Text Label 5800 2550 0    50   ~ 0
I2S3_CK
Text Label 5800 2650 0    50   ~ 0
I2S3_WS
Wire Wire Line
	5750 3750 6150 3750
Wire Wire Line
	5750 3650 6150 3650
Wire Wire Line
	5800 4600 6150 4600
Wire Wire Line
	5800 4700 6150 4700
Wire Wire Line
	5800 4800 6150 4800
Wire Wire Line
	5800 1400 6150 1400
Wire Wire Line
	5800 1500 6150 1500
Wire Wire Line
	5800 1600 6150 1600
Wire Wire Line
	5800 2450 6150 2450
Wire Wire Line
	5800 2550 6150 2550
Wire Wire Line
	5800 2650 6150 2650
Wire Wire Line
	6050 1050 6050 950 
Wire Wire Line
	6150 1050 6050 1050
$Comp
L power:GND #PWR05
U 1 1 5F0EF132
P 6050 950
F 0 "#PWR05" H 6050 700 50  0001 C CNN
F 1 "GND" H 6055 777 50  0000 C CNN
F 2 "" H 6050 950 50  0001 C CNN
F 3 "" H 6050 950 50  0001 C CNN
	1    6050 950 
	-1   0    0    1   
$EndComp
Wire Wire Line
	5900 1150 5900 1100
$Comp
L power:VCC #PWR01
U 1 1 5F0EF139
P 5900 1100
F 0 "#PWR01" H 5900 950 50  0001 C CNN
F 1 "VCC" H 5915 1273 50  0000 C CNN
F 2 "" H 5900 1100 50  0001 C CNN
F 3 "" H 5900 1100 50  0001 C CNN
	1    5900 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 1150 6150 1150
Wire Wire Line
	6050 2100 6050 2000
Wire Wire Line
	6150 2100 6050 2100
$Comp
L power:GND #PWR06
U 1 1 5F0F920D
P 6050 2000
F 0 "#PWR06" H 6050 1750 50  0001 C CNN
F 1 "GND" H 6055 1827 50  0000 C CNN
F 2 "" H 6050 2000 50  0001 C CNN
F 3 "" H 6050 2000 50  0001 C CNN
	1    6050 2000
	-1   0    0    1   
$EndComp
Wire Wire Line
	5900 2200 5900 2150
$Comp
L power:VCC #PWR02
U 1 1 5F0F9214
P 5900 2150
F 0 "#PWR02" H 5900 2000 50  0001 C CNN
F 1 "VCC" H 5915 2323 50  0000 C CNN
F 2 "" H 5900 2150 50  0001 C CNN
F 3 "" H 5900 2150 50  0001 C CNN
	1    5900 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 2200 6150 2200
Wire Wire Line
	6050 3200 6050 3100
Wire Wire Line
	6150 3200 6050 3200
$Comp
L power:GND #PWR07
U 1 1 5F0FCAC9
P 6050 3100
F 0 "#PWR07" H 6050 2850 50  0001 C CNN
F 1 "GND" H 6055 2927 50  0000 C CNN
F 2 "" H 6050 3100 50  0001 C CNN
F 3 "" H 6050 3100 50  0001 C CNN
	1    6050 3100
	-1   0    0    1   
$EndComp
Wire Wire Line
	5900 3300 5900 3250
$Comp
L power:VCC #PWR03
U 1 1 5F0FCAD0
P 5900 3250
F 0 "#PWR03" H 5900 3100 50  0001 C CNN
F 1 "VCC" H 5915 3423 50  0000 C CNN
F 2 "" H 5900 3250 50  0001 C CNN
F 3 "" H 5900 3250 50  0001 C CNN
	1    5900 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 3300 6150 3300
Wire Wire Line
	6050 4250 6050 4150
Wire Wire Line
	6150 4250 6050 4250
$Comp
L power:GND #PWR08
U 1 1 5F1006FE
P 6050 4150
F 0 "#PWR08" H 6050 3900 50  0001 C CNN
F 1 "GND" H 6055 3977 50  0000 C CNN
F 2 "" H 6050 4150 50  0001 C CNN
F 3 "" H 6050 4150 50  0001 C CNN
	1    6050 4150
	-1   0    0    1   
$EndComp
Wire Wire Line
	5900 4350 5900 4300
$Comp
L power:VCC #PWR04
U 1 1 5F100705
P 5900 4300
F 0 "#PWR04" H 5900 4150 50  0001 C CNN
F 1 "VCC" H 5915 4473 50  0000 C CNN
F 2 "" H 5900 4300 50  0001 C CNN
F 3 "" H 5900 4300 50  0001 C CNN
	1    5900 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 4350 6150 4350
Wire Wire Line
	6150 1250 5900 1250
Wire Wire Line
	5900 1250 5900 1150
Connection ~ 5900 1150
Wire Wire Line
	6150 3400 5900 3400
Wire Wire Line
	5900 3400 5900 3300
Connection ~ 5900 3300
Wire Wire Line
	6150 4450 6050 4450
Wire Wire Line
	6050 4450 6050 4250
Connection ~ 6050 4250
Wire Wire Line
	2350 3200 2250 3200
Wire Wire Line
	2350 3300 2250 3300
Wire Wire Line
	2350 3400 2250 3400
Wire Wire Line
	2350 2550 2250 2550
Wire Wire Line
	2350 2450 2250 2450
Wire Wire Line
	2350 2350 2250 2350
$Sheet
S 1450 2250 800  2300
U 5EF054A6
F0 "stm32f446re" 50
F1 "stm32f446re.sch" 50
F2 "3.0V" I L 1450 2500 50 
F3 "I2S1_WS" I R 2250 2550 50 
F4 "I2S1_SD" I R 2250 2350 50 
F5 "I2S3_SD" I R 2250 3200 50 
F6 "I2S3_WS" I R 2250 3400 50 
F7 "I2S3_CK" I R 2250 3300 50 
F8 "I2S1_CK" I R 2250 2450 50 
F9 "GND" I L 1450 2400 50 
F10 "nRESET" I L 1450 2950 50 
F11 "SWDIO" I L 1450 2850 50 
F12 "SWCLK" I L 1450 2750 50 
F13 "UART4_TX" I L 1450 4350 50 
F14 "UART4_RX" I L 1450 4450 50 
F15 "I2C1_SDA" I L 1450 3300 50 
F16 "I2C1_SCL" I L 1450 3400 50 
F17 "SPI2_CK" I R 2250 3900 50 
F18 "SPI2_NSS" I R 2250 4000 50 
F19 "SPI2_MOSI" I R 2250 3800 50 
F20 "SPI2_MISO" I R 2250 3700 50 
F21 "PC3" I R 2250 4200 50 
F22 "PA2" I R 2250 4300 50 
$EndSheet
Text Label 2350 4000 0    50   ~ 0
SPI2_NSS
Text Label 2350 3900 0    50   ~ 0
SPI2_CK
Text Label 2350 3800 0    50   ~ 0
SPI2_MOSI
Wire Wire Line
	2250 3800 2350 3800
Wire Wire Line
	2250 3900 2350 3900
Wire Wire Line
	2250 4000 2350 4000
Text Label 2350 3700 0    50   ~ 0
SPI2_MISO
Wire Wire Line
	2350 3700 2250 3700
$Comp
L power:VCC #PWR0103
U 1 1 5EF504C5
P 1100 2500
F 0 "#PWR0103" H 1100 2350 50  0001 C CNN
F 1 "VCC" H 1115 2673 50  0000 C CNN
F 2 "" H 1100 2500 50  0001 C CNN
F 3 "" H 1100 2500 50  0001 C CNN
	1    1100 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 2500 1450 2500
Wire Wire Line
	1300 2400 1450 2400
Text Label 950  1100 0    50   ~ 0
GND
$Comp
L power:VCC #PWR0123
U 1 1 5F338EA0
P 800 850
F 0 "#PWR0123" H 800 700 50  0001 C CNN
F 1 "VCC" H 815 1023 50  0000 C CNN
F 2 "" H 800 850 50  0001 C CNN
F 3 "" H 800 850 50  0001 C CNN
	1    800  850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  850  1150 850 
$Comp
L power:GND #PWR0127
U 1 1 5F345DAF
P 800 1100
F 0 "#PWR0127" H 800 850 50  0001 C CNN
F 1 "GND" H 805 927 50  0000 C CNN
F 2 "" H 800 1100 50  0001 C CNN
F 3 "" H 800 1100 50  0001 C CNN
	1    800  1100
	-1   0    0    1   
$EndComp
Wire Wire Line
	800  1100 950  1100
$Comp
L power:+3V0 #PWR0128
U 1 1 5F352F66
P 1150 850
F 0 "#PWR0128" H 1150 700 50  0001 C CNN
F 1 "+3V0" H 1165 1023 50  0000 C CNN
F 2 "" H 1150 850 50  0001 C CNN
F 3 "" H 1150 850 50  0001 C CNN
	1    1150 850 
	1    0    0    -1  
$EndComp
Text Label 9650 2450 0    50   ~ 0
SPI2_MISO
$Sheet
S 6150 4150 550  750 
U 5F096C14
F0 "sheet5F096C0C" 50
F1 "SPH0645LM4H-B .sch" 50
F2 "DATA_OUT" I L 6150 4600 50 
F3 "GND" I L 6150 4250 50 
F4 "VDD" I L 6150 4350 50 
F5 "WS" I L 6150 4800 50 
F6 "BCLK" I L 6150 4700 50 
F7 "SELECT" I L 6150 4450 50 
$EndSheet
$Sheet
S 6150 3100 550  750 
U 5F0950A2
F0 "sheet5F09509A" 50
F1 "SPH0645LM4H-B .sch" 50
F2 "DATA_OUT" I L 6150 3550 50 
F3 "GND" I L 6150 3200 50 
F4 "VDD" I L 6150 3300 50 
F5 "WS" I L 6150 3750 50 
F6 "BCLK" I L 6150 3650 50 
F7 "SELECT" I L 6150 3400 50 
$EndSheet
$Sheet
S 6150 2000 550  750 
U 5F093705
F0 "sheet5F0936FD" 50
F1 "SPH0645LM4H-B .sch" 50
F2 "DATA_OUT" I L 6150 2450 50 
F3 "GND" I L 6150 2100 50 
F4 "VDD" I L 6150 2200 50 
F5 "WS" I L 6150 2650 50 
F6 "BCLK" I L 6150 2550 50 
F7 "SELECT" I L 6150 2300 50 
$EndSheet
$Sheet
S 6150 950  550  750 
U 5F04603F
F0 "sheet5F046037" 50
F1 "SPH0645LM4H-B .sch" 50
F2 "DATA_OUT" I L 6150 1400 50 
F3 "GND" I L 6150 1050 50 
F4 "VDD" I L 6150 1150 50 
F5 "WS" I L 6150 1600 50 
F6 "BCLK" I L 6150 1500 50 
F7 "SELECT" I L 6150 1250 50 
$EndSheet
Text Label 9650 2350 0    50   ~ 0
SPI2_CK
Text Label 9650 2550 0    50   ~ 0
SPI2_MOSI
Text Label 9650 1850 0    50   ~ 0
SPI2_NSS
Wire Wire Line
	9650 2450 10150 2450
$Comp
L Connector_Generic:Conn_01x10 J2
U 1 1 5EE8979D
P 10350 2550
F 0 "J2" H 10430 2542 50  0000 L CNN
F 1 "CONN_10_RIGHT" H 10430 2451 50  0000 L CNN
F 2 "audio_shield:BF090-10-X-B1" H 10350 2550 50  0001 C CNN
F 3 "http://www.farnell.com/datasheets/2342918.pdf?_ga=2.210943958.293929472.1593689991-1142544513.1588679285" H 10350 2550 50  0001 C CNN
F 4 "Farnell" H 10350 2550 50  0001 C CNN "Supplier"
F 5 "2751429 " H 10350 2550 50  0001 C CNN "Supplier ref."
F 6 "1.46" H 10350 2550 50  0001 C CNN "Price"
	1    10350 2550
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x10 J1
U 1 1 5EE88981
P 10350 1450
F 0 "J1" H 10430 1442 50  0000 L CNN
F 1 "CONN_10_LEFT" H 10430 1351 50  0000 L CNN
F 2 "audio_shield:BF090-10-X-B1" H 10350 1450 50  0001 C CNN
F 3 "http://www.farnell.com/datasheets/2342918.pdf?_ga=2.210943958.293929472.1593689991-1142544513.1588679285" H 10350 1450 50  0001 C CNN
F 4 "Farnell" H 10350 1450 50  0001 C CNN "Supplier"
F 5 "2751429 " H 10350 1450 50  0001 C CNN "Supplier ref."
F 6 "1.46" H 10350 1450 50  0001 C CNN "Price"
	1    10350 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5EE8E607
P 10050 1950
F 0 "#PWR0101" H 10050 1700 50  0001 C CNN
F 1 "GND" H 10055 1777 50  0000 C CNN
F 2 "" H 10050 1950 50  0001 C CNN
F 3 "" H 10050 1950 50  0001 C CNN
	1    10050 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 1950 10150 1950
$Comp
L power:VCC #PWR0108
U 1 1 5EF6D11E
P 10000 1050
F 0 "#PWR0108" H 10000 900 50  0001 C CNN
F 1 "VCC" H 10015 1223 50  0000 C CNN
F 2 "" H 10000 1050 50  0001 C CNN
F 3 "" H 10000 1050 50  0001 C CNN
	1    10000 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 1050 10000 1050
Text Label 9900 1150 0    50   ~ 0
RX1
Text Label 9900 1250 0    50   ~ 0
TX1
Wire Wire Line
	10150 1150 9900 1150
Wire Wire Line
	10150 1250 9900 1250
Text Notes 9800 700  0    50   ~ 0
Crazyflie connectors\n
Text Label 5800 4600 0    50   ~ 0
I2S1_SD
Wire Wire Line
	6150 2300 6050 2300
Wire Wire Line
	6050 2300 6050 2100
Connection ~ 6050 2100
Text Label 8900 4000 0    50   ~ 0
I2S3_SD
Text Label 8900 4100 0    50   ~ 0
I2S3_CK
Text Label 8900 4200 0    50   ~ 0
I2S3_WS
$Comp
L Connector:TestPoint I2S3_CK1
U 1 1 5F0F583E
P 10050 3400
F 0 "I2S3_CK1" H 10108 3518 50  0000 L CNN
F 1 "TestPoint" H 10108 3427 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 10250 3400 50  0001 C CNN
F 3 "~" H 10250 3400 50  0001 C CNN
F 4 "np" H 10050 3400 50  0001 C CNN "Supplier"
	1    10050 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9550 3450 9550 3400
Wire Wire Line
	8900 4000 9550 4000
Wire Wire Line
	10050 3550 10050 3400
Wire Wire Line
	8900 4100 10050 4100
Wire Wire Line
	8900 4200 10550 4200
Text Label 8900 3450 0    50   ~ 0
I2S1_SD
Text Label 8900 3550 0    50   ~ 0
I2S1_CK
Text Label 8900 3650 0    50   ~ 0
I2S1_WS
Wire Wire Line
	10550 3650 10550 3400
$Comp
L Connector:TestPoint I2S1_SD1
U 1 1 5F155001
P 9550 3950
F 0 "I2S1_SD1" H 9608 4068 50  0000 L CNN
F 1 "TestPoint" H 9608 3977 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 9750 3950 50  0001 C CNN
F 3 "~" H 9750 3950 50  0001 C CNN
F 4 "np" H 9550 3950 50  0001 C CNN "Supplier"
	1    9550 3950
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint I2S1_CK1
U 1 1 5F155007
P 10050 3950
F 0 "I2S1_CK1" H 10108 4068 50  0000 L CNN
F 1 "TestPoint" H 10108 3977 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 10250 3950 50  0001 C CNN
F 3 "~" H 10250 3950 50  0001 C CNN
F 4 "np" H 10050 3950 50  0001 C CNN "Supplier"
	1    10050 3950
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint I2S1_WS1
U 1 1 5F15500D
P 10550 3950
F 0 "I2S1_WS1" H 10608 4068 50  0000 L CNN
F 1 "TestPoint" H 10608 3977 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 10750 3950 50  0001 C CNN
F 3 "~" H 10750 3950 50  0001 C CNN
F 4 "np" H 10550 3950 50  0001 C CNN "Supplier"
	1    10550 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9550 4000 9550 3950
Wire Wire Line
	10050 4100 10050 3950
Wire Wire Line
	10550 4200 10550 3950
Wire Wire Line
	8900 3650 10550 3650
Wire Wire Line
	8900 3550 10050 3550
Wire Wire Line
	8900 3450 9550 3450
Text Label 8900 4500 0    50   ~ 0
RX1
Text Label 8900 4600 0    50   ~ 0
TX1
$Comp
L Connector:TestPoint RX1
U 1 1 5F16A53B
P 9250 4450
F 0 "RX1" H 9308 4568 50  0000 L CNN
F 1 "TestPoint" H 9308 4477 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 9450 4450 50  0001 C CNN
F 3 "~" H 9450 4450 50  0001 C CNN
F 4 "np" H 9250 4450 50  0001 C CNN "Supplier"
	1    9250 4450
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TX1
U 1 1 5F16A541
P 9750 4450
F 0 "TX1" H 9808 4568 50  0000 L CNN
F 1 "TestPoint" H 9808 4477 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 9950 4450 50  0001 C CNN
F 3 "~" H 9950 4450 50  0001 C CNN
F 4 "np" H 9750 4450 50  0001 C CNN "Supplier"
	1    9750 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 4500 9250 4450
Wire Wire Line
	9750 4600 9750 4450
Wire Wire Line
	8900 4500 9250 4500
Wire Wire Line
	8900 4600 9750 4600
Wire Wire Line
	9050 5200 9050 5100
$Comp
L power:GND #PWR022
U 1 1 5F1C6E4D
P 9050 5100
F 0 "#PWR022" H 9050 4850 50  0001 C CNN
F 1 "GND" H 9055 4927 50  0000 C CNN
F 2 "" H 9050 5100 50  0001 C CNN
F 3 "" H 9050 5100 50  0001 C CNN
	1    9050 5100
	-1   0    0    1   
$EndComp
$Comp
L power:VCC #PWR021
U 1 1 5F1C6E54
P 8900 5700
F 0 "#PWR021" H 8900 5550 50  0001 C CNN
F 1 "VCC" H 8915 5873 50  0000 C CNN
F 2 "" H 8900 5700 50  0001 C CNN
F 3 "" H 8900 5700 50  0001 C CNN
	1    8900 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 5750 9150 5750
$Comp
L Connector:TestPoint I2S3_SD1
U 1 1 5F0E4486
P 9550 3400
F 0 "I2S3_SD1" H 9608 3518 50  0000 L CNN
F 1 "TestPoint" H 9608 3427 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 9750 3400 50  0001 C CNN
F 3 "~" H 9750 3400 50  0001 C CNN
F 4 "np" H 9550 3400 50  0001 C CNN "Supplier"
	1    9550 3400
	1    0    0    -1  
$EndComp
Text Label 8800 6250 0    50   ~ 0
SPI2_CK
Text Label 8800 6350 0    50   ~ 0
SPI2_MOSI
Text Label 8800 6150 0    50   ~ 0
SPI2_MISO
$Comp
L Connector:TestPoint SPI2_MISO1
U 1 1 5F27A2F0
P 9650 6000
F 0 "SPI2_MISO1" H 9708 6118 50  0000 L CNN
F 1 "TestPoint" H 9708 6027 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 9850 6000 50  0001 C CNN
F 3 "~" H 9850 6000 50  0001 C CNN
F 4 "np" H 9650 6000 50  0001 C CNN "Supplier"
	1    9650 6000
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint SPI2_CK1
U 1 1 5F27A2F6
P 10150 6000
F 0 "SPI2_CK1" H 10208 6118 50  0000 L CNN
F 1 "TestPoint" H 10208 6027 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 10350 6000 50  0001 C CNN
F 3 "~" H 10350 6000 50  0001 C CNN
F 4 "np" H 10150 6000 50  0001 C CNN "Supplier"
	1    10150 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 6150 9650 6000
Wire Wire Line
	10150 6250 10150 6000
$Comp
L Connector:TestPoint SPI2_MOSI1
U 1 1 5F27A2FF
P 10600 6000
F 0 "SPI2_MOSI1" H 10658 6118 50  0000 L CNN
F 1 "TestPoint" H 10658 6027 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 10800 6000 50  0001 C CNN
F 3 "~" H 10800 6000 50  0001 C CNN
F 4 "np" H 10600 6000 50  0001 C CNN "Supplier"
	1    10600 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 6150 9650 6150
Wire Wire Line
	8800 6250 10150 6250
Wire Wire Line
	8800 6350 10600 6350
Wire Wire Line
	10600 6000 10600 6350
Wire Wire Line
	8900 5700 8900 5750
Text Label 2350 4200 0    50   ~ 0
PC3
Text Label 2350 4300 0    50   ~ 0
PA2
Wire Wire Line
	2250 4300 2350 4300
Wire Wire Line
	2250 4200 2350 4200
Text Label 7450 6250 0    50   ~ 0
PC3
Text Label 7450 6350 0    50   ~ 0
PA2
$Comp
L Connector:TestPoint PC3
U 1 1 5F2D740A
P 7750 6200
F 0 "PC3" H 7808 6318 50  0000 L CNN
F 1 "TestPoint" H 7808 6227 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 7950 6200 50  0001 C CNN
F 3 "~" H 7950 6200 50  0001 C CNN
F 4 "np" H 7750 6200 50  0001 C CNN "Supplier"
	1    7750 6200
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint PA2
U 1 1 5F2D7410
P 8250 6200
F 0 "PA2" H 8308 6318 50  0000 L CNN
F 1 "TestPoint" H 8308 6227 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 8450 6200 50  0001 C CNN
F 3 "~" H 8450 6200 50  0001 C CNN
F 4 "np" H 8250 6200 50  0001 C CNN "Supplier"
	1    8250 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 6250 7750 6200
Wire Wire Line
	8250 6350 8250 6200
Wire Wire Line
	7450 6250 7750 6250
Wire Wire Line
	7450 6350 8250 6350
$Comp
L Connector:TestPoint I2S3_WS1
U 1 1 5F0FDA51
P 10550 3400
F 0 "I2S3_WS1" H 10608 3518 50  0000 L CNN
F 1 "TestPoint" H 10608 3427 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 10750 3400 50  0001 C CNN
F 3 "~" H 10750 3400 50  0001 C CNN
F 4 "np" H 10550 3400 50  0001 C CNN "Supplier"
	1    10550 3400
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint GND2
U 1 1 5F306697
P 9150 5150
F 0 "GND2" H 9208 5268 50  0000 L CNN
F 1 "TestPoint" H 9208 5177 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 9350 5150 50  0001 C CNN
F 3 "~" H 9350 5150 50  0001 C CNN
F 4 "np" H 9150 5150 50  0001 C CNN "Supplier"
	1    9150 5150
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint 3V0_2
U 1 1 5F37B87C
P 9150 5600
F 0 "3V0_2" H 9208 5718 50  0000 L CNN
F 1 "TestPoint" H 9208 5627 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 9350 5600 50  0001 C CNN
F 3 "~" H 9350 5600 50  0001 C CNN
F 4 "np" H 9150 5600 50  0001 C CNN "Supplier"
	1    9150 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 5600 9150 5750
Wire Wire Line
	9050 5200 9150 5200
Wire Wire Line
	9150 5200 9150 5150
$Comp
L Connector:AVR-JTAG-10 J48
U 1 1 5F223B53
P -1450 1450
F 0 "J48" H -1879 1496 50  0000 R CNN
F 1 "STLINK-V3MINI Placeholder" H -1879 1405 50  0000 R CNN
F 2 "" V -1600 1600 50  0001 C CNN
F 3 "https://www.digikey.ch/product-detail/fr/STLINK-V3MINI/497-19530-ND/10266291/?itemSeq=333705927 ~" H -2725 900 50  0001 C CNN
F 4 "np" H -1450 1450 50  0001 C CNN "Supplier"
	1    -1450 1450
	1    0    0    -1  
$EndComp
$Comp
L audio_shield:DS28E05P U2
U 1 1 5F3EF92F
P 1600 6750
F 0 "U2" H 1600 6915 50  0000 C CNN
F 1 "DS28E05P" H 1600 6824 50  0000 C CNN
F 2 "Package_SO_J-Lead:TSOC-6_3.76x3.94mm_P1.27mm" H 1600 6800 50  0001 C CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/DS28E05.pdf" H 1600 6800 50  0001 C CNN
F 4 "Mouser" H 1600 6750 50  0001 C CNN "Supplier"
F 5 "700-DS28E05P+" H 1600 6750 50  0001 C CNN "Supplier ref."
F 6 " 0.73" H 1600 6750 50  0001 C CNN "Price"
	1    1600 6750
	1    0    0    -1  
$EndComp
Text Notes 1150 6500 0    50   ~ 0
EEPROM for deck identification
$Comp
L power:GND #PWR0129
U 1 1 5F4170B9
P 1200 6850
F 0 "#PWR0129" H 1200 6600 50  0001 C CNN
F 1 "GND" H 1205 6677 50  0000 C CNN
F 2 "" H 1200 6850 50  0001 C CNN
F 3 "" H 1200 6850 50  0001 C CNN
	1    1200 6850
	-1   0    0    1   
$EndComp
Wire Wire Line
	1200 6850 1300 6850
Text Label 9750 2850 0    50   ~ 0
EEPROM
Wire Wire Line
	9750 2850 10150 2850
Text Label 950  6950 0    50   ~ 0
EEPROM
Wire Wire Line
	950  6950 1300 6950
Text Notes 700  3200 0    50   ~ 0
I2C adress 47\n
Wire Wire Line
	9650 2350 10150 2350
Wire Wire Line
	9650 2550 10150 2550
Wire Wire Line
	9650 1850 10150 1850
$Comp
L Connector:TestPoint EE1
U 1 1 5F815185
P 10250 4450
F 0 "EE1" H 10308 4568 50  0000 L CNN
F 1 "TestPoint" H 10308 4477 50  0000 L CNN
F 2 "audio_shield:TestPoint_Loop_D2.50mm_Drill1.0mm_LowProfile" H 10450 4450 50  0001 C CNN
F 3 "~" H 10450 4450 50  0001 C CNN
F 4 "np" H 10250 4450 50  0001 C CNN "Supplier"
	1    10250 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 4700 10250 4450
Wire Wire Line
	8900 4700 10250 4700
Text Label 8900 4700 0    50   ~ 0
EEPROM
$EndSCHEMATC
