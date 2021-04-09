EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 7 7
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
L Device:R R1
U 1 1 5EF30F53
P 4450 2250
F 0 "R1" H 4520 2296 50  0000 L CNN
F 1 "10k" H 4520 2205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4380 2250 50  0001 C CNN
F 3 "https://www.distrelec.ch/fr/resistance-couche-epaisse-resistance-cms-0603-10kohm-100mw-rnd-components-rnd-1550603saj0103t5e/p/30056687?track=true&no-cache=true" H 4450 2250 50  0001 C CNN
F 4 "Distrelec" H 4450 2250 50  0001 C CNN "Supplier"
F 5 "300-56-687" H 4450 2250 50  0001 C CNN "Supplier ref."
F 6 "0.0084" H 4450 2250 50  0001 C CNN "Price"
F 7 "Thick film resistor 0.1W 1%" H 4450 2250 50  0001 C CNN "Description"
	1    4450 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5EF385B8
P 6600 1150
F 0 "C1" V 6852 1150 50  0000 C CNN
F 1 "100n" V 6761 1150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6638 1000 50  0001 C CNN
F 3 "~" H 6600 1150 50  0001 C CNN
F 4 "Distrelec" H 6600 1150 50  0001 C CNN "Supplier"
F 5 "300-86-429" H 6600 1150 50  0001 C CNN "Supplier ref."
F 6 "0.0093" H 6600 1150 50  0001 C CNN "Price"
	1    6600 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 5EF3876A
P 6100 1000
F 0 "L1" V 5919 1000 50  0000 C CNN
F 1 "2.2uH/15mA" V 6010 1000 50  0000 C CNN
F 2 "Inductor_SMD:L_0603_1608Metric" H 6100 1000 50  0001 C CNN
F 3 "https://www.distrelec.ch/fr/inductance-cms-2uh-10-prefix-prefix-15-suffix-suffix-ma-kemet-l0603c2r2ksmst/p/30143947?track=true&no-cache=true" H 6100 1000 50  0001 C CNN
F 4 "Distrelec" V 6100 1000 50  0001 C CNN "Supplier"
F 5 "301-43-947" V 6100 1000 50  0001 C CNN "Supplier ref."
F 6 "0.3708" V 6100 1000 50  0001 C CNN "Price"
	1    6100 1000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5EF3F301
P 6850 1400
F 0 "#PWR0119" H 6850 1150 50  0001 C CNN
F 1 "GND" V 6855 1272 50  0000 R CNN
F 2 "" H 6850 1400 50  0001 C CNN
F 3 "" H 6850 1400 50  0001 C CNN
	1    6850 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 1550 5400 1200
Wire Wire Line
	5400 1200 5500 1200
Wire Wire Line
	5500 1200 5500 1550
Wire Wire Line
	5600 1550 5600 1200
Wire Wire Line
	5600 1200 5500 1200
Connection ~ 5500 1200
Wire Wire Line
	5700 1550 5700 1200
Connection ~ 5600 1200
Wire Wire Line
	5800 1550 5800 1200
Wire Wire Line
	5800 1200 5700 1200
Connection ~ 5700 1200
Wire Wire Line
	5900 1550 5900 1100
Wire Wire Line
	5900 1100 6400 1100
Wire Wire Line
	6400 1100 6400 1000
Wire Wire Line
	6400 1000 6600 1000
Wire Wire Line
	6250 1000 6400 1000
Connection ~ 6400 1000
Wire Wire Line
	6850 1300 6850 1400
Wire Wire Line
	5950 1000 5800 1000
Wire Wire Line
	5800 1000 5800 1200
Connection ~ 5800 1200
Wire Wire Line
	3000 6700 3400 6700
Connection ~ 3400 6700
$Comp
L power:+3V0 #PWR0120
U 1 1 5EF44037
P 3800 6200
F 0 "#PWR0120" H 3800 6050 50  0001 C CNN
F 1 "+3V0" H 3815 6373 50  0000 C CNN
F 2 "" H 3800 6200 50  0001 C CNN
F 3 "" H 3800 6200 50  0001 C CNN
	1    3800 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 6200 3800 6300
Wire Wire Line
	3800 6850 3800 6700
Connection ~ 3800 6700
Wire Wire Line
	3800 6700 4200 6700
Wire Wire Line
	3800 6300 4200 6300
Connection ~ 3800 6300
Connection ~ 3400 6300
Wire Wire Line
	3000 6300 3400 6300
Wire Wire Line
	4200 6600 4200 6700
Wire Wire Line
	3800 6600 3800 6700
Wire Wire Line
	3400 6600 3400 6700
Wire Wire Line
	3000 6600 3000 6700
$Comp
L Device:C C8
U 1 1 5EF424AE
P 4200 6450
F 0 "C8" V 4452 6450 50  0000 C CNN
F 1 "100n" V 4361 6450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4238 6300 50  0001 C CNN
F 3 "~" H 4200 6450 50  0001 C CNN
F 4 "Distrelec" H 4200 6450 50  0001 C CNN "Supplier"
F 5 "300-86-429" H 4200 6450 50  0001 C CNN "Supplier ref."
F 6 "0.0093" H 4200 6450 50  0001 C CNN "Price"
	1    4200 6450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5EF41FC2
P 3800 6450
F 0 "C7" V 4052 6450 50  0000 C CNN
F 1 "100n" V 3961 6450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3838 6300 50  0001 C CNN
F 3 "~" H 3800 6450 50  0001 C CNN
F 4 "Distrelec" H 3800 6450 50  0001 C CNN "Supplier"
F 5 "300-86-429" H 3800 6450 50  0001 C CNN "Supplier ref."
F 6 "0.0093" H 3800 6450 50  0001 C CNN "Price"
	1    3800 6450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5EF41CB9
P 3400 6450
F 0 "C6" V 3652 6450 50  0000 C CNN
F 1 "100n" V 3561 6450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3438 6300 50  0001 C CNN
F 3 "~" H 3400 6450 50  0001 C CNN
F 4 "Distrelec" H 3400 6450 50  0001 C CNN "Supplier"
F 5 "300-86-429" H 3400 6450 50  0001 C CNN "Supplier ref."
F 6 "0.0093" H 3400 6450 50  0001 C CNN "Price"
	1    3400 6450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5EF417A3
P 3000 6450
F 0 "C5" V 3252 6450 50  0000 C CNN
F 1 "100n" V 3161 6450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3038 6300 50  0001 C CNN
F 3 "~" H 3000 6450 50  0001 C CNN
F 4 "Distrelec" H 3000 6450 50  0001 C CNN "Supplier"
F 5 "300-86-429" H 3000 6450 50  0001 C CNN "Supplier ref."
F 6 "0.0093" H 3000 6450 50  0001 C CNN "Price"
	1    3000 6450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 5EF06534
P 5600 5150
F 0 "#PWR0121" H 5600 4900 50  0001 C CNN
F 1 "GND" H 5605 4977 50  0000 C CNN
F 2 "" H 5600 5150 50  0001 C CNN
F 3 "" H 5600 5150 50  0001 C CNN
	1    5600 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 5050 5400 5150
Wire Wire Line
	5400 5150 5500 5150
Wire Wire Line
	5500 5050 5500 5150
Connection ~ 5500 5150
Wire Wire Line
	5500 5150 5600 5150
Wire Wire Line
	5600 5050 5600 5150
Connection ~ 5600 5150
Wire Wire Line
	5700 5050 5700 5150
Wire Wire Line
	5700 5150 5600 5150
Wire Wire Line
	5800 5050 5800 5150
Wire Wire Line
	5800 5150 5700 5150
Connection ~ 5700 5150
$Comp
L Device:C C3
U 1 1 5EF0DF8A
P 4750 2350
F 0 "C3" H 4865 2396 50  0000 L CNN
F 1 "4.7u" H 4865 2305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4788 2200 50  0001 C CNN
F 3 "~" H 4750 2350 50  0001 C CNN
F 4 "Distrelec" H 4750 2350 50  0001 C CNN "Supplier"
F 5 "300-31-676" H 4750 2350 50  0001 C CNN "Supplier ref."
F 6 "0.3896" H 4750 2350 50  0001 C CNN "Price"
	1    4750 2350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 5EF1299F
P 4600 2550
F 0 "#PWR0122" H 4600 2300 50  0001 C CNN
F 1 "GND" H 4605 2377 50  0000 C CNN
F 2 "" H 4600 2550 50  0001 C CNN
F 3 "" H 4600 2550 50  0001 C CNN
	1    4600 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 2400 4450 2550
Wire Wire Line
	4450 2550 4600 2550
Wire Wire Line
	4750 2500 4750 2550
Wire Wire Line
	4750 2550 4600 2550
Connection ~ 4600 2550
Wire Wire Line
	4900 1950 4450 1950
Wire Wire Line
	4450 1950 4450 2100
Wire Wire Line
	4750 2200 4750 2150
Wire Wire Line
	4750 2150 4900 2150
Wire Wire Line
	5600 1200 5700 1200
Text HLabel 5600 1000 1    50   Input ~ 0
3.0V
Wire Wire Line
	5600 1000 5600 1200
$Comp
L MCU_ST_STM32F4:STM32F446RETx U1
U 1 1 5EF2EE57
P 5600 3250
F 0 "U1" H 4800 1450 50  0000 C CNN
F 1 "STM32F446RETx" H 5000 1350 50  0000 C CNN
F 2 "Package_QFP:LQFP-64_10x10mm_P0.5mm" H 5000 1550 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00141306.pdf" H 5600 3250 50  0001 C CNN
F 4 "Distrelec" H 5600 3250 50  0001 C CNN "Supplier"
F 5 "301-70-759" H 5600 3250 50  0001 C CNN "Supplier ref."
F 6 "8.63" H 5600 3250 50  0001 C CNN "Price"
F 7 "Microcontrôleur 32bit 512KB LQFP-64" H 5600 3250 50  0001 C CNN "Description"
	1    5600 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5EF27DB4
P 7050 1150
F 0 "C2" H 7165 1196 50  0000 L CNN
F 1 "1u" H 7165 1105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7088 1000 50  0001 C CNN
F 3 "~" H 7050 1150 50  0001 C CNN
F 4 "Distrelec" H 7050 1150 50  0001 C CNN "Supplier"
F 5 "301-44-765" H 7050 1150 50  0001 C CNN "Supplier ref."
F 6 "0.0629" H 7050 1150 50  0001 C CNN "Price"
	1    7050 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 1000 7050 1000
Connection ~ 6600 1000
Wire Wire Line
	7050 1300 6850 1300
Connection ~ 6850 1300
Wire Wire Line
	6850 1300 6600 1300
Wire Wire Line
	4200 6700 4600 6700
Wire Wire Line
	4200 6300 4600 6300
Wire Wire Line
	4600 6600 4600 6700
$Comp
L Device:C C9
U 1 1 5EF2E5AE
P 4600 6450
F 0 "C9" V 4852 6450 50  0000 C CNN
F 1 "100n" V 4761 6450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4638 6300 50  0001 C CNN
F 3 "~" H 4600 6450 50  0001 C CNN
F 4 "Distrelec" H 4600 6450 50  0001 C CNN "Supplier"
F 5 "300-86-429" H 4600 6450 50  0001 C CNN "Supplier ref."
F 6 "0.0093" H 4600 6450 50  0001 C CNN "Price"
	1    4600 6450
	1    0    0    -1  
$EndComp
$Comp
L power:+3V0 #PWR0124
U 1 1 5EF2FB0B
P 2400 6250
F 0 "#PWR0124" H 2400 6100 50  0001 C CNN
F 1 "+3V0" H 2415 6423 50  0000 C CNN
F 2 "" H 2400 6250 50  0001 C CNN
F 3 "" H 2400 6250 50  0001 C CNN
	1    2400 6250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 5EF30B6A
P 2400 6650
F 0 "#PWR0125" H 2400 6400 50  0001 C CNN
F 1 "GND" V 2405 6522 50  0000 R CNN
F 2 "" H 2400 6650 50  0001 C CNN
F 3 "" H 2400 6650 50  0001 C CNN
	1    2400 6650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5EF31DDB
P 2400 6450
F 0 "C4" H 2515 6496 50  0000 L CNN
F 1 "4.7u" H 2515 6405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2438 6300 50  0001 C CNN
F 3 "~" H 2400 6450 50  0001 C CNN
F 4 "Distrelec" H 2400 6450 50  0001 C CNN "Supplier"
F 5 "300-31-676" H 2400 6450 50  0001 C CNN "Supplier ref."
F 6 "0.3896" H 2400 6450 50  0001 C CNN "Price"
	1    2400 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 6300 2400 6250
Wire Wire Line
	2400 6650 2400 6600
Wire Wire Line
	3400 6300 3800 6300
$Comp
L power:GND #PWR0126
U 1 1 5EF446E5
P 3800 6850
F 0 "#PWR0126" H 3800 6600 50  0001 C CNN
F 1 "GND" V 3805 6722 50  0000 R CNN
F 2 "" H 3800 6850 50  0001 C CNN
F 3 "" H 3800 6850 50  0001 C CNN
	1    3800 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 6700 3800 6700
Text HLabel 6400 2150 2    50   Input ~ 0
I2S1_WS
Text HLabel 6400 2450 2    50   Input ~ 0
I2S1_SD
Text HLabel 6400 3450 2    50   Input ~ 0
I2S3_SD
Text HLabel 6400 3250 2    50   Input ~ 0
I2S3_WS
Text HLabel 4800 4350 0    50   Input ~ 0
I2S3_CK
Text HLabel 6400 3750 2    50   Input ~ 0
I2S1_CK
Wire Wire Line
	6400 2450 6300 2450
Wire Wire Line
	6400 2150 6300 2150
Wire Wire Line
	6400 3250 6300 3250
Wire Wire Line
	6400 3450 6300 3450
Wire Wire Line
	6400 3750 6300 3750
Wire Wire Line
	4800 4350 4900 4350
Text HLabel 5800 5200 3    50   Input ~ 0
GND
Wire Wire Line
	5800 5150 5800 5200
Connection ~ 5800 5150
Connection ~ 4200 6300
Connection ~ 4200 6700
Text Notes 3250 5900 0    50   ~ 0
Decoupling capacitances\n (1 for each VCC/VSS pair + Vbat)
Text HLabel 4750 1750 0    50   Input ~ 0
nRESET
Text HLabel 6400 3050 2    50   Input ~ 0
SWDIO
Text HLabel 6400 3150 2    50   Input ~ 0
SWCLK
Wire Wire Line
	6300 3050 6400 3050
Wire Wire Line
	6300 3150 6400 3150
Text HLabel 6450 4450 2    50   Input ~ 0
SPI2_CK
Wire Wire Line
	6300 4450 6450 4450
Text HLabel 6450 4550 2    50   Input ~ 0
SPI2_NSS
Wire Wire Line
	6450 4550 6300 4550
Text HLabel 4750 3450 0    50   Input ~ 0
SPI2_MOSI
Wire Wire Line
	4750 3450 4900 3450
Text HLabel 6400 1750 2    50   Input ~ 0
UART4_TX
Text HLabel 6400 1850 2    50   Input ~ 0
UART4_RX
Wire Wire Line
	6300 1750 6400 1750
Wire Wire Line
	6300 1850 6400 1850
Text HLabel 6450 4150 2    50   Input ~ 0
I2C1_SDA
Text HLabel 6450 4050 2    50   Input ~ 0
I2C1_SCL
Wire Wire Line
	6300 4050 6450 4050
Wire Wire Line
	6300 4150 6450 4150
Text HLabel 4750 3550 0    50   Input ~ 0
SPI2_MISO
Wire Wire Line
	4750 3550 4900 3550
Wire Wire Line
	4750 1750 4900 1750
Text HLabel 4750 3650 0    50   Input ~ 0
PC3
Text HLabel 6400 1950 2    50   Input ~ 0
PA2
Wire Wire Line
	4750 3650 4900 3650
Wire Wire Line
	6400 1950 6300 1950
Text HLabel 6400 2350 2    50   Input ~ 0
TIM3_CH1
Wire Wire Line
	6400 2350 6300 2350
Text HLabel 6400 2550 2    50   Input ~ 0
TIM1_CH1
Text HLabel 6400 2650 2    50   Input ~ 0
TIM1_CH2
Text HLabel 6400 2750 2    50   Input ~ 0
TIM1_CH3
Text HLabel 6400 2850 2    50   Input ~ 0
TIM1_CH4
Wire Wire Line
	6400 2550 6300 2550
Wire Wire Line
	6400 2650 6300 2650
Wire Wire Line
	6400 2750 6300 2750
Wire Wire Line
	6400 2850 6300 2850
$EndSCHEMATC
