EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
Title "Quack Mouse Adapter"
Date "2021-08-15"
Rev "1.4"
Comp "Lostwave"
Comment1 "https://68kmla.org"
Comment2 "https://github.com/demik/quack/tree/master/EDA"
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 9500 5000 1000 1000
U 5F5EA845
F0 "quack_connectors" 50
F1 "quack_connectors.sch" 50
$EndSheet
$Comp
L Regulator_Linear:AMS1117-3.3 U1
U 1 1 5F607118
P 9000 2000
F 0 "U1" H 9000 2242 50  0000 C CNN
F 1 "AMS1117-3.3" H 9000 2151 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 9000 2200 50  0001 C CNN
F 3 "http://www.advanced-monolithic.com/pdf/ds1117.pdf" H 9100 1750 50  0001 C CNN
	1    9000 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5F60BEC0
P 9000 2600
F 0 "#PWR0108" H 9000 2350 50  0001 C CNN
F 1 "GND" H 9005 2427 50  0000 C CNN
F 2 "" H 9000 2600 50  0001 C CNN
F 3 "" H 9000 2600 50  0001 C CNN
	1    9000 2600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0109
U 1 1 5F60C48E
P 8500 1750
F 0 "#PWR0109" H 8500 1600 50  0001 C CNN
F 1 "+5V" H 8400 1950 50  0000 L CNN
F 2 "" H 8500 1750 50  0001 C CNN
F 3 "" H 8500 1750 50  0001 C CNN
	1    8500 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0110
U 1 1 5F60D325
P 9500 1750
F 0 "#PWR0110" H 9500 1600 50  0001 C CNN
F 1 "+3.3V" H 9400 1950 50  0000 L CNN
F 2 "" H 9500 1750 50  0001 C CNN
F 3 "" H 9500 1750 50  0001 C CNN
	1    9500 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5F60FDC9
P 8500 2250
F 0 "C1" H 8615 2296 50  0000 L CNN
F 1 "10uF" H 8615 2205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8538 2100 50  0001 C CNN
F 3 "~" H 8500 2250 50  0001 C CNN
	1    8500 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 2300 9000 2500
Wire Wire Line
	9000 2500 8500 2500
Wire Wire Line
	8500 2500 8500 2400
Connection ~ 9000 2500
Wire Wire Line
	9000 2500 9000 2600
Wire Wire Line
	8500 2100 8500 2000
Wire Wire Line
	8500 2000 8700 2000
Wire Wire Line
	8500 2000 8500 1750
Connection ~ 8500 2000
$Comp
L Device:CP C2
U 1 1 5F6139D7
P 9500 2250
F 0 "C2" H 9618 2296 50  0000 L CNN
F 1 "100uF" H 9618 2205 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-3528-12_Kemet-T" H 9538 2100 50  0001 C CNN
F 3 "~" H 9500 2250 50  0001 C CNN
	1    9500 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 2000 9500 2000
Wire Wire Line
	9500 2000 9500 2100
Wire Wire Line
	9500 2400 9500 2500
Wire Wire Line
	9500 2500 9000 2500
Wire Wire Line
	9500 1750 9500 2000
Connection ~ 9500 2000
$Comp
L power:+3.3V #PWR0111
U 1 1 5F6191ED
P 4850 1400
F 0 "#PWR0111" H 4850 1250 50  0001 C CNN
F 1 "+3.3V" H 4865 1573 50  0000 C CNN
F 2 "" H 4850 1400 50  0001 C CNN
F 3 "" H 4850 1400 50  0001 C CNN
	1    4850 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 1500 4850 2000
Wire Wire Line
	4950 2000 4950 1500
Wire Wire Line
	4950 1500 4850 1500
Connection ~ 4850 1500
Wire Wire Line
	4850 1500 4850 1400
Wire Wire Line
	4750 2000 4750 1500
Wire Wire Line
	4750 1500 4850 1500
Wire Wire Line
	5050 2000 5050 1500
Connection ~ 4950 1500
$Comp
L power:GND #PWR0112
U 1 1 5F61B257
P 6050 4700
F 0 "#PWR0112" H 6050 4450 50  0001 C CNN
F 1 "GND" H 6055 4527 50  0000 C CNN
F 2 "" H 6050 4700 50  0001 C CNN
F 3 "" H 6050 4700 50  0001 C CNN
	1    6050 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 4600 6050 4600
Wire Wire Line
	6050 4600 6050 4700
Wire Wire Line
	5150 2000 5150 1500
Wire Wire Line
	4950 1500 5050 1500
Connection ~ 5050 1500
Wire Wire Line
	5050 1500 5150 1500
Wire Wire Line
	5250 2000 5250 1500
Wire Wire Line
	5250 1500 5150 1500
Connection ~ 5150 1500
$Comp
L power:+3.3V #PWR0113
U 1 1 5F61CCB7
P 6500 3500
F 0 "#PWR0113" H 6500 3350 50  0001 C CNN
F 1 "+3.3V" V 6515 3628 50  0000 L CNN
F 2 "" H 6500 3500 50  0001 C CNN
F 3 "" H 6500 3500 50  0001 C CNN
	1    6500 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	5950 3500 6500 3500
NoConn ~ 5950 2900
NoConn ~ 5950 3000
NoConn ~ 5950 3100
NoConn ~ 4150 4700
NoConn ~ 4150 4200
NoConn ~ 4150 4400
$Comp
L Device:LED D4
U 1 1 5F621DE4
P 7250 4000
F 0 "D4" H 7243 4216 50  0000 C CNN
F 1 "GREEN LED" H 7243 4125 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric_Castellated" H 7250 4000 50  0001 C CNN
F 3 "~" H 7250 4000 50  0001 C CNN
	1    7250 4000
	-1   0    0    1   
$EndComp
Wire Wire Line
	5950 4000 6600 4000
$Comp
L Device:LED D1
U 1 1 5F6256F8
P 2750 3000
F 0 "D1" H 2743 3216 50  0000 C CNN
F 1 "BLUE LED" H 2743 3125 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric_Castellated" H 2750 3000 50  0001 C CNN
F 3 "~" H 2750 3000 50  0001 C CNN
	1    2750 3000
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 5F626D09
P 2750 3500
F 0 "D2" H 2743 3716 50  0000 C CNN
F 1 "YELLOW LED" H 2743 3625 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric_Castellated" H 2750 3500 50  0001 C CNN
F 3 "~" H 2750 3500 50  0001 C CNN
	1    2750 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D3
U 1 1 5F62988B
P 2750 4000
F 0 "D3" H 2743 4216 50  0000 C CNN
F 1 "RED LED" H 2743 4125 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric_Castellated" H 2750 4000 50  0001 C CNN
F 3 "~" H 2750 4000 50  0001 C CNN
	1    2750 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5F62B2A2
P 3250 3000
F 0 "R1" V 3043 3000 50  0000 C CNN
F 1 "470" V 3134 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3180 3000 50  0001 C CNN
F 3 "~" H 3250 3000 50  0001 C CNN
	1    3250 3000
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5F62BD09
P 3250 3500
F 0 "R2" V 3043 3500 50  0000 C CNN
F 1 "470" V 3134 3500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3180 3500 50  0001 C CNN
F 3 "~" H 3250 3500 50  0001 C CNN
	1    3250 3500
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5F62CA4D
P 3250 4000
F 0 "R3" V 3043 4000 50  0000 C CNN
F 1 "470" V 3134 4000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3180 4000 50  0001 C CNN
F 3 "~" H 3250 4000 50  0001 C CNN
	1    3250 4000
	0    1    1    0   
$EndComp
Wire Wire Line
	3100 3000 2900 3000
Wire Wire Line
	3100 3500 2900 3500
Wire Wire Line
	3100 4000 2900 4000
Wire Wire Line
	4150 3200 3600 3200
Wire Wire Line
	4150 3300 3500 3300
Wire Wire Line
	3500 3300 3500 3500
Wire Wire Line
	3500 3500 3400 3500
Wire Wire Line
	4150 3400 3600 3400
Wire Wire Line
	3600 3400 3600 4000
Wire Wire Line
	3600 4000 3400 4000
Wire Wire Line
	3600 3200 3600 3000
Wire Wire Line
	3400 3000 3600 3000
$Comp
L power:GND #PWR0114
U 1 1 5F631BD7
P 2250 4100
F 0 "#PWR0114" H 2250 3850 50  0001 C CNN
F 1 "GND" H 2255 3927 50  0000 C CNN
F 2 "" H 2250 4100 50  0001 C CNN
F 3 "" H 2250 4100 50  0001 C CNN
	1    2250 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 3000 2250 3500
Connection ~ 2250 4000
Wire Wire Line
	2250 4000 2250 4100
Connection ~ 2250 3500
Wire Wire Line
	2250 3500 2250 4000
Wire Wire Line
	2250 3000 2600 3000
Wire Wire Line
	2250 3500 2600 3500
Wire Wire Line
	2250 4000 2600 4000
$Comp
L Device:R R4
U 1 1 5F63A510
P 6750 4000
F 0 "R4" V 6543 4000 50  0000 C CNN
F 1 "1200" V 6634 4000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6680 4000 50  0001 C CNN
F 3 "~" H 6750 4000 50  0001 C CNN
	1    6750 4000
	0    1    1    0   
$EndComp
Wire Wire Line
	6900 4000 7100 4000
$Comp
L power:GND #PWR0115
U 1 1 5F64B0BE
P 7750 4100
F 0 "#PWR0115" H 7750 3850 50  0001 C CNN
F 1 "GND" H 7755 3927 50  0000 C CNN
F 2 "" H 7750 4100 50  0001 C CNN
F 3 "" H 7750 4100 50  0001 C CNN
	1    7750 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 4100 7750 4000
Wire Wire Line
	7750 4000 7400 4000
Text GLabel 6050 3800 2    50   Input ~ 0
U0RXD
Text GLabel 6050 3900 2    50   Output ~ 0
U0TXD
Wire Wire Line
	6050 3800 5950 3800
Wire Wire Line
	6050 3900 5950 3900
Text GLabel 4050 4000 0    50   Input ~ 0
~FLASH
$Comp
L Device:C C4
U 1 1 5F66EA30
P 4500 1750
F 0 "C4" H 4615 1796 50  0000 L CNN
F 1 "100nF" H 4500 1650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4538 1600 50  0001 C CNN
F 3 "~" H 4500 1750 50  0001 C CNN
	1    4500 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5F66FB55
P 4000 1750
F 0 "C3" H 4115 1796 50  0000 L CNN
F 1 "10uF" H 4115 1705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4038 1600 50  0001 C CNN
F 3 "~" H 4000 1750 50  0001 C CNN
	1    4000 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 1500 4500 1500
Wire Wire Line
	4000 1500 4000 1600
Connection ~ 4750 1500
$Comp
L power:GND #PWR0118
U 1 1 5F671A03
P 4000 2100
F 0 "#PWR0118" H 4000 1850 50  0001 C CNN
F 1 "GND" H 4005 1927 50  0000 C CNN
F 2 "" H 4000 2100 50  0001 C CNN
F 3 "" H 4000 2100 50  0001 C CNN
	1    4000 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2100 4000 2000
Wire Wire Line
	4500 1900 4500 2000
Wire Wire Line
	4500 2000 4000 2000
Connection ~ 4000 2000
Wire Wire Line
	4000 2000 4000 1900
Wire Wire Line
	4500 1600 4500 1500
Connection ~ 4500 1500
Wire Wire Line
	4500 1500 4000 1500
Text GLabel 4050 3600 0    50   Output ~ 0
QX1_3V
Text GLabel 4050 3700 0    50   Output ~ 0
QX2_3V
Text GLabel 4050 3500 0    50   Output ~ 0
QY1_3V
Text GLabel 4050 3800 0    50   Output ~ 0
QY2_3V
Text GLabel 4050 3900 0    50   Output ~ 0
CLICK_3V
Wire Wire Line
	4050 3500 4150 3500
Wire Wire Line
	4050 3600 4150 3600
Wire Wire Line
	4050 3700 4150 3700
Wire Wire Line
	4050 3800 4150 3800
Wire Wire Line
	4050 3900 4150 3900
Text GLabel 4050 4100 0    50   BiDi ~ 0
ADB
Wire Wire Line
	4150 4100 4050 4100
NoConn ~ 5950 4100
NoConn ~ 4150 5000
NoConn ~ 4150 4900
NoConn ~ 4150 4800
NoConn ~ 5950 4300
NoConn ~ 5950 4200
NoConn ~ 5950 4400
NoConn ~ 5950 4500
Text GLabel 4050 3100 0    50   Input ~ 0
~BTDIS
Text GLabel 4050 3000 0    50   Input ~ 0
~ADBHOST
Wire Wire Line
	4150 3100 4050 3100
Wire Wire Line
	4150 3000 4050 3000
NoConn ~ 4150 4600
NoConn ~ 4150 4500
NoConn ~ 4150 4300
NoConn ~ 5950 3400
NoConn ~ 5950 3300
NoConn ~ 5950 3200
NoConn ~ 4150 5200
NoConn ~ 4150 5300
$Comp
L Device:R R5
U 1 1 5F628CC1
P 3500 4850
F 0 "R5" H 3570 4896 50  0000 L CNN
F 1 "10k" H 3570 4805 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3430 4850 50  0001 C CNN
F 3 "~" H 3500 4850 50  0001 C CNN
	1    3500 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5F6294FB
P 3500 5350
F 0 "C5" H 3615 5396 50  0000 L CNN
F 1 "100nF" H 3615 5305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3538 5200 50  0001 C CNN
F 3 "~" H 3500 5350 50  0001 C CNN
	1    3500 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 5200 3500 5100
Wire Wire Line
	3500 5100 4150 5100
Connection ~ 3500 5100
Wire Wire Line
	3500 5100 3500 5000
$Comp
L power:GND #PWR0121
U 1 1 5F62D413
P 3500 5600
F 0 "#PWR0121" H 3500 5350 50  0001 C CNN
F 1 "GND" H 3505 5427 50  0000 C CNN
F 2 "" H 3500 5600 50  0001 C CNN
F 3 "" H 3500 5600 50  0001 C CNN
	1    3500 5600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0122
U 1 1 5F62DCC0
P 3500 4600
F 0 "#PWR0122" H 3500 4450 50  0001 C CNN
F 1 "+3.3V" H 3515 4773 50  0000 C CNN
F 2 "" H 3500 4600 50  0001 C CNN
F 3 "" H 3500 4600 50  0001 C CNN
	1    3500 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 4600 3500 4700
Wire Wire Line
	3500 5500 3500 5600
$Comp
L ESP32-PICO-D4:ESP32-PICO-D4 IC1
U 1 1 5F5E5479
P 4150 2900
F 0 "IC1" H 5050 327 50  0000 C CNN
F 1 "ESP32-PICO-D4" H 5050 236 50  0000 C CNN
F 2 "ESP32-PICO-D4:ESP32-PICO-D4_1" H 5800 3600 50  0001 L CNN
F 3 "https://hr.mouser.com/datasheet/2/891/esp32-pico-d4_datasheet_en-1365829.pdf" H 5800 3500 50  0001 L CNN
F 4 "ESP32-PICO-D4 module" H 5800 3400 50  0001 L CNN "Description"
F 5 "" H 5800 3300 50  0001 L CNN "Height"
F 6 "356-ESP32-PICO-D4" H 5800 3200 50  0001 L CNN "Mouser Part Number"
F 7 "" H 5800 3100 50  0001 L CNN "Mouser Price/Stock"
F 8 "Espressif" H 5800 3000 50  0001 L CNN "Manufacturer_Name"
F 9 "ESP32-PICO-D4" H 5800 2900 50  0001 L CNN "Manufacturer_Part_Number"
	1    4150 2900
	1    0    0    -1  
$EndComp
Text GLabel 6050 3700 2    50   Output ~ 0
~OE
Wire Wire Line
	5950 3700 6050 3700
$Comp
L Device:Antenna ANT1
U 1 1 5FDD7966
P 3500 1700
F 0 "ANT1" H 3250 1550 50  0000 L CNN
F 1 "Antenna_Shield" V 3644 1648 50  0001 L CNN
F 2 "RF_Antenna:Texas_SWRA117D_2.4GHz_Right" H 3500 1800 50  0001 C CNN
F 3 "~" H 3500 1800 50  0001 C CNN
	1    3500 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 2900 3500 1900
Wire Wire Line
	3500 2900 4150 2900
Text GLabel 6050 3600 2    50   Output ~ 0
DIR
Wire Wire Line
	5950 3600 6050 3600
Wire Wire Line
	4050 4000 4150 4000
$EndSCHEMATC
