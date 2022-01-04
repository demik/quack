EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Quack back panel"
Date "2021-12-08"
Rev "1.0"
Comp "Lostwave"
Comment1 "https://68kmla.org"
Comment2 "https://github.com/quack/quack"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:Earth #PWR0101
U 1 1 61B0AB0B
P 3000 3000
F 0 "#PWR0101" H 3000 2750 50  0001 C CNN
F 1 "Earth" H 3000 2850 50  0001 C CNN
F 2 "" H 3000 3000 50  0001 C CNN
F 3 "~" H 3000 3000 50  0001 C CNN
	1    3000 3000
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Small TP1
U 1 1 61B0AF58
P 3000 2500
F 0 "TP1" H 3048 2546 50  0000 L CNN
F 1 "TestPoint_Small" H 3048 2455 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 3200 2500 50  0001 C CNN
F 3 "~" H 3200 2500 50  0001 C CNN
	1    3000 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 3000 3000 2750
$Comp
L Connector:TestPoint_Small TP2
U 1 1 61B0C87F
P 3000 2750
F 0 "TP2" H 3048 2796 50  0000 L CNN
F 1 "TestPoint_Small" H 3048 2705 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 3200 2750 50  0001 C CNN
F 3 "~" H 3200 2750 50  0001 C CNN
	1    3000 2750
	1    0    0    -1  
$EndComp
Connection ~ 3000 2750
Wire Wire Line
	3000 2750 3000 2500
$EndSCHEMATC
