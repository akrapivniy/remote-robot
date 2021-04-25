EESchema Schematic File Version 4
LIBS:stm8-car-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L MCU_ST_STM8:STM8L051F3P U2
U 1 1 5E2B183A
P 4200 2900
F 0 "U2" H 3800 3950 50  0000 C CNN
F 1 "STM8L051F3P" H 3800 3850 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 4250 3900 50  0001 L CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00060484.pdf" H 4200 2500 50  0001 C CNN
	1    4200 2900
	1    0    0    -1  
$EndComp
Text Label 3250 3200 0    50   ~ 0
SPI_CS
Text Label 3250 3300 0    50   ~ 0
SPI_CLK
Text Label 3250 3400 0    50   ~ 0
SPI_MOSI
Text Label 3250 3500 0    50   ~ 0
SPI_MISO
Wire Wire Line
	3600 3200 3250 3200
Wire Wire Line
	3250 3300 3600 3300
Wire Wire Line
	3600 3400 3250 3400
Wire Wire Line
	3250 3500 3600 3500
Text Label 5800 1550 0    50   ~ 0
SPI_CS
Text Label 5800 1450 0    50   ~ 0
SPI_CLK
Text Label 5800 1250 0    50   ~ 0
SPI_MOSI
Text Label 5800 1350 0    50   ~ 0
SPI_MISO
Wire Wire Line
	6200 1250 5800 1250
Wire Wire Line
	5800 1350 6200 1350
Wire Wire Line
	6200 1450 5800 1450
Wire Wire Line
	5800 1550 6200 1550
$Comp
L power:+3.3V #PWR08
U 1 1 5E2B2C03
P 4200 2000
F 0 "#PWR08" H 4200 1850 50  0001 C CNN
F 1 "+3.3V" H 4215 2173 50  0000 C CNN
F 2 "" H 4200 2000 50  0001 C CNN
F 3 "" H 4200 2000 50  0001 C CNN
	1    4200 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 5E2B2CC3
P 4200 3800
F 0 "#PWR018" H 4200 3550 50  0001 C CNN
F 1 "GND" H 4205 3627 50  0000 C CNN
F 2 "" H 4200 3800 50  0001 C CNN
F 3 "" H 4200 3800 50  0001 C CNN
	1    4200 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5E2B2D0D
P 6700 2300
F 0 "#PWR011" H 6700 2050 50  0001 C CNN
F 1 "GND" H 6705 2127 50  0000 C CNN
F 2 "" H 6700 2300 50  0001 C CNN
F 3 "" H 6700 2300 50  0001 C CNN
	1    6700 2300
	1    0    0    -1  
$EndComp
$Comp
L akrapivniy:NTS4101PT1G VT3
U 1 1 5E2F4AD3
P 8000 5600
F 0 "VT3" H 8105 5646 50  0000 L CNN
F 1 "NTS4101PT1G" H 8105 5555 50  0000 L CNN
F 2 "kicad-libraries:SOT23-3" H 8000 5600 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/NTS4101P-D.PDF" H 8000 5600 50  0001 C CNN
	1    8000 5600
	1    0    0    -1  
$EndComp
Text Label 6650 5600 0    50   ~ 0
DeviceEnable
Text Label 8100 5900 0    50   ~ 0
DevicePower
Wire Wire Line
	8000 5800 8000 5900
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 5E2F5B32
P 10650 900
F 0 "J1" H 10730 892 50  0000 L CNN
F 1 "Prog" H 10730 801 50  0000 L CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_1x04_P1.27mm_Vertical" H 10650 900 50  0001 C CNN
F 3 "~" H 10650 900 50  0001 C CNN
	1    10650 900 
	1    0    0    -1  
$EndComp
$Comp
L dk_Transistors-Bipolar-BJT-Single:BC847BLT1G Q1
U 1 1 5E2FFAAD
P 1750 4900
F 0 "Q1" H 1938 4953 60  0000 L CNN
F 1 "BC847BLT1G" H 1938 4847 60  0000 L CNN
F 2 "kicad-libraries:SOT23-3" H 1950 5100 60  0001 L CNN
F 3 "http://www.onsemi.com/pub/Collateral/BC846ALT1-D.PDF" H 1950 5200 60  0001 L CNN
F 4 "BC847BLT1GOSCT-ND" H 1950 5300 60  0001 L CNN "Digi-Key_PN"
F 5 "BC847BLT1G" H 1950 5400 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 1950 5500 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 1950 5600 60  0001 L CNN "Family"
F 8 "http://www.onsemi.com/pub/Collateral/BC846ALT1-D.PDF" H 1950 5700 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/BC847BLT1G/BC847BLT1GOSCT-ND/917834" H 1950 5800 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 45V 0.1A SOT23" H 1950 5900 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 1950 6000 60  0001 L CNN "Manufacturer"
F 12 "Active" H 1950 6100 60  0001 L CNN "Status"
	1    1750 4900
	1    0    0    -1  
$EndComp
$Comp
L akrapivniy:NTS4101PT1G VT1
U 1 1 5E2FFAB4
P 2550 4700
F 0 "VT1" H 2655 4746 50  0000 L CNN
F 1 "NTS4101PT1G" H 2655 4655 50  0000 L CNN
F 2 "kicad-libraries:SOT23-3" H 2550 4700 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/NTS4101P-D.PDF" H 2550 4700 50  0001 C CNN
	1    2550 4700
	1    0    0    -1  
$EndComp
Text Label 750  4900 0    50   ~ 0
ServoEnable
$Comp
L power:GND #PWR027
U 1 1 5E2FFABD
P 1850 5200
F 0 "#PWR027" H 1850 4950 50  0001 C CNN
F 1 "GND" H 1855 5027 50  0000 C CNN
F 2 "" H 1850 5200 50  0001 C CNN
F 3 "" H 1850 5200 50  0001 C CNN
	1    1850 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 4700 2250 4700
Text Label 2650 5000 0    50   ~ 0
ServoPower
Wire Wire Line
	2550 4900 2550 5000
Wire Wire Line
	2550 5000 2650 5000
$Comp
L Device:R R6
U 1 1 5E30104B
P 1850 4550
F 0 "R6" H 1920 4596 50  0000 L CNN
F 1 "100K" H 1920 4505 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1780 4550 50  0001 C CNN
F 3 "~" H 1850 4550 50  0001 C CNN
	1    1850 4550
	1    0    0    -1  
$EndComp
Connection ~ 1850 4700
Wire Wire Line
	1850 4400 2550 4400
Wire Wire Line
	2550 4400 2550 4500
Text Label 3050 2400 0    50   ~ 0
SWIM_RESET
Text Label 9950 1100 0    50   ~ 0
SWIM_RESET
$Comp
L power:GND #PWR04
U 1 1 5E302ACE
P 9750 1000
F 0 "#PWR04" H 9750 750 50  0001 C CNN
F 1 "GND" H 9755 827 50  0000 C CNN
F 2 "" H 9750 1000 50  0001 C CNN
F 3 "" H 9750 1000 50  0001 C CNN
	1    9750 1000
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR01
U 1 1 5E302B8F
P 9950 800
F 0 "#PWR01" H 9950 650 50  0001 C CNN
F 1 "+3.3V" H 9965 973 50  0000 C CNN
F 2 "" H 9950 800 50  0001 C CNN
F 3 "" H 9950 800 50  0001 C CNN
	1    9950 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 800  9950 800 
Wire Wire Line
	9950 1100 10450 1100
Connection ~ 2550 4400
$Comp
L Device:R R8
U 1 1 5E309C3A
P 1400 4900
F 0 "R8" V 1193 4900 50  0000 C CNN
F 1 "1K" V 1284 4900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1330 4900 50  0001 C CNN
F 3 "~" H 1400 4900 50  0001 C CNN
	1    1400 4900
	0    1    1    0   
$EndComp
Wire Wire Line
	1250 4900 750  4900
Text Label 3050 3000 0    50   ~ 0
MotorB_PWM
Text Label 3050 2800 0    50   ~ 0
MotorA_PWM
Wire Wire Line
	3600 3000 3050 3000
$Comp
L Device:C C4
U 1 1 5E311099
P 2550 5150
F 0 "C4" H 2665 5196 50  0000 L CNN
F 1 "0,1uF" H 2665 5105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2588 5000 50  0001 C CNN
F 3 "~" H 2550 5150 50  0001 C CNN
	1    2550 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5E311256
P 2850 1050
F 0 "C1" H 2965 1096 50  0000 L CNN
F 1 "100nF" H 2965 1005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2888 900 50  0001 C CNN
F 3 "~" H 2850 1050 50  0001 C CNN
	1    2850 1050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR02
U 1 1 5E315BFF
P 2850 850
F 0 "#PWR02" H 2850 700 50  0001 C CNN
F 1 "+3.3V" H 2865 1023 50  0000 C CNN
F 2 "" H 2850 850 50  0001 C CNN
F 3 "" H 2850 850 50  0001 C CNN
	1    2850 850 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5E315C55
P 2850 1300
F 0 "#PWR05" H 2850 1050 50  0001 C CNN
F 1 "GND" H 2855 1127 50  0000 C CNN
F 2 "" H 2850 1300 50  0001 C CNN
F 3 "" H 2850 1300 50  0001 C CNN
	1    2850 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR024
U 1 1 5E31D540
P 2550 5350
F 0 "#PWR024" H 2550 5100 50  0001 C CNN
F 1 "GND" H 2555 5177 50  0000 C CNN
F 2 "" H 2550 5350 50  0001 C CNN
F 3 "" H 2550 5350 50  0001 C CNN
	1    2550 5350
	1    0    0    -1  
$EndComp
Text Label 5500 2600 0    50   ~ 0
NRF_IRQ
Wire Wire Line
	3600 2800 3050 2800
Text Label 5800 1850 0    50   ~ 0
NRF_IRQ
Wire Wire Line
	6200 1850 5800 1850
Wire Wire Line
	2850 1300 2850 1200
Wire Wire Line
	6700 950  6850 950 
Wire Wire Line
	2550 5300 2550 5350
Wire Wire Line
	9750 1000 10450 1000
Wire Wire Line
	10450 900  9900 900 
Wire Wire Line
	7300 6000 7300 6050
Text Label 9900 900  0    50   ~ 0
SWIM_DATA
Wire Wire Line
	1850 5100 1850 5200
Wire Wire Line
	2550 4300 2550 4400
$Comp
L akrapivniy:NRF24L01SMD U1
U 1 1 5E3A542E
P 6700 1550
F 0 "U1" H 7178 1528 50  0000 L CNN
F 1 "NRF24L" H 7178 1437 50  0000 L CNN
F 2 "akrapivniy:NRF24L01SMD" H 6700 1700 50  0001 C CNN
F 3 "" H 6700 1700 50  0001 C CNN
	1    6700 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5E3A5ADD
P 2400 2150
F 0 "R3" H 2470 2196 50  0000 L CNN
F 1 "100K" H 2470 2105 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2330 2150 50  0001 C CNN
F 3 "~" H 2400 2150 50  0001 C CNN
	1    2400 2150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR07
U 1 1 5E3A7F9F
P 2400 1950
F 0 "#PWR07" H 2400 1800 50  0001 C CNN
F 1 "+3.3V" H 2415 2123 50  0000 C CNN
F 2 "" H 2400 1950 50  0001 C CNN
F 3 "" H 2400 1950 50  0001 C CNN
	1    2400 1950
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG03
U 1 1 5E3AC9A1
P 3050 6650
F 0 "#FLG03" H 3050 6725 50  0001 C CNN
F 1 "PWR_FLAG" H 3050 6824 50  0000 C CNN
F 2 "" H 3050 6650 50  0001 C CNN
F 3 "~" H 3050 6650 50  0001 C CNN
	1    3050 6650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5E3B9A21
P 2800 2150
F 0 "R4" H 2870 2196 50  0000 L CNN
F 1 "100K" H 2870 2105 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2730 2150 50  0001 C CNN
F 3 "~" H 2800 2150 50  0001 C CNN
	1    2800 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 2400 2400 2300
Wire Wire Line
	2400 2400 3600 2400
Wire Wire Line
	2400 2000 2400 1950
$Comp
L Device:LED D2
U 1 1 5E3E444A
P 1550 2100
F 0 "D2" H 1541 2316 50  0000 C CNN
F 1 "LED" H 1541 2225 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 1550 2100 50  0001 C CNN
F 3 "~" H 1550 2100 50  0001 C CNN
	1    1550 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5E3EEC1C
P 1550 1800
F 0 "D1" H 1541 2016 50  0000 C CNN
F 1 "LED" H 1541 1925 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 1550 1800 50  0001 C CNN
F 3 "~" H 1550 1800 50  0001 C CNN
	1    1550 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5E3EED7B
P 950 2300
F 0 "#PWR010" H 950 2050 50  0001 C CNN
F 1 "GND" H 955 2127 50  0000 C CNN
F 2 "" H 950 2300 50  0001 C CNN
F 3 "" H 950 2300 50  0001 C CNN
	1    950  2300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5E3F17F9
P 1100 1800
F 0 "R1" V 893 1800 50  0000 C CNN
F 1 "1K" V 984 1800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1030 1800 50  0001 C CNN
F 3 "~" H 1100 1800 50  0001 C CNN
	1    1100 1800
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5E3F18FF
P 1100 2100
F 0 "R2" V 893 2100 50  0000 C CNN
F 1 "1K" V 984 2100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1030 2100 50  0001 C CNN
F 3 "~" H 1100 2100 50  0001 C CNN
	1    1100 2100
	0    1    1    0   
$EndComp
Wire Wire Line
	1400 1800 1250 1800
Wire Wire Line
	950  1800 950  2100
Connection ~ 950  2100
Wire Wire Line
	950  2100 950  2300
Wire Wire Line
	1400 2100 1250 2100
$Comp
L Connector_Generic:Conn_01x02 J8
U 1 1 5E32D46C
P 10800 4200
F 0 "J8" H 10880 4192 50  0000 L CNN
F 1 "3P BAT" H 10880 4101 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 10800 4200 50  0001 C CNN
F 3 "~" H 10800 4200 50  0001 C CNN
	1    10800 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5E32D7F9
P 10550 4300
F 0 "#PWR022" H 10550 4050 50  0001 C CNN
F 1 "GND" H 10555 4127 50  0000 C CNN
F 2 "" H 10550 4300 50  0001 C CNN
F 3 "" H 10550 4300 50  0001 C CNN
	1    10550 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10550 4300 10600 4300
$Comp
L power:+3.3V #PWR033
U 1 1 5E35E9AF
P 3350 6650
F 0 "#PWR033" H 3350 6500 50  0001 C CNN
F 1 "+3.3V" H 3365 6823 50  0000 C CNN
F 2 "" H 3350 6650 50  0001 C CNN
F 3 "" H 3350 6650 50  0001 C CNN
	1    3350 6650
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:MIC5219-3.3YM5 U6
U 1 1 5E63207E
P 2650 6900
F 0 "U6" H 2650 7250 50  0000 C CNN
F 1 "L78L33_SOT89" H 2650 7150 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5_HandSoldering" H 2650 7100 50  0001 C CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/15/55/e5/aa/23/5b/43/fd/CD00000446.pdf/files/CD00000446.pdf/jcr:content/translations/en.CD00000446.pdf" H 2650 6850 50  0001 C CNN
	1    2650 6900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR036
U 1 1 5E6461EA
P 2650 7300
F 0 "#PWR036" H 2650 7050 50  0001 C CNN
F 1 "GND" H 2655 7127 50  0000 C CNN
F 2 "" H 2650 7300 50  0001 C CNN
F 3 "" H 2650 7300 50  0001 C CNN
	1    2650 7300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 7200 2650 7300
Wire Wire Line
	1850 1800 1700 1800
Wire Wire Line
	1850 2100 1700 2100
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5E6C077D
P 10000 4200
F 0 "#FLG02" H 10000 4275 50  0001 C CNN
F 1 "PWR_FLAG" H 10000 4374 50  0000 C CNN
F 2 "" H 10000 4200 50  0001 C CNN
F 3 "~" H 10000 4200 50  0001 C CNN
	1    10000 4200
	1    0    0    -1  
$EndComp
Text Label 3050 2300 0    50   ~ 0
SWIM_DATA
Wire Wire Line
	2800 2300 3600 2300
Wire Wire Line
	2800 2000 2400 2000
Connection ~ 2400 2000
$Comp
L Device:C C9
U 1 1 5E712868
P 3350 7000
F 0 "C9" H 3465 7046 50  0000 L CNN
F 1 "100nF" H 3465 6955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3388 6850 50  0001 C CNN
F 3 "~" H 3350 7000 50  0001 C CNN
	1    3350 7000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5E712916
P 2050 7150
F 0 "C8" H 2165 7196 50  0000 L CNN
F 1 "100nF" H 2165 7105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2088 7000 50  0001 C CNN
F 3 "~" H 2050 7150 50  0001 C CNN
	1    2050 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 7300 2650 7300
Connection ~ 2650 7300
Text Label 5800 1750 0    50   ~ 0
NRF_CE
Wire Wire Line
	6200 1750 5800 1750
Text Label 4950 2900 0    50   ~ 0
NRF_CE
Wire Wire Line
	10000 4200 10400 4200
$Comp
L power:+3V8 #PWR019
U 1 1 5EA2FD2A
P 10400 4200
F 0 "#PWR019" H 10400 4050 50  0001 C CNN
F 1 "+3V8" H 10415 4373 50  0000 C CNN
F 2 "" H 10400 4200 50  0001 C CNN
F 3 "" H 10400 4200 50  0001 C CNN
	1    10400 4200
	1    0    0    -1  
$EndComp
Connection ~ 10400 4200
Wire Wire Line
	10400 4200 10600 4200
$Comp
L power:+3V8 #PWR020
U 1 1 5EA2FFE4
P 2550 4300
F 0 "#PWR020" H 2550 4150 50  0001 C CNN
F 1 "+3V8" H 2565 4473 50  0000 C CNN
F 2 "" H 2550 4300 50  0001 C CNN
F 3 "" H 2550 4300 50  0001 C CNN
	1    2550 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 2500 4800 2500
$Comp
L Connector_Generic:Conn_01x03 J3
U 1 1 5EA30A71
P 10650 2500
F 0 "J3" H 10730 2542 50  0000 L CNN
F 1 "SERVO" H 10730 2451 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 10650 2500 50  0001 C CNN
F 3 "~" H 10650 2500 50  0001 C CNN
	1    10650 2500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5EA30CB8
P 9900 2600
F 0 "#PWR013" H 9900 2350 50  0001 C CNN
F 1 "GND" H 9905 2427 50  0000 C CNN
F 2 "" H 9900 2600 50  0001 C CNN
F 3 "" H 9900 2600 50  0001 C CNN
	1    9900 2600
	1    0    0    -1  
$EndComp
$Comp
L Driver_Motor:TB6612FNG U4
U 1 1 5EA497C9
P 8050 3450
F 0 "U4" H 8050 4628 50  0000 C CNN
F 1 "TB6612FNG" H 8050 4537 50  0000 C CNN
F 2 "Package_SO:SSOP-24_5.3x8.2mm_P0.65mm" H 8500 4050 50  0001 C CNN
F 3 "https://toshiba.semicon-storage.com/us/product/linear/motordriver/detail.TB6612FNG.html" H 8500 4050 50  0001 C CNN
	1    8050 3450
	1    0    0    -1  
$EndComp
$Comp
L power:+3V8 #PWR09
U 1 1 5EA49CCB
P 8350 2250
F 0 "#PWR09" H 8350 2100 50  0001 C CNN
F 1 "+3V8" H 8365 2423 50  0000 C CNN
F 2 "" H 8350 2250 50  0001 C CNN
F 3 "" H 8350 2250 50  0001 C CNN
	1    8350 2250
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR03
U 1 1 5EA49F5B
P 6850 950
F 0 "#PWR03" H 6850 800 50  0001 C CNN
F 1 "+3.3V" H 6865 1123 50  0000 C CNN
F 2 "" H 6850 950 50  0001 C CNN
F 3 "" H 6850 950 50  0001 C CNN
	1    6850 950 
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR012
U 1 1 5EA4A093
P 7750 2350
F 0 "#PWR012" H 7750 2200 50  0001 C CNN
F 1 "+3.3V" H 7765 2523 50  0000 C CNN
F 2 "" H 7750 2350 50  0001 C CNN
F 3 "" H 7750 2350 50  0001 C CNN
	1    7750 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 2350 7750 2450
$Comp
L power:GND #PWR023
U 1 1 5EA4EC12
P 8350 4550
F 0 "#PWR023" H 8350 4300 50  0001 C CNN
F 1 "GND" H 8355 4377 50  0000 C CNN
F 2 "" H 8350 4550 50  0001 C CNN
F 3 "" H 8350 4550 50  0001 C CNN
	1    8350 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 4550 8350 4450
Wire Wire Line
	8350 4450 8250 4450
Connection ~ 8350 4450
Connection ~ 8050 4450
Wire Wire Line
	8050 4450 7750 4450
Connection ~ 8150 4450
Wire Wire Line
	8150 4450 8050 4450
Connection ~ 8250 4450
Wire Wire Line
	8250 4450 8150 4450
$Comp
L Connector_Generic:Conn_01x02 J5
U 1 1 5EA58593
P 9150 3150
F 0 "J5" H 9230 3142 50  0000 L CNN
F 1 "MOTOR1" H 9230 3051 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 9150 3150 50  0001 C CNN
F 3 "~" H 9150 3150 50  0001 C CNN
	1    9150 3150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J7
U 1 1 5EA5869D
P 9150 3650
F 0 "J7" H 9230 3642 50  0000 L CNN
F 1 "MOTOR2" H 9230 3551 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 9150 3650 50  0001 C CNN
F 3 "~" H 9150 3650 50  0001 C CNN
	1    9150 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 3150 8650 3150
Wire Wire Line
	8950 3250 8650 3250
Wire Wire Line
	8650 3250 8650 3350
Connection ~ 8650 3250
Wire Wire Line
	8650 3050 8650 3150
Connection ~ 8650 3150
Wire Wire Line
	8950 3650 8650 3650
Wire Wire Line
	8650 3550 8650 3650
Connection ~ 8650 3650
Wire Wire Line
	8950 3750 8650 3750
Wire Wire Line
	8650 3750 8650 3850
Connection ~ 8650 3750
Wire Wire Line
	8150 2450 8250 2450
Connection ~ 8250 2450
Wire Wire Line
	8250 2450 8350 2450
Wire Wire Line
	8350 2450 8350 2250
Connection ~ 8350 2450
Text Label 3050 2900 0    50   ~ 0
ServoPWM
Text Label 6950 3250 0    50   ~ 0
MotorA_PWM
Text Label 6950 3350 0    50   ~ 0
MotorB_PWM
Wire Wire Line
	7450 3350 6950 3350
Wire Wire Line
	7450 3250 6950 3250
Text Label 2900 2500 0    50   ~ 0
AIN1
Text Label 2900 2600 0    50   ~ 0
AIN2
Text Label 4950 2300 0    50   ~ 0
BIN1
Text Label 4950 2400 0    50   ~ 0
BIN2
Text Label 7150 3550 0    50   ~ 0
AIN1
Text Label 7150 3650 0    50   ~ 0
AIN2
Text Label 7150 3750 0    50   ~ 0
BIN1
Text Label 7150 3850 0    50   ~ 0
BIN2
Wire Wire Line
	7450 3550 7150 3550
Wire Wire Line
	7150 3650 7450 3650
Wire Wire Line
	7450 3750 7150 3750
Wire Wire Line
	7150 3850 7450 3850
Text Label 4950 2500 0    50   ~ 0
ServoEnable
Text Label 1750 2400 0    50   ~ 0
DeviceEnable
Wire Wire Line
	3600 2500 2900 2500
Wire Wire Line
	3600 2600 2900 2600
Wire Wire Line
	4950 2900 4800 2900
Wire Wire Line
	2400 2400 1750 2400
Connection ~ 2400 2400
Text Label 4950 2600 0    50   ~ 0
UART_TX
Text Label 4950 2700 0    50   ~ 0
UART_RX
$Comp
L Connector_Generic:Conn_01x03 J2
U 1 1 5E99F9D7
P 10650 1600
F 0 "J2" H 10730 1642 50  0000 L CNN
F 1 "UART" H 10730 1551 50  0000 L CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_1x03_P1.27mm_Vertical" H 10650 1600 50  0001 C CNN
F 3 "~" H 10650 1600 50  0001 C CNN
	1    10650 1600
	1    0    0    -1  
$EndComp
Text Label 10100 1500 0    50   ~ 0
UART_TX
Text Label 10100 1600 0    50   ~ 0
UART_RX
$Comp
L power:GND #PWR06
U 1 1 5E99FB1B
P 10100 1700
F 0 "#PWR06" H 10100 1450 50  0001 C CNN
F 1 "GND" H 10105 1527 50  0000 C CNN
F 2 "" H 10100 1700 50  0001 C CNN
F 3 "" H 10100 1700 50  0001 C CNN
	1    10100 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 1700 10100 1700
Wire Wire Line
	10100 1600 10450 1600
Wire Wire Line
	10450 1500 10100 1500
Text Label 1850 1800 0    50   ~ 0
AIN1
Text Label 1850 2100 0    50   ~ 0
BIN1
Wire Wire Line
	6650 5600 7700 5600
$Comp
L Connector_Generic:Conn_01x02 J9
U 1 1 5E9C5900
P 10800 4900
F 0 "J9" H 10880 4892 50  0000 L CNN
F 1 "DEVICE2" H 10880 4801 50  0000 L CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_1x02_P1.27mm_Vertical" H 10800 4900 50  0001 C CNN
F 3 "~" H 10800 4900 50  0001 C CNN
	1    10800 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 2600 5500 2600
$Comp
L Amplifier_Current:INA181 U3
U 1 1 5E993A26
P 2250 3100
F 0 "U3" H 2591 3146 50  0000 L CNN
F 1 "INA181" H 2591 3055 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6_Handsoldering" H 2300 3150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/ina181.pdf" H 2400 3250 50  0001 C CNN
	1    2250 3100
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 5E993B46
P 1600 3150
F 0 "J4" H 1680 3142 50  0000 L CNN
F 1 "DIODE" H 1680 3051 50  0000 L CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_1x02_P1.27mm_Vertical" H 1600 3150 50  0001 C CNN
F 3 "~" H 1600 3150 50  0001 C CNN
	1    1600 3150
	-1   0    0    1   
$EndComp
Wire Wire Line
	1950 3200 1800 3200
Wire Wire Line
	1800 3200 1800 3150
$Comp
L Device:R R5
U 1 1 5E9B0043
P 1800 3450
F 0 "R5" V 1593 3450 50  0000 C CNN
F 1 "?K" V 1684 3450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1730 3450 50  0001 C CNN
F 3 "~" H 1800 3450 50  0001 C CNN
	1    1800 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5E9B022D
P 1800 3700
F 0 "#PWR017" H 1800 3450 50  0001 C CNN
F 1 "GND" H 1805 3527 50  0000 C CNN
F 2 "" H 1800 3700 50  0001 C CNN
F 3 "" H 1800 3700 50  0001 C CNN
	1    1800 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 3700 1800 3650
Wire Wire Line
	1800 3300 1800 3200
Connection ~ 1800 3200
$Comp
L power:+3.3V #PWR014
U 1 1 5E9BC232
P 1800 2750
F 0 "#PWR014" H 1800 2600 50  0001 C CNN
F 1 "+3.3V" H 1815 2923 50  0000 C CNN
F 2 "" H 1800 2750 50  0001 C CNN
F 3 "" H 1800 2750 50  0001 C CNN
	1    1800 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3000 1800 3000
Wire Wire Line
	1800 3000 1800 3050
Wire Wire Line
	1800 2750 1800 2800
Connection ~ 1800 3000
Wire Wire Line
	2150 2800 1800 2800
Connection ~ 1800 2800
Wire Wire Line
	1800 2800 1800 3000
Wire Wire Line
	2150 3400 2150 3650
Wire Wire Line
	2150 3650 1800 3650
Connection ~ 1800 3650
Wire Wire Line
	1800 3650 1800 3600
NoConn ~ 2350 3400
Wire Wire Line
	2550 3100 3600 3100
$Comp
L power:GND #PWR026
U 1 1 5E9FC56E
P 10250 5000
F 0 "#PWR026" H 10250 4750 50  0001 C CNN
F 1 "GND" H 10255 4827 50  0000 C CNN
F 2 "" H 10250 5000 50  0001 C CNN
F 3 "" H 10250 5000 50  0001 C CNN
	1    10250 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	10600 5000 10250 5000
Text Label 9900 2500 0    50   ~ 0
ServoPower
Connection ~ 7750 2450
Wire Wire Line
	7450 2450 7750 2450
Wire Wire Line
	7450 3050 7450 2450
$Comp
L dk_Transistors-Bipolar-BJT-Single:BC847BLT1G Q2
U 1 1 5EA12543
P 4900 4900
F 0 "Q2" H 5088 4953 60  0000 L CNN
F 1 "BC847BLT1G" H 5088 4847 60  0000 L CNN
F 2 "kicad-libraries:SOT23-3" H 5100 5100 60  0001 L CNN
F 3 "http://www.onsemi.com/pub/Collateral/BC846ALT1-D.PDF" H 5100 5200 60  0001 L CNN
F 4 "BC847BLT1GOSCT-ND" H 5100 5300 60  0001 L CNN "Digi-Key_PN"
F 5 "BC847BLT1G" H 5100 5400 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 5100 5500 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 5100 5600 60  0001 L CNN "Family"
F 8 "http://www.onsemi.com/pub/Collateral/BC846ALT1-D.PDF" H 5100 5700 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/BC847BLT1G/BC847BLT1GOSCT-ND/917834" H 5100 5800 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 45V 0.1A SOT23" H 5100 5900 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 5100 6000 60  0001 L CNN "Manufacturer"
F 12 "Active" H 5100 6100 60  0001 L CNN "Status"
	1    4900 4900
	1    0    0    -1  
$EndComp
$Comp
L akrapivniy:NTS4101PT1G VT2
U 1 1 5EA1254A
P 5700 4700
F 0 "VT2" H 5805 4746 50  0000 L CNN
F 1 "NTS4101PT1G" H 5805 4655 50  0000 L CNN
F 2 "kicad-libraries:SOT23-3" H 5700 4700 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/NTS4101P-D.PDF" H 5700 4700 50  0001 C CNN
	1    5700 4700
	1    0    0    -1  
$EndComp
Text Label 3900 4900 0    50   ~ 0
Device2Enable
$Comp
L power:GND #PWR028
U 1 1 5EA12552
P 5000 5200
F 0 "#PWR028" H 5000 4950 50  0001 C CNN
F 1 "GND" H 5005 5027 50  0000 C CNN
F 2 "" H 5000 5200 50  0001 C CNN
F 3 "" H 5000 5200 50  0001 C CNN
	1    5000 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 4700 5400 4700
Text Label 5800 5000 0    50   ~ 0
Device2Power
Wire Wire Line
	5700 4900 5700 5000
Wire Wire Line
	5700 5000 5800 5000
$Comp
L Device:R R7
U 1 1 5EA1255C
P 5000 4550
F 0 "R7" H 5070 4596 50  0000 L CNN
F 1 "100K" H 5070 4505 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4930 4550 50  0001 C CNN
F 3 "~" H 5000 4550 50  0001 C CNN
	1    5000 4550
	1    0    0    -1  
$EndComp
Connection ~ 5000 4700
Wire Wire Line
	5000 4400 5700 4400
Wire Wire Line
	5700 4400 5700 4500
Connection ~ 5700 4400
$Comp
L Device:R R9
U 1 1 5EA12567
P 4550 4900
F 0 "R9" V 4343 4900 50  0000 C CNN
F 1 "1K" V 4434 4900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4480 4900 50  0001 C CNN
F 3 "~" H 4550 4900 50  0001 C CNN
	1    4550 4900
	0    1    1    0   
$EndComp
Wire Wire Line
	4400 4900 3900 4900
$Comp
L Device:C C5
U 1 1 5EA1256F
P 5700 5150
F 0 "C5" H 5815 5196 50  0000 L CNN
F 1 "0,1uF" H 5815 5105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5738 5000 50  0001 C CNN
F 3 "~" H 5700 5150 50  0001 C CNN
	1    5700 5150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 5EA12577
P 5700 5350
F 0 "#PWR025" H 5700 5100 50  0001 C CNN
F 1 "GND" H 5705 5177 50  0000 C CNN
F 2 "" H 5700 5350 50  0001 C CNN
F 3 "" H 5700 5350 50  0001 C CNN
	1    5700 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 5300 5700 5350
Wire Wire Line
	5700 4300 5700 4400
$Comp
L power:+3V8 #PWR021
U 1 1 5EA12580
P 5700 4300
F 0 "#PWR021" H 5700 4150 50  0001 C CNN
F 1 "+3V8" H 5715 4473 50  0000 C CNN
F 2 "" H 5700 4300 50  0001 C CNN
F 3 "" H 5700 4300 50  0001 C CNN
	1    5700 4300
	1    0    0    -1  
$EndComp
Text Label 5500 2700 0    50   ~ 0
Device2Enable
Wire Wire Line
	4800 2700 5500 2700
$Comp
L Connector_Generic:Conn_01x02 J10
U 1 1 5EA229F4
P 10800 5450
F 0 "J10" H 10880 5442 50  0000 L CNN
F 1 "DEVICE" H 10880 5351 50  0000 L CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_1x02_P1.27mm_Vertical" H 10800 5450 50  0001 C CNN
F 3 "~" H 10800 5450 50  0001 C CNN
	1    10800 5450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR030
U 1 1 5EA229FB
P 10250 5550
F 0 "#PWR030" H 10250 5300 50  0001 C CNN
F 1 "GND" H 10255 5377 50  0000 C CNN
F 2 "" H 10250 5550 50  0001 C CNN
F 3 "" H 10250 5550 50  0001 C CNN
	1    10250 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 5900 8000 5900
Text Label 10100 5450 0    50   ~ 0
DevicePower
Text Label 10000 4900 0    50   ~ 0
Device2Power
Wire Wire Line
	10600 4900 10000 4900
Wire Wire Line
	10450 2500 9900 2500
Wire Wire Line
	9900 2600 10450 2600
Wire Wire Line
	10250 5550 10600 5550
Wire Wire Line
	10600 5450 10100 5450
Wire Wire Line
	3600 2900 3050 2900
Wire Wire Line
	10450 2400 9900 2400
Text Label 9900 2400 0    50   ~ 0
ServoPWM
Wire Wire Line
	5000 5100 5000 5200
$Comp
L Device:C C2
U 1 1 5EB24651
P 9650 3200
F 0 "C2" H 9765 3246 50  0000 L CNN
F 1 "0,1uF" H 9765 3155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9688 3050 50  0001 C CNN
F 3 "~" H 9650 3200 50  0001 C CNN
	1    9650 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 3050 8650 3050
Connection ~ 8650 3050
Wire Wire Line
	9650 3350 8650 3350
Connection ~ 8650 3350
$Comp
L Device:C C3
U 1 1 5EB351AA
P 9700 3700
F 0 "C3" H 9815 3746 50  0000 L CNN
F 1 "0,1uF" H 9815 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9738 3550 50  0001 C CNN
F 3 "~" H 9700 3700 50  0001 C CNN
	1    9700 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 3550 8650 3550
Connection ~ 8650 3550
Wire Wire Line
	9700 3850 8650 3850
Connection ~ 8650 3850
Wire Wire Line
	6700 2150 6700 2300
Connection ~ 5700 5000
Connection ~ 2550 5000
$Comp
L Device:C C10
U 1 1 5E9C087A
P 2500 1050
F 0 "C10" H 2615 1096 50  0000 L CNN
F 1 "100nF" H 2615 1005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2538 900 50  0001 C CNN
F 3 "~" H 2500 1050 50  0001 C CNN
	1    2500 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 900  2850 850 
Wire Wire Line
	2850 900  2500 900 
Connection ~ 2850 900 
Wire Wire Line
	2500 1200 2850 1200
Connection ~ 2850 1200
$Comp
L Device:C C11
U 1 1 5E9D8BD9
P 2150 1050
F 0 "C11" H 2265 1096 50  0000 L CNN
F 1 "100nF" H 2265 1005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2188 900 50  0001 C CNN
F 3 "~" H 2150 1050 50  0001 C CNN
	1    2150 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 900  2150 900 
Connection ~ 2500 900 
Wire Wire Line
	2500 1200 2150 1200
Connection ~ 2500 1200
$Comp
L power:+3.3V #PWR0101
U 1 1 5E9B965A
P 8000 5200
F 0 "#PWR0101" H 8000 5050 50  0001 C CNN
F 1 "+3.3V" H 8015 5373 50  0000 C CNN
F 2 "" H 8000 5200 50  0001 C CNN
F 3 "" H 8000 5200 50  0001 C CNN
	1    8000 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 5200 8000 5400
$Comp
L Mechanical:MountingHole H1
U 1 1 5E9C2999
P 9250 6100
F 0 "H1" H 9350 6146 50  0000 L CNN
F 1 "MountingHole" H 9350 6055 50  0000 L CNN
F 2 "MountingHole:MountingHole_3mm" H 9250 6100 50  0001 C CNN
F 3 "~" H 9250 6100 50  0001 C CNN
	1    9250 6100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 5E9C2AD1
P 9500 6100
F 0 "H2" H 9600 6146 50  0000 L CNN
F 1 "MountingHole" H 9600 6055 50  0000 L CNN
F 2 "MountingHole:MountingHole_3mm" H 9500 6100 50  0001 C CNN
F 3 "~" H 9500 6100 50  0001 C CNN
	1    9500 6100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 5E9C2B6B
P 9750 6100
F 0 "H3" H 9850 6146 50  0000 L CNN
F 1 "MountingHole" H 9850 6055 50  0000 L CNN
F 2 "MountingHole:MountingHole_3mm" H 9750 6100 50  0001 C CNN
F 3 "~" H 9750 6100 50  0001 C CNN
	1    9750 6100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 5E9C2BFF
P 10000 6100
F 0 "H4" H 10100 6146 50  0000 L CNN
F 1 "MountingHole" H 10100 6055 50  0000 L CNN
F 2 "MountingHole:MountingHole_3mm" H 10000 6100 50  0001 C CNN
F 3 "~" H 10000 6100 50  0001 C CNN
	1    10000 6100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H5
U 1 1 5E9C2C8D
P 10250 6100
F 0 "H5" H 10350 6146 50  0000 L CNN
F 1 "MountingHole" H 10350 6055 50  0000 L CNN
F 2 "MountingHole:MountingHole_3mm" H 10250 6100 50  0001 C CNN
F 3 "~" H 10250 6100 50  0001 C CNN
	1    10250 6100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H6
U 1 1 5E9C2D1F
P 10500 6100
F 0 "H6" H 10600 6146 50  0000 L CNN
F 1 "MountingHole" H 10600 6055 50  0000 L CNN
F 2 "MountingHole:MountingHole_3mm" H 10500 6100 50  0001 C CNN
F 3 "~" H 10500 6100 50  0001 C CNN
	1    10500 6100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H7
U 1 1 5E9C39B2
P 10750 6100
F 0 "H7" H 10850 6146 50  0000 L CNN
F 1 "MountingHole" H 10850 6055 50  0000 L CNN
F 2 "MountingHole:MountingHole_3mm" H 10750 6100 50  0001 C CNN
F 3 "~" H 10750 6100 50  0001 C CNN
	1    10750 6100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H8
U 1 1 5E9C3A46
P 11000 6100
F 0 "H8" H 11100 6146 50  0000 L CNN
F 1 "MountingHole" H 11100 6055 50  0000 L CNN
F 2 "MountingHole:MountingHole_3mm" H 11000 6100 50  0001 C CNN
F 3 "~" H 11000 6100 50  0001 C CNN
	1    11000 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 5F2779AE
P 5350 2300
F 0 "R10" V 5143 2300 50  0000 C CNN
F 1 "1K" V 5234 2300 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5280 2300 50  0001 C CNN
F 3 "~" H 5350 2300 50  0001 C CNN
	1    5350 2300
	0    1    1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 5F277B98
P 5650 2400
F 0 "R11" V 5443 2400 50  0000 C CNN
F 1 "1K" V 5534 2400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5580 2400 50  0001 C CNN
F 3 "~" H 5650 2400 50  0001 C CNN
	1    5650 2400
	0    1    1    0   
$EndComp
Wire Wire Line
	4800 2400 5500 2400
Wire Wire Line
	4800 2300 5200 2300
Wire Wire Line
	5500 2300 5800 2300
Wire Wire Line
	5800 2300 5800 2400
$Comp
L power:+3.3V #PWR015
U 1 1 5F288699
P 5800 2200
F 0 "#PWR015" H 5800 2050 50  0001 C CNN
F 1 "+3.3V" H 5815 2373 50  0000 C CNN
F 2 "" H 5800 2200 50  0001 C CNN
F 3 "" H 5800 2200 50  0001 C CNN
	1    5800 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 2200 5800 2300
Connection ~ 5800 2300
$Comp
L power:+3V8 #PWR034
U 1 1 5EA4886A
P 2050 6700
F 0 "#PWR034" H 2050 6550 50  0001 C CNN
F 1 "+3V8" H 2065 6873 50  0000 C CNN
F 2 "" H 2050 6700 50  0001 C CNN
F 3 "" H 2050 6700 50  0001 C CNN
	1    2050 6700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5F2A1740
P 3050 7100
F 0 "C6" H 3165 7146 50  0000 L CNN
F 1 "100nF" H 3165 7055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3088 6950 50  0001 C CNN
F 3 "~" H 3050 7100 50  0001 C CNN
	1    3050 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 6800 2050 6800
Wire Wire Line
	2050 6800 2050 6700
Wire Wire Line
	2050 6800 2050 6900
Connection ~ 2050 6800
Wire Wire Line
	2350 6900 2050 6900
Connection ~ 2050 6900
Wire Wire Line
	2050 6900 2050 7000
Wire Wire Line
	2950 6800 3050 6800
Wire Wire Line
	3050 6800 3050 6650
Wire Wire Line
	3050 6800 3350 6800
Wire Wire Line
	3350 6800 3350 6650
Connection ~ 3050 6800
Wire Wire Line
	3350 6800 3350 6850
Connection ~ 3350 6800
Wire Wire Line
	3350 7150 3350 7300
Wire Wire Line
	2650 7300 3050 7300
Wire Wire Line
	3050 7250 3050 7300
Connection ~ 3050 7300
Wire Wire Line
	3050 7300 3350 7300
Wire Wire Line
	3050 6950 3050 6900
Wire Wire Line
	3050 6900 2950 6900
$Comp
L Mechanical:MountingHole H10
U 1 1 5FC96DAE
P 8200 6300
F 0 "H10" H 8300 6346 50  0000 L CNN
F 1 "MountingHole" H 8300 6255 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 8200 6300 50  0001 C CNN
F 3 "~" H 8200 6300 50  0001 C CNN
	1    8200 6300
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H9
U 1 1 5FC96EE0
P 7900 6300
F 0 "H9" H 8000 6346 50  0000 L CNN
F 1 "MountingHole" H 8000 6255 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 7900 6300 50  0001 C CNN
F 3 "~" H 7900 6300 50  0001 C CNN
	1    7900 6300
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H11
U 1 1 5FC96FA0
P 8500 6300
F 0 "H11" H 8600 6346 50  0000 L CNN
F 1 "MountingHole" H 8600 6255 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 8500 6300 50  0001 C CNN
F 3 "~" H 8500 6300 50  0001 C CNN
	1    8500 6300
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H12
U 1 1 5FC97046
P 8800 6300
F 0 "H12" H 8900 6346 50  0000 L CNN
F 1 "MountingHole" H 8900 6255 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 8800 6300 50  0001 C CNN
F 3 "~" H 8800 6300 50  0001 C CNN
	1    8800 6300
	1    0    0    -1  
$EndComp
$EndSCHEMATC