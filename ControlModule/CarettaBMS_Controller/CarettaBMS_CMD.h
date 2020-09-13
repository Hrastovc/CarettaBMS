/****************************************************************************************
 *   _____                         _     _             ____    __  __    _____ 
 *  / ____|                       | |   | |           |  _ \  |  \/  |  / ____|
 * | |        __ _   _ __    ___  | |_  | |_    __ _  | |_) | | \  / | | (___  
 * | |       / _` | | '__|  / _ \ | __| | __|  / _` | |  _ <  | |\/| |  \___ \ 
 * | |____  | (_| | | |    |  __/ | |_  | |_  | (_| | | |_) | | |  | |  ____) |
 *  \_____|  \__,_| |_|     \___|  \__|  \__|  \__,_| |____/  |_|  |_| |_____/ 
 *
 ***************************************************************************************/

/************************************************************************************//**
* \file       CarettaBMS_CMD.c
* \brief      CarettaBMS commands defines
* \internal
*----------------------------------------------------------------------------------------
*                                    L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of CarettaBMS. CarettaBMS is free software: you can redistribute it
* and/or modify it under the terms of the GNU General Public License version 3 as
* published by the Free Software Foundation.
*
* CarettaBMS is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with CarettaBMS. It
* should be located in ".\Doc\license.md". If not, check on gitHub repository to obtain
* a copy.
*
* \endinternal
****************************************************************************************/

/****************************************************************************************
* Command definitions
****************************************************************************************/
#define CMD_000 (uint8_t)(0x01)
#define CMD_001 (uint8_t)(0x02)
#define CMD_002 (uint8_t)(0x04)
#define CMD_003 (uint8_t)(0x07)
#define CMD_004 (uint8_t)(0x08)
#define CMD_005 (uint8_t)(0x0B)
#define CMD_006 (uint8_t)(0x0D)
#define CMD_007 (uint8_t)(0x0E)
#define CMD_008 (uint8_t)(0x10)
#define CMD_009 (uint8_t)(0x13)
#define CMD_010 (uint8_t)(0x15)
#define CMD_011 (uint8_t)(0x16)
#define CMD_012 (uint8_t)(0x19)
#define CMD_013 (uint8_t)(0x1A)
#define CMD_014 (uint8_t)(0x1C)
#define CMD_015 (uint8_t)(0x1F)
/* START OF ASCII CMDs from 0x20 to 0x7E */
#define CMD_016 (uint8_t)(' ')
#define CMD_017 (uint8_t)('#')
#define CMD_018 (uint8_t)('%')
#define CMD_019 (uint8_t)('&')
#define CMD_020 (uint8_t)(')')
#define CMD_021 (uint8_t)('*')
#define CMD_022 (uint8_t)(',')
#define CMD_023 (uint8_t)('/')
#define CMD_024 (uint8_t)('1')
#define CMD_025 (uint8_t)('2')
#define CMD_026 (uint8_t)('4')
#define CMD_027 (uint8_t)('7')
#define CMD_028 (uint8_t)('8')
#define CMD_029 (uint8_t)(';')
#define CMD_030 (uint8_t)('=')
#define CMD_031 (uint8_t)('>')
#define CMD_GOTO_SLEEP         (uint8_t)('@') /* 0x40 */
#define CMD_ALARM_CODES        (uint8_t)('a') /* 0x61 */
#define CMD_GOTO_BOOTLOADER    (uint8_t)('b') /* 0x62 */
#define CMD_VOLTAGE_CAL        (uint8_t)('C') /* 0x43 */
#define CMD_BALANCE_CURRENT    (uint8_t)('d') /* 0x64 */
#define CMD_EXT_INPUTS         (uint8_t)('E') /* 0x45 */
#define CMD_MAX_TEMP_BMS       (uint8_t)('F') /* 0x46 */
#define CMD_MAX_TEMP_RECOVER   (uint8_t)('g') /* 0x67 */
#define CMD_HIGH_VOLT_LIMIT    (uint8_t)('h') /* 0x68 */
#define CMD_HIGH_VOLT_RECOVER  (uint8_t)('I') /* 0x49 */
#define CMD_LOW_VOLT_RECOVER   (uint8_t)('J') /* 0x4A */
#define CMD_LOW_VOLT_LIMIT     (uint8_t)('L') /* 0x4C */
#define CMD_CHARGED_RECOVER    (uint8_t)('k') /* 0x6B */
#define CMD_MEASURE_INTERVAL   (uint8_t)('m') /* 0x6D */
#define CMD_NMBR_OF_MODULES    (uint8_t)('n') /* 0x6E */
#define CMD_BALANCE_START      (uint8_t)('O') /* 0x4F */
#define CMD_BALANCE_STOP       (uint8_t)('p') /* 0x70 */
#define CMD_BALANCE_HYSTERESIS (uint8_t)('Q') /* 0x51 */
#define CMD_RESET_SETTINGS     (uint8_t)('R') /* 0x52 */
#define CMD_SAVE_SETTINGS      (uint8_t)('s') /* 0x73 */
#define CMD_INT_TEMPERATURE    (uint8_t)('T') /* 0x54 */
#define CMD_TARGET_VOLTAGE     (uint8_t)('u') /* 0x75 */
#define CMD_CELL_VOLTAGE       (uint8_t)('v') /* 0x76 */
#define CMD_WAKE_UP_SYNC       (uint8_t)('W') /* 0x57 */
#define CMD_EMPTY_VOLTAGE      (uint8_t)('X') /* 0x58 */
#define CMD_EMPTY_RECOVER      (uint8_t)('y') /* 0x79 */
#define CMD_CHARGED_VOLTAGE    (uint8_t)('z') /* 0x7A */
#define CMD_059 (uint8_t)('[')
#define CMD_060 (uint8_t)(']')
#define CMD_061 (uint8_t)('^')
#define CMD_062 (uint8_t)('|')
/* END OF ASCII CMDs */
#define CMD_TWI_INIT           (uint8_t)(0x7F)
#define CMD_TWI_STATUS         (uint8_t)(0x80)
#define CMD_TWI_START          (uint8_t)(0x83)
#define CMD_TWI_STOP           (uint8_t)(0x85)
#define CMD_TWI_WRITE          (uint8_t)(0x86)
#define CMD_TWI_READ           (uint8_t)(0x89)
#define CMD_069 (uint8_t)(0x8A)
#define CMD_070 (uint8_t)(0x8C)
#define CMD_071 (uint8_t)(0x8F)
#define CMD_072 (uint8_t)(0x91)
#define CMD_073 (uint8_t)(0x92)
#define CMD_074 (uint8_t)(0x94)
#define CMD_075 (uint8_t)(0x97)
#define CMD_076 (uint8_t)(0x98)
#define CMD_077 (uint8_t)(0x9B)
#define CMD_078 (uint8_t)(0x9D)
#define CMD_079 (uint8_t)(0x9E)
#define CMD_080 (uint8_t)(0xA1)
#define CMD_081 (uint8_t)(0xA2)
#define CMD_082 (uint8_t)(0xA4)
#define CMD_083 (uint8_t)(0xA7)
#define CMD_084 (uint8_t)(0xA8)
#define CMD_085 (uint8_t)(0xAB)
#define CMD_086 (uint8_t)(0xAD)
#define CMD_087 (uint8_t)(0xAE)
#define CMD_088 (uint8_t)(0xB0)
#define CMD_089 (uint8_t)(0xB3)
#define CMD_090 (uint8_t)(0xB5)
#define CMD_091 (uint8_t)(0xB6)
#define CMD_092 (uint8_t)(0xB9)
#define CMD_093 (uint8_t)(0xBA)
#define CMD_094 (uint8_t)(0xBC)
#define CMD_095 (uint8_t)(0xBF)
#define CMD_096 (uint8_t)(0xC1)
#define CMD_097 (uint8_t)(0xC2)
#define CMD_098 (uint8_t)(0xC4)
#define CMD_099 (uint8_t)(0xC7)
#define CMD_100 (uint8_t)(0xC8)
#define CMD_101 (uint8_t)(0xCB)
#define CMD_102 (uint8_t)(0xCD)
#define CMD_103 (uint8_t)(0xCE)
#define CMD_104 (uint8_t)(0xD0)
#define CMD_105 (uint8_t)(0xD3)
#define CMD_106 (uint8_t)(0xD5)
#define CMD_107 (uint8_t)(0xD6)
#define CMD_108 (uint8_t)(0xD9)
#define CMD_109 (uint8_t)(0xDA)
#define CMD_110 (uint8_t)(0xDC)
#define CMD_111 (uint8_t)(0xDF)
#define CMD_112 (uint8_t)(0xE0)
#define CMD_113 (uint8_t)(0xE3)
#define CMD_114 (uint8_t)(0xE5)
#define CMD_115 (uint8_t)(0xE6)
#define CMD_116 (uint8_t)(0xE9)
#define CMD_117 (uint8_t)(0xEA)
#define CMD_118 (uint8_t)(0xEC)
#define CMD_119 (uint8_t)(0xEF)
#define CMD_120 (uint8_t)(0xF1)
#define CMD_121 (uint8_t)(0xF2)
#define CMD_122 (uint8_t)(0xF4)
#define CMD_123 (uint8_t)(0xF7)
#define CMD_124 (uint8_t)(0xF8)
#define CMD_125 (uint8_t)(0xFB)
#define CMD_126 (uint8_t)(0xFD)
#define CMD_127 (uint8_t)(0xFE)

/******************************* end of CarettaBMS_CMD.c *******************************/