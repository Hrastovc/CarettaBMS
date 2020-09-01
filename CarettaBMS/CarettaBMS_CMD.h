#define CMD_00 (uint8_t)('!')
#define CMD_01 (uint8_t)('"')
#define CMD_02 (uint8_t)('$')
#define CMD_03 (uint8_t)('(')
#define CMD_04 (uint8_t)('+')
#define CMD_05 (uint8_t)('-')
#define CMD_06 (uint8_t)('.')
#define CMD_07 (uint8_t)('3')
#define CMD_08 (uint8_t)('5')
#define CMD_09 (uint8_t)('6')
#define CMD_10 (uint8_t)('9')
#define CMD_11 (uint8_t)(':')
#define CMD_12 (uint8_t)('<')
#define CMD_13 (uint8_t)('?')

#define CMD_ALARM_INTERVAL     (uint8_t)('A') /* 0x41 */
#define CMD_ALARM_CONT_SINGLE  (uint8_t)('B') /* 0x42 */
#define CMD_VOLTAGE_CAL        (uint8_t)('c') /* 0x63 */
#define CMD_BALANCE_CURRENT    (uint8_t)('D') /* 0x44 */
#define CMD_EXT_TEMPERATURE    (uint8_t)('e') /* 0x65 */
#define CMD_MAX_TEMP_BMS       (uint8_t)('f') /* 0x66 */
#define CMD_MAX_TEMP_BAT       (uint8_t)('G') /* 0x47 */
#define CMD_HIGH_VOLT_LIMIT    (uint8_t)('H') /* 0x48 */
#define CMD_CHARGED_VOLTAGE    (uint8_t)('i') /* 0x69 */
#define CMD_EMPTY_VOLTAGE      (uint8_t)('j') /* 0x6A */
#define CMD_LOW_VOLT_LIMIT     (uint8_t)('l') /* 0x6C */
#define CMD_EXT_TEMP_CAL       (uint8_t)('K') /* 0x4B */
#define CMD_MEASURE_INTERVAL   (uint8_t)('M') /* 0x4D */
#define CMD_NMBR_OF_MODULES    (uint8_t)('N') /* 0x4E */
#define CMD_BALANCE_START      (uint8_t)('o') /* 0x6F */
#define CMD_BALANCE_STOP       (uint8_t)('P') /* 0x50 */
#define CMD_BALANCE_HYSTERESIS (uint8_t)('q') /* 0x71 */
#define CMD_RESET_SETTINGS     (uint8_t)('r') /* 0x72 */
#define CMD_GOTO_SLEEP         (uint8_t)('S') /* 0x53 */
#define CMD_INT_TEMPERATURE    (uint8_t)('t') /* 0x74 */
#define CMD_TARGET_VOLTAGE     (uint8_t)('U') /* 0x55 */
#define CMD_CELL_VOLTAGE       (uint8_t)('V') /* 0x56 */
#define CMD_WAKE_UP_SYNC       (uint8_t)('w') /* 0x77 */
#define CMD_EXTRA_0            (uint8_t)('x') /* 0x78 */
#define CMD_EXTRA_1            (uint8_t)('Y') /* 0x59 */
#define CMD_EXTRA_2            (uint8_t)('Z') /* 0x5A */

#define CMD_28 (uint8_t)('\\')
#define CMD_29 (uint8_t)('_')
#define CMD_42 (uint8_t)('{')
#define CMD_43 (uint8_t)('}')
#define CMD_44 (uint8_t)('~')
