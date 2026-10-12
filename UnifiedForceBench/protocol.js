/**
 * Dynamixel Protocol 1.0 & Protocol 2.0 Encoder, Decoder, CRC & Control Table
 * Unified Force Bench - Hardware Communication Engine
 */

// CRC-16 Lookup Table for Protocol 2.0 (Polynomial 0x8005)
const CRC16_TABLE = new Uint16Array([
  0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
  0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
  0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
  0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
  0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
  0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
  0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
  0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
  0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
  0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
  0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
  0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
  0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
  0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
  0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
  0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
  0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
  0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
  0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
  0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
  0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
  0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
  0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
  0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
  0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
  0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
  0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
]);

// Instructions
const DXL_INST = {
  PING: 0x01,
  READ: 0x02,
  WRITE: 0x03,
  REG_WRITE: 0x04,
  ACTION: 0x05,
  FACTORY_RESET: 0x06,
  REBOOT: 0x08,
  CLEAR: 0x10,
  STATUS: 0x55,
  SYNC_READ: 0x82,
  SYNC_WRITE: 0x83,
  BULK_READ: 0x92,
  BULK_WRITE: 0x93,
};

// Known Model Dictionary
const DXL_MODELS = {
  0x000c: "AX-12A",
  0x0012: "AX-18A",
  0x001d: "MX-28",
  0x0036: "MX-64",
  0x0040: "MX-106",
  0x0424: "XL430-W250",
  0x0438: "XC430-W150",
  0x0442: "XC430-W240",
  0x0406: "XM430-W210",
  0x0410: "XM430-W350",
  0x0460: "XH430-W210",
  0x046a: "XH430-W350",
  0x047e: "XL330-M077",
  0x0488: "XL330-M288",
  0x042e: "XC330-M181",
  0x0438: "XC330-M288",
};

// Control Table Offsets for Protocol 2.0 (X-Series: XL430/XM430/XC430/XL330/MX-2.0)
const DXL_P2_CTRL = {
  MODEL_NUMBER: { addr: 0, size: 2 },
  FIRMWARE_VERSION: { addr: 6, size: 1 },
  ID: { addr: 7, size: 1 },
  BAUD_RATE: { addr: 8, size: 1 },
  DRIVE_MODE: { addr: 10, size: 1 },
  OPERATING_MODE: { addr: 11, size: 1 },
  HOMING_OFFSET: { addr: 20, size: 4 },
  MOVING_THRESHOLD: { addr: 24, size: 4 },
  TEMP_LIMIT: { addr: 31, size: 1 },
  MAX_VOLTAGE_LIMIT: { addr: 32, size: 2 },
  MIN_VOLTAGE_LIMIT: { addr: 34, size: 2 },
  PWM_LIMIT: { addr: 36, size: 2 },
  CURRENT_LIMIT: { addr: 38, size: 2 },
  ACCELERATION_LIMIT: { addr: 40, size: 4 },
  VELOCITY_LIMIT: { addr: 44, size: 4 },
  MAX_POSITION_LIMIT: { addr: 48, size: 4 },
  MIN_POSITION_LIMIT: { addr: 52, size: 4 },
  SHUTDOWN: { addr: 63, size: 1 },
  TORQUE_ENABLE: { addr: 64, size: 1 },
  LED: { addr: 65, size: 1 },
  STATUS_RETURN_LEVEL: { addr: 68, size: 1 },
  HARDWARE_ERROR_STATUS: { addr: 70, size: 1 },
  VELOCITY_I_GAIN: { addr: 76, size: 2 },
  VELOCITY_P_GAIN: { addr: 78, size: 2 },
  POSITION_D_GAIN: { addr: 80, size: 2 },
  POSITION_I_GAIN: { addr: 82, size: 2 },
  POSITION_P_GAIN: { addr: 84, size: 2 },
  GOAL_PWM: { addr: 100, size: 2 },
  GOAL_CURRENT: { addr: 102, size: 2 },
  GOAL_VELOCITY: { addr: 104, size: 4 },
  PROFILE_ACCELERATION: { addr: 108, size: 4 },
  PROFILE_VELOCITY: { addr: 112, size: 4 },
  GOAL_POSITION: { addr: 116, size: 4 },
  REALTIME_TICK: { addr: 120, size: 2 },
  MOVING: { addr: 122, size: 1 },
  MOVING_STATUS: { addr: 123, size: 1 },
  PRESENT_PWM: { addr: 124, size: 2 },
  PRESENT_CURRENT: { addr: 126, size: 2 },
  PRESENT_VELOCITY: { addr: 128, size: 4 },
  PRESENT_POSITION: { addr: 132, size: 4 },
  VELOCITY_TRAJECTORY: { addr: 136, size: 4 },
  POSITION_TRAJECTORY: { addr: 140, size: 4 },
  PRESENT_INPUT_VOLTAGE: { addr: 144, size: 2 },
  PRESENT_TEMPERATURE: { addr: 146, size: 1 },
};

/**
 * Protocol 2.0 CRC Calculation (CRC-16-IBM with polynomial 0x8005)
 */
function calculateCRC2(dataArray, dataLen) {
  let crc = 0;
  for (let j = 0; j < dataLen; j++) {
    const i = ((crc >> 8) ^ dataArray[j]) & 0xff;
    crc = ((crc << 8) ^ CRC16_TABLE[i]) & 0xffff;
  }
  return crc;
}

/**
 * Protocol 1.0 Checksum Calculation (~(Sum of bytes from ID to Last Param))
 */
function calculateChecksum1(packetArray) {
  let sum = 0;
  for (let i = 2; i < packetArray.length; i++) {
    sum += packetArray[i];
  }
  return (~sum) & 0xff;
}

/**
 * Build Protocol 2.0 Packet
 */
function buildPacketP2(id, instruction, params = []) {
  const paramLength = params.length;
  const length = paramLength + 3; // Instruction (1) + CRC_L (1) + CRC_H (1)
  const lengthL = length & 0xff;
  const lengthH = (length >> 8) & 0xff;

  const packet = [
    0xff, 0xff, 0xfd, 0x00,
    id & 0xff,
    lengthL,
    lengthH,
    instruction & 0xff,
    ...params
  ];

  const crc = calculateCRC2(packet, packet.length);
  packet.push(crc & 0xff);
  packet.push((crc >> 8) & 0xff);

  return packet;
}

/**
 * Build Protocol 1.0 Packet
 */
function buildPacketP1(id, instruction, params = []) {
  const length = params.length + 2; // Instruction (1) + Checksum (1)
  const packet = [
    0xff, 0xff,
    id & 0xff,
    length & 0xff,
    instruction & 0xff,
    ...params
  ];

  const checksum = calculateChecksum1(packet);
  packet.push(checksum);
  return packet;
}

/**
 * Byte conversions
 */
function intToBytes(value, byteCount, signed = false) {
  const bytes = [];
  let val = Math.round(value);
  if (signed && val < 0) {
    val = (1 << (8 * byteCount)) + val;
  }
  for (let i = 0; i < byteCount; i++) {
    bytes.push((val >> (8 * i)) & 0xff);
  }
  return bytes;
}

function bytesToInt(byteArray, signed = false) {
  let val = 0;
  const len = byteArray.length;
  for (let i = 0; i < len; i++) {
    val |= (byteArray[i] << (8 * i));
  }
  if (signed && (byteArray[len - 1] & 0x80)) {
    val -= (1 << (8 * len));
  }
  return val;
}

// Export object for browser globals
window.DXL = {
  INST: DXL_INST,
  MODELS: DXL_MODELS,
  P2_CTRL: DXL_P2_CTRL,
  calculateCRC2,
  calculateChecksum1,
  buildPacketP2,
  buildPacketP1,
  intToBytes,
  bytesToInt
};
