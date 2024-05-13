import serial
import time

PKT_HEADER0 = 0
PKT_HEADER1 = 1
PKT_ID = 2
PKT_LENGTH = 3
PKT_INSTRUCTION = 4
PKT_ERROR = 4
PKT_PARAMETER0 = 5

INST_WRITE = 3

def generate_instruction_packet(dynamixel_id, goal_position):
    # Dynamixel 프로토콜 1에 따라 명령 패킷을 생성
    instruction_packet = b'\xFF\xFF'  # 시작 바이트
    instruction_packet += bytes([dynamixel_id])  # Dynamixel ID
    instruction_packet += b'\x07'  # 길이: ID 및 명령 코드를 제외한 데이터 길이
    instruction_packet += b'\x03'  # 명령 코드: write
    instruction_packet += b'\x74'  # Goal Position Address
    instruction_packet += goal_position.to_bytes(2, byteorder='little')  # 데이터: 목표 위치
    instruction_packet += bytes([calculate_checksum(instruction_packet[2:])])  # 체크섬 계산 및 추가
    return instruction_packet

def calculate_checksum(packet):
    # 체크섬 계산
    checksum = sum(packet) & 0xFF
    return (~checksum) & 0xFF

def writeTxOnly(dxl_id, address, length, data):
    txpacket = [0] * (length + 7)

    txpacket[PKT_HEADER0] = 0xFF
    txpacket[PKT_HEADER1] = 0xFF
    txpacket[PKT_ID] = dxl_id
    txpacket[PKT_LENGTH] = length + 3
    txpacket[PKT_INSTRUCTION] = INST_WRITE
    txpacket[PKT_PARAMETER0] = address

    txpacket[PKT_PARAMETER0 + 1: PKT_PARAMETER0 + 1 + length] = data[0: length]

    checksum = 0
    total_packet_length = txpacket[PKT_LENGTH] + 4
    
    for idx in range(2, total_packet_length - 1):  # except header, checksum
        checksum += txpacket[idx]

    txpacket[total_packet_length - 1] = ~checksum & 0xFF

    return txpacket

# 테스트용 명령 패킷 생성
# print(bytes(writeTxOnly(1, 116, 4, [0, 0 ,0 , 0])))

# #-=-------------------------------- 10진수를 16진수로 변환
# hex_value = hex(1000)

# # '0x'를 제거하고 앞에 0을 채워서 4자리로 만듭니다.
# hex_value = hex_value[2:].zfill(4)
# print(hex_value)


# # 16진수를 바이트로 변환하여 리스트에 저장
# byte_list = [bytes([int(hex_value[i:i+2], 16)]) for i in range(0, len(hex_value), 2)]

# tmp = byte_list[0]
# byte_list[0] = byte_list[1]
# byte_list[1] = tmp

# byte_list = byte_list + [b'\x00', b'\x00']
# byte_list = [int(byte.hex(), 16) for byte in byte_list]

# # 결과 출력
# print(byte_list)

def generate_read_packet(instruction, address, length):
        packet = [0] * (length + 4)

        packet[PKT_HEADER0] = 0xFF
        packet[PKT_HEADER1] = 0xFF
        packet[PKT_ID] = 1
        packet[PKT_LENGTH] = length
        packet[PKT_INSTRUCTION] = instruction
        packet[PKT_PARAMETER0] = address
        packet[PKT_PARAMETER0 + 1] = length

        checksum = 0
        total_packet_length = packet[PKT_LENGTH] + 4
        
        for idx in range(2, total_packet_length - 1):  # except header, checksum
            checksum += packet[idx]

        packet[total_packet_length - 1] = ~checksum & 0xFF

        return packet

print(generate_read_packet(2, 132, 4))