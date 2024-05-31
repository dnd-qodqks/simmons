import rclpy
from rclpy.node import Node
import serial
import threading
import time
import os
from dynamixel_sdk import *
# from result_msgs.msg import Force

class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')

        # self.sub_force_info_ = self.create_subscription(
        #     Force,
        #     'force_info',
        #     self.force_info_callback,
        #     10)
        
        # self.sub_force_info_  # prevent unused variable warning

        self.DEBUG = False

        # Control table address
        self.ADDR_MX_TORQUE_ENABLE       = 64                              # Control table address is different in Dynamixel model
        self.ADDR_MX_LED_ENABLE          = 65                            
        self.ADDR_MX_GOAL_POSITION       = 116
        self.ADDR_MX_PRESENT_POSITION    = 132
        self.ADDR_PROFILE_ACCELERATION   = 108
        self.ADDR_PROFILE_VELOCITY       = 112
        self.ADDR_MOVING                 = 46

        # Protocol version
        self.PROTOCOL_VERSION            = 1                               # See which protocol version is used in the Dynamixel

        # Default setting
        self.DXL_ID                      = [0, 1, 2, 3]                          # Dynamixel ID list
        self.BAUDRATE                    = 115200
        self.DEVICENAME                  = "/dev/ttyUSB0"                  # Check which port is being used on your controller
                                                                           # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

        self.TORQUE_ENABLE               = 1                               # Value for enabling the torque
        self.LED_ENABLE                  = 1                               # Value for enabling the LED
        self.PROFILE_ACCELERATION        = 10
        self.PROFILE_VELOCITY            = 210
        self.TORQUE_DISABLE              = 0                               # Value for disabling the torque
        self.LED_DISABLE                 = 0                               # Value for enabling the LED
        self.DXL_MOVING_STATUS_THRESHOLD = 20                              # Dynamixel moving status threshold
        self.COMM_SUCCESS                = 0                               # Communication Success result value
        self.LEN_MX_GOAL_POSITION        = 4
        self.LEN_MX_PRESENT_POSITION     = 4

        self.portHandler_ = PortHandler(self.DEVICENAME)
        self.portHandler_.setBaudRate(self.BAUDRATE)
        self.packetHandler_ = PacketHandler(self.PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler_, 
                                             self.packetHandler_, 
                                             self.ADDR_MX_GOAL_POSITION, 
                                             self.LEN_MX_GOAL_POSITION)
        self.groupBulkRead = GroupBulkRead(self.portHandler_, 
                                           self.packetHandler_)

        self.dxl_goal_position      = [0 for _ in range(len(self.DXL_ID))]     # Goal position
        self.dxl_present_position   = [0 for _ in range(len(self.DXL_ID))]     # Present position
        self.dxl_moving             = [0 for _ in range(len(self.DXL_ID))] # Moving

        # Open port
        if not (self.portHandler_.openPort()):
            self.get_logger().info("포트를 열 수 없습니다!")
        
        # Init
        self.set_dxl_torque(self.TORQUE_ENABLE)
        self.set_dxl_led(self.LED_ENABLE)
        self.set_dxl_profile(self.PROFILE_ACCELERATION, self.PROFILE_VELOCITY)
        self.set_multi_goal_position([0, 0, 0, 0])
        time.sleep(1.5)

        # Test
        self.test_sync_bulk()

    def force_info_callback(self, msg):
        # 각 모터의 각도 계산
        
        angles = [0, 1, 2, 3]        # 각 모터의 각도 저장
        goal_postions = angles2goal_positions(angles) # 각도를 골 포지션으로 변환
        
        set_multi_goal_position(goal_postions)
        
        while True:
            if self.dxl_moving[0] == 0 and \
               self.dxl_moving[1] == 0 and \
               self.dxl_moving[2] == 0 and \
               self.dxl_moving[3] == 0:
                   break

    def angles2goal_positions(self, angles):
        pass
        # return goal_postions

    def set_dxl_torque(self, signal):
        for id in self.DXL_ID:
            dxl_comm_result, dxl_error = self.packetHandler_.write1ByteTxRx(self.portHandler_, id, self.ADDR_MX_TORQUE_ENABLE, signal)
            if dxl_comm_result != self.COMM_SUCCESS:
                self.get_logger().info(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().info(f"{self.packetHandler_.getRxPacketError(dxl_error)}")
            else:
                if signal == self.TORQUE_ENABLE:
                    self.get_logger().info(f"Dynamixel#{id} has been successfully torque enable")
                else:
                    self.get_logger().info(f"Dynamixel#{id} has been successfully torque disable")

    def set_dxl_led(self, signal):
        for id in self.DXL_ID:
            dxl_comm_result, dxl_error = self.packetHandler_.write1ByteTxRx(self.portHandler_, id, self.ADDR_MX_LED_ENABLE, signal)
            if dxl_comm_result != self.COMM_SUCCESS:
                self.get_logger().info(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().info(f"{self.packetHandler_.getRxPacketError(dxl_error)}")
            else:
                if signal == self.LED_ENABLE:
                    self.get_logger().info(f"Dynamixel#{id} has been successfully led enable")
                else:
                    self.get_logger().info(f"Dynamixel#{id} has been successfully led disable")

    def set_dxl_profile(self, acc, vel):
        for id in self.DXL_ID:
            dxl_comm_result, dxl_error = self.packetHandler_.write4ByteTxRx(self.portHandler_, id, self.ADDR_PROFILE_ACCELERATION, acc)

            if dxl_comm_result != self.COMM_SUCCESS:
                self.get_logger().info(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().info(f"{self.packetHandler_.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().info(f"Dynamixel#{id} has been successfully profile acceleration: {acc}")

            dxl_comm_result, dxl_error = self.packetHandler_.write4ByteTxRx(self.portHandler_, id, self.ADDR_PROFILE_VELOCITY, vel)

            if dxl_comm_result != self.COMM_SUCCESS:
                self.get_logger().info(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().info(f"{self.packetHandler_.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().info(f"Dynamixel#{id} has been successfully profile velocity: {vel}")

    def set_goal_position(self, id, change_goal_position):
        start_time = time.time()

        # Write goal position
        result, error = self.packetHandler_.write4ByteTxRx(self.portHandler_, id, self.ADDR_MX_GOAL_POSITION, change_goal_position)
        if result == self.COMM_SUCCESS:
            self.get_logger().info("successfully goal position")
        else:
            self.get_logger().info(f"Fail goal position, Error: {self.packetHandler_.getRxPacketError(error)}")

        while 1:
            # Read present position
            self.dxl_present_position = self.packetHandler_.read2ByteTxRx(self.portHandler_, id, self.ADDR_MX_PRESENT_POSITION)
            if self.dxl_present_position[0] == 65535: self.dxl_present_position[0] = 0

            # self.get_logger().info(f"[ID:{id}] GoalPos:{change_goal_position}  PresPos:{self.dxl_present_position[0]}")

            position_error = abs(change_goal_position - self.dxl_present_position[0])
            # self.get_logger().info(f"position_error: {position_error}")

            if position_error <= self.DXL_MOVING_STATUS_THRESHOLD:
                break
        end_time = time.time()
        time_taken = end_time - start_time
        self.get_logger().info(f"successfully goal position: {change_goal_position}, time_take: {time_taken:.5f} [s]")

    def set_multi_goal_position(self, goal_postions):

        if self.DEBUG:
            start_time = time.time()

        # Allocate goal position value into byte array
        for id in self.DXL_ID:
            self.get_logger().debug(f"ID: {id}")
            self.get_logger().info(f'goal_postions: {goal_postions}')
            self.dxl_goal_position[id] = goal_postions[id]

            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(self.dxl_goal_position[id])), 
                                   DXL_HIBYTE(DXL_LOWORD(self.dxl_goal_position[id])), 
                                   DXL_LOBYTE(DXL_HIWORD(self.dxl_goal_position[id])), 
                                   DXL_HIBYTE(DXL_HIWORD(self.dxl_goal_position[id]))]

            # Add Dynamixel#ID goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self.groupSyncWrite.addParam(self.DXL_ID[id], param_goal_position)
            if dxl_addparam_result != True:
                self.get_logger().info(f"[ID:{self.DXL_ID[id]:03d}] groupSyncWrite addparam failed")

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != self.COMM_SUCCESS:
            self.get_logger().debug(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

        if self.DEBUG:
            end_time = time.time()
            time_taken = end_time - start_time
            self.get_logger().info(f"successfully goal position, time_take: {time_taken:.5f} [s]")
    
    def read_multi_present_position(self):
        
        if self.DEBUG:
            start_time = time.time()

        for id in self.DXL_ID:
            # Add parameter storage for Dynamixel#ID present position
            dxl_addparam_result = self.groupBulkRead.addParam(id, self.ADDR_MX_PRESENT_POSITION, self.LEN_MX_PRESENT_POSITION)
            if dxl_addparam_result != True:
                self.get_logger().info(f"[ID:{id:03d}] groupBulkRead addparam failed")

            # Bulkread present position
            dxl_comm_result = self.groupBulkRead.txRxPacket()
            if dxl_comm_result != self.COMM_SUCCESS:
                self.get_logger().info(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")

            # Check if groupbulkread data of Dynamixel#1 is available
            dxl_getdata_result = self.groupBulkRead.isAvailable(id, self.ADDR_MX_PRESENT_POSITION, self.LEN_MX_PRESENT_POSITION)
            if dxl_getdata_result != True:
                self.get_logger().info(f"[ID:{id:03d}] groupBulkRead getdata failed")

            # Get Dynamixel#1 present position value
            self.dxl_present_position[id] = self.groupBulkRead.getData(id, self.ADDR_MX_PRESENT_POSITION, self.LEN_MX_PRESENT_POSITION)

        self.groupBulkRead.clearParam()

        if self.DEBUG:
            end_time = time.time()
            time_taken = end_time - start_time
            self.get_logger().info(f"successfully read present position, time_take: {time_taken:.5f} [s]")
            self.get_logger().info(f"DXL present position: {self.dxl_present_position}")

    def test_sync_bulk(self):
        max_err = 0
        add_goal_postion = [0, 0, 0, 0]

        while True:
            self.set_multi_goal_position(add_goal_postion)

            while True:
                self.read_multi_present_position()

                self.get_logger().info(f"--- DXL goal position: {self.dxl_goal_position} ---")
                self.get_logger().info(f"--- DXL pres position: {self.dxl_present_position} ---")
                self.get_logger().info(f"--- DXL moving: {self.dxl_moving} ---")
                    
                self.check_moving_pram()
                
                if self.dxl_moving[0] == 0 and self.dxl_moving[1] == 0 and self.dxl_moving[2] == 0 and self.dxl_moving[3] == 0:
                    break

            for id in self.DXL_ID:
                add_goal_postion[id] += 100
            
            while True:
                cnt = 0
                
                for id in self.DXL_ID:
                    if add_goal_postion[id] >= 4095:
                        cnt += 1
                        add_goal_postion[id] += 100

                if cnt == 0: break

    def check_moving_pram(self):
        for id in self.DXL_ID:
            # Add parameter storage for Dynamixel#ID present position
            dxl_addparam_result = self.groupBulkRead.addParam(id, self.ADDR_MOVING, 1)
            if dxl_addparam_result != True:
                self.get_logger().info(f"[ID:{id:03d}] [Moving] groupBulkRead addparam failed")

            # Bulkread present position
            dxl_comm_result = self.groupBulkRead.txRxPacket()
            if dxl_comm_result != self.COMM_SUCCESS:
                self.get_logger().info(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")

            # Check if groupbulkread data of Dynamixel#1 is available
            dxl_getdata_result = self.groupBulkRead.isAvailable(id, self.ADDR_MOVING, 1)
            if dxl_getdata_result != True:
                self.get_logger().info(f"[ID:{id:03d}] [Moving] groupBulkRead getdata failed")

            # Get Dynamixel#1 present position value
            self.dxl_moving[id] = self.groupBulkRead.getData(id, self.ADDR_MOVING, 1)

        self.groupBulkRead.clearParam()

    def __del__(self):
        self.set_dxl_torque(self.TORQUE_DISABLE)
        self.set_dxl_led(self.LED_DISABLE)
        self.portHandler_.closePort()

def main(args=None):
    rclpy.init(args=args)
    dynamixel_controller = DynamixelController()
    rclpy.spin(dynamixel_controller)
    dynamixel_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
