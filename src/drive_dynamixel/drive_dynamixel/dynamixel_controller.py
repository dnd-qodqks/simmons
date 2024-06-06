import rclpy
from rclpy.node import Node
import math
from dynamixel_sdk import *
from result_msgs.msg import Force
from result_msgs.msg import Person

class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')

        self.sub_force_info_ = self.create_subscription(
            Force,
            'force_info',
            self.force_info_callback,
            10)
        
        self.person_info_sub_ = self.create_subscription(
            Person,
            'person_info',
            self.person_info_callback,
            10)
        
        # prevent unused variable warning
        self.sub_force_info_ 
        self.person_info_sub_
        
        self.DEBUG = False

        # Control table address
        self.ADDR_MX_TORQUE_ENABLE       = 64                              # Control table address is different in Dynamixel model
        self.ADDR_MX_LED_ENABLE          = 65                            
        self.ADDR_MX_GOAL_POSITION       = 116
        self.ADDR_MX_PRESENT_POSITION    = 132
        self.ADDR_PROFILE_ACCELERATION   = 108
        self.ADDR_PROFILE_VELOCITY       = 112
        self.ADDR_MOVING                 = 122

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
        self.OFFSET                      = [-528, -78, 107, -32]

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
        self.dxl_moving             = [0 for _ in range(len(self.DXL_ID))]     # Moving

        # Open port
        if not (self.portHandler_.openPort()):
            self.get_logger().info("포트를 열 수 없습니다!")
        
        # Init
        self.set_dxl_torque(self.TORQUE_ENABLE)
        self.set_dxl_led(self.LED_ENABLE)
        self.set_dxl_profile(self.PROFILE_ACCELERATION, self.PROFILE_VELOCITY)
        self.set_multi_goal_position([2048, 2048, 2048, 2048])
        time.sleep(1.0)

        # Test
        # self.test_sync_bulk()

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

    def person_info_callback(self, msg):
        length_factor = 1000.0
        length = msg.length * length_factor
        degree_factor = 2.0
        degree = (msg.degree - 90.0) * degree_factor
        
        steering_info = self.steering_kinematics(length, degree)
        
        goal_postions = []
        for radian in steering_info[:-1]:
            goal_postions.append(math.fabs(int((self.rad2deg(radian) / 90.0) * 2048)))
        
        self.set_multi_goal_position(goal_postions)
        
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
        # Allocate goal position value into byte array
        for id in self.DXL_ID:
            self.get_logger().info(f'ID: {id}, goal_postions: {goal_postions}')
            self.dxl_goal_position[id] = goal_postions[id] + self.OFFSET[id]

            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(self.dxl_goal_position[id])), 
                                   DXL_HIBYTE(DXL_LOWORD(self.dxl_goal_position[id])), 
                                   DXL_LOBYTE(DXL_HIWORD(self.dxl_goal_position[id])), 
                                   DXL_HIBYTE(DXL_HIWORD(self.dxl_goal_position[id]))]

            # Add Dynamixel#ID goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self.groupSyncWrite.addParam(self.DXL_ID[id], param_goal_position)
            if dxl_addparam_result != True:
                self.get_logger().warning(f"[ID:{self.DXL_ID[id]:03d}] groupSyncWrite addparam failed")

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != self.COMM_SUCCESS:
            self.get_logger().warning(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

        while True:
            self.read_multi_moving()            
            
            self.get_logger().debug(f'Moving: {self.dxl_moving}')
            
            if self.dxl_moving[0] == 0 and \
               self.dxl_moving[1] == 0 and \
               self.dxl_moving[2] == 0 and \
               self.dxl_moving[3] == 0:
                   break
    
    def read_multi_moving(self):
        for id in self.DXL_ID:
            dxl_addparam_result = self.groupBulkRead.addParam(id, self.ADDR_MOVING, 1)
            if dxl_addparam_result != True:
                self.get_logger().info(f"[ID:{id:03d}] groupBulkRead addparam failed")

            dxl_comm_result = self.groupBulkRead.txRxPacket()
            if dxl_comm_result != self.COMM_SUCCESS:
                self.get_logger().info(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")

            dxl_getdata_result = self.groupBulkRead.isAvailable(id, self.ADDR_MOVING, 1)
            if dxl_getdata_result != True:
                self.get_logger().info(f"[ID:{id:03d}] groupBulkRead getdata failed")

            self.dxl_moving[id] = self.groupBulkRead.getData(id, self.ADDR_MOVING, 1)

        self.groupBulkRead.clearParam()

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
        
    def deg2rad(self, deg):
        return math.pi*deg/180.0

    def rad2deg(self, rad):
        return 180.0*rad/math.pi

    def steering_kinematics(self, dist, angle):
        L = 490
        T = 490

        if (angle == 0.0):
            # rot_radius = dist/(0.000000000000001)
            rot_radius = dist/(1e-15)    
        else:
            rot_radius = dist/(2.0*math.sin(self.deg2rad(angle)))
            
        # print(f"rot_radius: {rot_radius} (same scale with distance)\n")

        bicycle_front = math.atan2(L/2.0, rot_radius)
        bicycle_back = -bicycle_front
        # print(f"rad2deg(bicycle_front): {self.rad2deg(bicycle_front)} (same scale with distance)\n")
        
        front_left = math.atan2(L/2.0, rot_radius - T/2.0) - math.pi/2.0
        front_right = math.atan2(L/2.0, rot_radius + T/2.0) - math.pi/2.0
        back_left = -front_left
        back_right = -front_right
        # center_radius = rot_radius
        
        steering_info = [front_left, front_right, back_left, back_right, center_radius]
        
        return steering_info

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
