import serial
import time
import serial.tools.list_ports
from typing import List, Optional, Tuple
import logging

# 配置日志系统
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)


class PaxiniTac():
    def __init__(self):
        # 扫描可用串口
        self.logger = logging.getLogger(__name__)
        self.logger.info("扫描可用串口...")
        available_ports = list(serial.tools.list_ports.comports())
        
        if not available_ports:
            self.logger.error("未找到可用串口，请检查硬件连接")
            print("未找到可用串口，请检查硬件连接")
            return
        
        # 显示可用串口信息
        print("可用串口列表:")
        for n, port in enumerate(available_ports):
            print(f" {n} - {port.device} - {port.description}")
        
        # port_num = int(input("Input the port number to connect: "))
        port_num = 32
        # try to connect to assigned port
        try:
            self.ser = serial.Serial(available_ports[port_num].device, 460800, timeout=1)
            self.logger.info(f"成功连接到串口: {self.ser.name}")
            
            if not self.get_ser_response(self.ser, "get_version"):
                self.logger.error("获取设备版本信息失败")
                return False

            self.get_ser_response(self.ser,"set_mode")

            self.get_ser_response(self.ser,"get_mode")

        except serial.SerialException as e:
            print(f"串口连接失败: {e}")
            self.logger.error(f"串口连接异常: {e}")
        except Exception as e:
            print(f"程序启动错误: {e}")
            self.logger.error(f"程序启动异常: {e}")
    
    def send_hex_data(self,ser: serial.Serial, hex_data: str) -> bool:
        """
        向串口发送16进制数据，包含详细的错误处理
        
        参数:
            ser: 已打开的串口对象
            hex_data: 要发送的16进制字符串，如"55 AA 7B 7B"
            
        返回:
            发送是否成功
        """
        try:
            # 转换16进制字符串为字节数据
            data_bytes = bytes.fromhex(hex_data)
            # 发送数据并记录发送的字节数
            bytes_sent = ser.write(data_bytes)
            self.logger.debug(f"成功发送 {bytes_sent} 字节数据: {hex_data}")
            return True
        except serial.SerialException as e:
            self.logger.error(f"串口发送错误（硬件层面）: {e}")
            return False
        except ValueError as e:
            self.logger.error(f"16进制转换错误: {e}，数据: {hex_data}")
            return False
        except Exception as e:
            self.logger.error(f"发送数据时发生未知错误: {e}")
            return False
    
    def read_serial_response(self,ser: serial.Serial, timeout: float = 0.1) -> Optional[bytes]:
        """
        从串口读取响应数据，包含超时处理和数据有效性检查
        
        参数:
            ser: 已打开的串口对象
            timeout: 读取超时时间（秒）
            
        返回:
            读取到的字节数据，读取失败或无数据时返回None
        """
        try:
            # 等待数据到达的小延迟，确保数据接收完整
            time.sleep(0.002)
            # 读取所有可用数据
            response = ser.read_all()
            
            if response:
                self.logger.debug(f"接收到 {len(response)} 字节响应数据")
                return response
            return None
        except serial.SerialException as e:
            self.logger.error(f"串口读取错误（硬件层面）: {e}")
            return None
        except Exception as e:
            self.logger.error(f"读取数据时发生未知错误: {e}")
            return None
        
    def calculate_lrc(self,data: bytes) -> int:
        """
        计算LRC（纵向冗余校验）校验码
        
        参数:
            data: 用于计算校验码的字节数据
            
        返回:
            LRC校验码（0-255）
            
        算法说明:
            1. 对所有字节进行累加
            2. 对累加结果取反
            3. 取反结果加1
            4. 保留低8位
        """
        lrc = 0
        for byte in data:
            lrc = (lrc + byte) & 0xFF  # 累加并保持8位无符号整数
        lrc = ((~lrc) + 1) & 0xFF  # 取反加一，得到补码校验值
        return lrc

    def get_ser_response(self,ser: serial.Serial, command: str, data_length: int = 0) -> Optional[str]:
        """
        向模组控制盒发送命令并获取响应，包含协议解析逻辑
        
        参数:
            ser: 已打开的串口对象
            command: 命令类型，如"get_version"
            data_length: 对于数据获取命令，指定请求的数据长度（字节）
            
        返回:
            解析后的响应数据，失败时返回None
        """
        # 命令协议配置，分离数据与逻辑
        command_config = {
            "get_version": {
                "body": "0E 00 60 A0 01 00 00",
                "sleep": 1.0,
                "parse": "ascii"
            },
            "recalibration": {
                "body": "0E 00 70 B0 02 02 00 03 01",
                "sleep": 1.0,
                "parse": "text"
            },
            "set_mode": {
                "body": "0E 00 70 C0 0C 01 00 06",
                "sleep": 2.0,
                "parse": ""
            },
            "get_mode": {
                "body": "0E 00 70 C0 0D 00 00 B5",
                "sleep": 1.0,
                "parse": "text"
            },
            "choose_port1": {
                "body": "0E 00 70 B1 0A 01 00 02",
                "sleep": 0.0,
                "parse": ""
            },
            "choose_port2": {
                "body": "0E 00 70 B1 0A 01 00 08",
                "sleep": 0.0,
                "parse": ""
            },
            "choose_port3": {
                "body": "0E 00 70 B1 0A 01 00 0E",
                "sleep": 0.0,
                "parse": ""
            },
            "choose_port4": {
                "body": "0E 00 70 B1 0A 01 00 14",
                "sleep": 0.0,
                "parse": ""
            },
            "get_data": {
                "body": "0E 00 70 C0 06 05 00 7B 0E 04",
                "sleep": 0.0,
                "parse": "hex"
            },
            "get_resultant_data": {
                "body": "0E 00 70 C0 06 05 00 7B F0 03",
                "sleep": 0.0,
                "parse": "force"
            }
        }
        
        # 检查命令是否存在
        if command not in command_config:
            self.logger.error(f"未知命令: {command}")
            return None
        
        config = command_config[command]
        body = config["body"]
        sleep_time = config["sleep"]
        parse_type = config["parse"]
        frame_head = "55 AA 7B 7B"
        frame_tail = "55 AA 7D 7D"
        
        # 处理数据获取命令的长度参数
        if command in ["get_data", "get_resultant_data"]:
            if data_length <= 0:
                self.logger.error(f"错误：{command} 命令需要指定有效数据长度")
                return None
                
            # 将长度参数转换为小端字节序
            length_bytes = data_length.to_bytes(2, byteorder='little')
            body = f"{body}{length_bytes.hex()}"
            # self.logger.info(f"发送{command}请求，数据长度: {data_length} 字节，命令: {body}")

        # 计算LRC校验码
        try:
            data_bytes = bytes.fromhex(body)
            lrc = self.calculate_lrc(data_bytes)
        except ValueError as e:
            self.logger.error(f"LRC计算错误: {e}，数据: {body}")
            return None
        
        # 构建完整的发送帧
        full_frame = f"{frame_head}{body}{lrc:02X}{frame_tail}"
        
        # 发送请求
        if not self.send_hex_data(ser, full_frame):
            return None
        
        # 等待响应
        time.sleep(sleep_time)
        
        # 读取响应
        response = self.read_serial_response(ser)
        if not response:
            self.logger.warning("未收到设备响应")
            return None
        
        # 解析响应数据
        if len(response) >= 16:
            # 验证帧头和帧尾
            if response[0:4] == bytes.fromhex(frame_head) and response[-4:] == bytes.fromhex(frame_tail):
                error_code = response[9]
                if error_code == 0x00:
                    # 解析数据长度（小端字节序）
                    length_bytes = response[10:12]
                    data_length = int.from_bytes(length_bytes, byteorder='little')
                    
                    # 提取数据域
                    data_start = 12
                    data_end = data_start + data_length
                    response_data = response[data_start:data_end] if data_end <= len(response) else response[data_start:]
                    
                    if not response_data:
                        self.logger.warning("响应数据域为空")
                        return None
                    
                    # 根据解析类型处理数据
                    if parse_type == "ascii":
                        try:
                            data_str = response_data.decode('ascii')
                            # self.logger.info(f"{command} 响应: {data_str}")
                            return data_str
                        except UnicodeDecodeError:
                            self.logger.warning(f"ASCII解码失败，原始数据: {response_data.hex()}")
                    elif parse_type == "text":
                        return response_data.hex()
                    elif parse_type == "hex":
                        # self.logger.info(f"测点数据: {response_data.hex()}")
                        return response_data.hex()
                    elif parse_type == "force":
                        # self.logger.info(f"合力(FxFyFz)数据: {response_data.hex()}")
                        return response_data.hex()
                    else:
                        self.logger.info(f"{command} 执行成功")
                else:
                    self.logger.warning(f"设备返回错误码: {error_code:02X}")
            else:
                self.logger.warning("响应帧格式错误，帧头或帧尾不匹配")
        else:
            self.logger.warning(f"响应数据长度不足，无法解析（实际长度: {len(response)} 字节）")
        
        return None

    # def initialize_device(self,ser: serial.Serial) -> bool:
    #     """
    #     初始化设备，包含完整的初始化流程和错误处理
        
    #     参数:
    #         ser: 已打开的串口对象
            
    #     返回:
    #         初始化是否成功
    #     """
        
    #     try:
    #         self.logger.info("开始设备初始化...")

    #         # 按顺序执行初始化命令
    #         if not self.get_ser_response(ser, "get_version"):
    #             self.logger.error("获取设备版本信息失败")
    #             return False

    #         self.get_ser_response(ser,"set_mode")

    #         self.get_ser_response(ser,"get_mode")


    #         self.logger.info("设备初始化完成")
    #         return True
    #     except Exception as e:
    #         self.logger.error(f"设备初始化过程中发生异常: {e}")
    #         return False
        
    def convert_hex_to_sensor_data(self,valid_data: str, axis_types: List[str]) -> Optional[List[int]]:
        """
        将16进制数据按轴类型转换为传感器数值（支持有符号/无符号转换）
        
        参数:
            valid_data: 裁切后的16进制数据字符串
            axis_types: 轴类型列表，每个元素为 "signed"（有符号）或 "unsigned"（无符号）
            
        返回:
            转换后的数值列表，失败时返回None
        """
        try:
            bytes_data = bytes.fromhex(valid_data)
            result = []
            
            # 检查数据长度与轴类型数量是否匹配
            if len(bytes_data) != len(axis_types):
                self.logger.error(f"数据长度与轴类型数量不匹配，数据长度: {len(bytes_data)}, 轴类型数量: {len(axis_types)}")
                return None
            
            # 按轴类型转换数据
            for byte, axis_type in zip(bytes_data, axis_types):
                if axis_type == "signed":
                    # 有符号转换（Fx/Fy轴）
                    value = byte if byte < 128 else byte - 256
                elif axis_type == "unsigned":
                    # 无符号转换（Fz轴）
                    value = byte
                else:
                    self.logger.warning(f"未知轴类型: {axis_type}，使用默认有符号转换")
                    value = byte if byte < 128 else byte - 256
                result.append(value)
                
            return result
        
        except ValueError as e:
            self.logger.error(f"十六进制数据解析错误: {e}，数据: {valid_data}")
            return None
        except Exception as e:
            self.logger.error(f"数据转换异常: {e}")
            return None


    def get_raw_sensor_data(self,ser: serial.Serial, command: str, request_length: int, trim_length: int) -> Optional[List[int]]:
        """
        通用函数：获取传感器原始数据并进行基础处理
        
        参数:
            ser: 已打开的串口对象
            command: 要发送的命令类型 ("get_data" 或 "get_resultant_data")
            request_length: 请求的数据长度（字节）
            trim_length: 需要裁切的头部长度（字节）
            
        返回:
            解析后的有符号单字节数据列表，失败时返回None
        """
        # 检查命令有效性
        if command not in ["get_data", "get_resultant_data"]:
            self.logger.error(f"不支持的命令类型: {command}")
            return None
        
        # 调用底层接口获取数据
        data = self.get_ser_response(ser, command, request_length)
        if not data:
            self.logger.error(f"获取{command}传感器原始数据失败")
            return None
        
        # 数据有效性验证
        if len(data) <= trim_length * 2:
            self.logger.error(f"数据长度不足，实际 {len(data)} 个字符，至少需要 {trim_length*2} 个字符")
            return None
        
        # 裁切头部数据
        valid_data = data[trim_length*2:]  # 转换为字符裁切
        
        # 验证剩余数据长度
        expected_char_length = request_length * 2
        if len(valid_data) != expected_char_length:
            self.logger.error(f"裁切后数据长度不符，实际 {len(valid_data)} 个字符，期望 {expected_char_length} 个字符")
            return None
        
        # self.logger.info(f"成功获取 {request_length} 字节{command}传感器原始数据")
        
        
        # 根据命令类型选择不同的解析方式,转换为有符号单字节整数列表
        if command == "get_resultant_data":
            # Fx/Fy为有符号，Fz为无符号
            axis_types = ["signed", "signed", "unsigned"]
            return self.convert_hex_to_sensor_data(valid_data, axis_types)
        
        elif command == "get_data":
            # 处理120个点的三维数据
            points = []
            bytes_data = bytes.fromhex(valid_data)
            
            # 验证数据长度是否符合120个点（360字节）
            if len(bytes_data) != request_length:
                self.logger.error(f"数据长度不符，{request_length}字节，实际{len(bytes_data)}字节")
                return None
                
            for i in range(0, request_length, 3):
                # 每3个字节为一组(X,Y,Z)
                x = bytes_data[i]
                y = bytes_data[i+1]
                z = bytes_data[i+2]
                
                # 转换为有符号整数（X/Y）和无符号整数（Z）,原始数据是无符号整数，所以无需转换
                x_signed = x if x < 128 else x - 256
                y_signed = y if y < 128 else y - 256
                
                points.append([x_signed, y_signed, z])
                
            return points
        
        return None
            
        
