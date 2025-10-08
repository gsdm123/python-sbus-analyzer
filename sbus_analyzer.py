#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SBUS协议分析器
集成了SBUS协议解析和数据记录功能
支持电平取反的硬件实现
"""

import serial
import serial.tools.list_ports
import time
import threading
import struct
import csv
import os
from datetime import datetime
import sys


class SBUSConstants:
    """SBUS协议相关常量"""
    START_BYTE = 0x0F
    END_BYTE = 0x00
    FRAME_LENGTH = 25
    CHANNELS = 16
    BAUD_RATE = 100000
    TIMEOUT = 0.1

    # SBUS数据范围
    SBUS_MIN = 172
    SBUS_MAX = 1811
    PWM_MIN = 1000
    PWM_MAX = 2000

    # 显示配置
    DISPLAY_COLUMNS = 4
    FRAME_TIMEOUT = 5.0
    MONITOR_INTERVAL = 0.1


class SBUSAnalyzer:
    def __init__(self):
        """初始化SBUS分析器"""
        # 串口连接状态
        self.serial_conn = None
        self.is_connected = False

        # 监控状态
        self.is_monitoring = False
        self.monitor_thread = None

        # 数据缓冲区
        self.buffer = bytearray()
        self.last_frame_time = 0
        self.frame_count = 0
        self.error_count = 0

        # 解析后的通道数据
        self.channels = [0] * SBUSConstants.CHANNELS
        self.digital_channels = [False] * 2  # CH17, CH18
        self.frame_lost = False
        self.failsafe = False

        # 数据记录功能
        self.is_logging = False
        self.logging_thread = None
        self.log_file = "sbus_data.csv"
        self.csv_writer = None
        self.csv_file = None

        # 串口设置
        self.port = 'COM3'  # 默认串口

    def _handle_error(self, operation, error, show_details=True):
        """
        统一的错误处理机制

        Args:
            operation: 操作名称
            error: 错误对象或错误消息
            show_details: 是否显示详细错误信息
        """
        error_msg = str(error) if hasattr(error, '__str__') else error

        if show_details:
            print(f"{operation}错误: {error_msg}")
        else:
            print(f"{operation}失败")

        # 记录错误计数
        self.error_count += 1

    def connect(self, port=None, baudrate=None, timeout=None):
        """连接到SBUS串口"""
        try:
            if self.is_connected:
                self.disconnect()

            # 使用默认值或传入的参数
            port = port or self.port
            baudrate = baudrate or SBUSConstants.BAUD_RATE
            timeout = timeout or SBUSConstants.TIMEOUT
 
            print(f"正在连接到 {port}，波特率: {baudrate}")
            self.serial_conn = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                parity=serial.PARITY_EVEN,  # SBUS使用偶校验
                stopbits=serial.STOPBITS_TWO,  # SBUS使用2个停止位
                bytesize=serial.EIGHTBITS
            )

            # 等待连接建立
            time.sleep(0.5)

            if self.serial_conn.is_open:
                self.is_connected = True
                self.port = port  # 更新当前端口
                print(f"成功连接到 {port}")
                print(f"波特率: {baudrate}")
                print(f"校验位: 偶校验")
                print(f"停止位: 2位")
                return True
            else:
                print(f"连接失败: {port}")
                return False

        except serial.SerialException as e:
            self._handle_error("串口连接", e)
            return False
        except Exception as e:
            self._handle_error("连接", e)
            return False

    def disconnect(self):
        """断开串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.stop_monitoring()
            self.stop_logging()
            self.serial_conn.close()
            self.is_connected = False
            print("SBUS串口连接已断开")
        else:
            print("当前没有活动的串口连接")

    def parse_sbus_frame(self, frame):
        """
        解析SBUS数据帧

        Args:
            frame: 25字节的SBUS帧数据

        Returns:
            bool: 解析成功返回True，失败返回False
        """
        if len(frame) != SBUSConstants.FRAME_LENGTH:
            return False

        # 检查起始和结束字节
        if frame[0] != SBUSConstants.START_BYTE or frame[-1] != SBUSConstants.END_BYTE:
            return False

        try:
            # 解析16个模拟通道（11位分辨率）
            channels = [0] * SBUSConstants.CHANNELS

            # 通道1-8
            channels[0] = ((frame[1] | frame[2] << 8) & 0x07FF)
            channels[1] = ((frame[2] >> 3 | frame[3] << 5) & 0x07FF)
            channels[2] = ((frame[3] >> 6 | frame[4] << 2 | frame[5] << 10) & 0x07FF)
            channels[3] = ((frame[5] >> 1 | frame[6] << 7) & 0x07FF)
            channels[4] = ((frame[6] >> 4 | frame[7] << 4) & 0x07FF)
            channels[5] = ((frame[7] >> 7 | frame[8] << 1 | frame[9] << 9) & 0x07FF)
            channels[6] = ((frame[9] >> 2 | frame[10] << 6) & 0x07FF)
            channels[7] = ((frame[10] >> 5 | frame[11] << 3) & 0x07FF)

            # 通道9-16
            channels[8] = ((frame[12] | frame[13] << 8) & 0x07FF)
            channels[9] = ((frame[13] >> 3 | frame[14] << 5) & 0x07FF)
            channels[10] = ((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FF)
            channels[11] = ((frame[16] >> 1 | frame[17] << 7) & 0x07FF)
            channels[12] = ((frame[17] >> 4 | frame[18] << 4) & 0x07FF)
            channels[13] = ((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FF)
            channels[14] = ((frame[20] >> 2 | frame[21] << 6) & 0x07FF)
            channels[15] = ((frame[21] >> 5 | frame[22] << 3) & 0x07FF)

            # 解析数字通道和状态位
            digital_byte = frame[23]
            self.digital_channels[0] = bool(digital_byte & 0x01)  # CH17
            self.digital_channels[1] = bool(digital_byte & 0x02)  # CH18
            self.frame_lost = bool(digital_byte & 0x04)
            self.failsafe = bool(digital_byte & 0x08)

            # 将11位值转换为PWM值
            for i in range(SBUSConstants.CHANNELS):
                if channels[i] == 0:
                    self.channels[i] = 0  # 无效通道
                else:
                    # SBUS范围：172-1811，对应PWM 1000-2000us
                    self.channels[i] = self._convert_sbus_to_pwm(channels[i])

            return True

        except Exception as e:
            self._handle_error("解析SBUS帧", e)
            return False

    def _convert_sbus_to_pwm(self, sbus_value):
        """
        将SBUS值转换为PWM值

        Args:
            sbus_value: SBUS原始值 (172-1811)

        Returns:
            int: PWM值 (1000-2000us)
        """
        if sbus_value < SBUSConstants.SBUS_MIN or sbus_value > SBUSConstants.SBUS_MAX:
            return 0

        # 线性映射：SBUS(172-1811) -> PWM(1000-2000)
        pwm_value = int((sbus_value - SBUSConstants.SBUS_MIN) * 
                       (SBUSConstants.PWM_MAX - SBUSConstants.PWM_MIN) / 
                       (SBUSConstants.SBUS_MAX - SBUSConstants.SBUS_MIN) + 
                       SBUSConstants.PWM_MIN)
        return pwm_value

    def start_monitoring(self):
        """开始监控SBUS数据"""
        if not self.is_connected:
            print("请先连接串口")
            return

        if self.is_monitoring:
            print("数据监控已在运行")
            return

        self.is_monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        print("开始监控SBUS数据...")

    def stop_monitoring(self):
        """停止监控SBUS数据"""
        if self.is_monitoring:
            self.is_monitoring = False
            # 等待监控线程结束
            if hasattr(self, 'monitor_thread') and self.monitor_thread:
                self.monitor_thread.join(timeout=1)
            print("停止监控SBUS数据")

    def _monitor_loop(self):
        """监控循环"""
        while self.is_monitoring and self.is_connected:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read_all()
                    self.buffer.extend(data)

                    # 处理缓冲区中的数据
                    self._process_buffer()

                time.sleep(SBUSConstants.MONITOR_INTERVAL)
            except Exception as e:
                self._handle_error("监控", e)
                break

    def _process_buffer(self):
        """处理数据缓冲区"""
        while len(self.buffer) >= SBUSConstants.FRAME_LENGTH:
            # 查找起始字节
            start_idx = -1
            for i in range(len(self.buffer) - SBUSConstants.FRAME_LENGTH + 1):
                if self.buffer[i] == SBUSConstants.START_BYTE:
                    start_idx = i
                    break

            if start_idx == -1:
                # 没有找到起始字节，清空缓冲区
                self.buffer.clear()
                break

            # 移除起始字节之前的数据
            if start_idx > 0:
                self.buffer = self.buffer[start_idx:]

            # 检查是否有完整帧
            if len(self.buffer) >= SBUSConstants.FRAME_LENGTH:
                frame = self.buffer[:SBUSConstants.FRAME_LENGTH]

                # 解析帧
                if self.parse_sbus_frame(frame):
                    self.frame_count += 1
                    self.last_frame_time = time.time()
                    self._display_frame_data()
                else:
                    self.error_count += 1
                # 移除已处理的数据
                self.buffer = self.buffer[SBUSConstants.FRAME_LENGTH:]
            else:
                break

    def _display_frame_data(self):
        """显示帧数据"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        print(f"\n[{timestamp}] SBUS帧 #{self.frame_count}")
        print("-" * 50)

        # 显示通道数据
        for i in range(SBUSConstants.CHANNELS):
            if self.channels[i] > 0:
                print(f"CH{i+1:2d}: {self.channels[i]:4d}us", end="  ")
                if (i + 1) % 4 == 0:
                    print()

        if SBUSConstants.CHANNELS % 4 != 0:
            print()

        # 显示数字通道和状态
        print(f"CH17: {'ON' if self.digital_channels[0] else 'OFF'}")
        print(f"CH18: {'ON' if self.digital_channels[1] else 'OFF'}")
        print(f"帧丢失: {'是' if self.frame_lost else '否'}")
        print(f"故障保护: {'是' if self.failsafe else '否'}")

        # 显示统计信息
        if self.error_count > 0:
            print(f"错误帧数: {self.error_count}")

    def start_logging(self, log_file="sbus_data.csv"):
        """开始记录数据 - 持续接收并记录最新的SBUS数据"""
        if not self.is_connected:
            print("请先连接串口")
            return

        if self.is_logging:
            print("数据记录已在运行")
            return

        self.log_file = log_file
        self._create_csv_file()

        # 启动记录线程
        self.is_logging = True
        self.logging_thread = threading.Thread(target=self._logging_loop)
        self.logging_thread.daemon = True
        self.logging_thread.start()
        print(f"开始记录数据到 {self.log_file}")
        print("正在持续接收并记录最新的SBUS数据...")
        print("提示：数据会实时更新到文件中")

    def _create_csv_file(self):
        """创建CSV文件并设置字段名"""
        try:
            self.csv_file = open(self.log_file, 'w', newline='', encoding='utf-8')

            # 创建字段名列表
            fieldnames = ['timestamp', 'frame_count', 'error_count', 'is_valid', 'raw_data']
            for i in range(1, SBUSConstants.CHANNELS + 1):
                fieldnames.append(f'ch{i}')
            fieldnames.extend(['ch17', 'ch18', 'frame_lost', 'failsafe'])

            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
            self.csv_writer.writeheader()

        except Exception as e:
            self._handle_error("创建CSV文件", e)

    def stop_logging(self):
        """停止记录数据"""
        if self.is_logging:
            self.is_logging = False
            # 等待记录线程结束
            if hasattr(self, 'logging_thread') and self.logging_thread:
                self.logging_thread.join(timeout=1)
            if self.csv_file:
                self.csv_file.close()
            print("停止数据记录")

    def _logging_loop(self):
        """记录循环 - 持续接收并记录最新的SBUS数据"""
        while self.is_logging and self.is_connected:
            try:
                # 使用通用函数处理帧，无限制处理
                if self._receive_and_process_frames(max_frames=None, timeout=0.1):
                    # 记录最新的有效帧
                    self._log_frame_data()

            except Exception as e:
                self._handle_error("记录循环", e)
                break

    def _log_frame_data(self):
        """记录当前帧数据"""
        if not self.is_logging or not self.csv_writer:
            return

        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            row_data = {
                'timestamp': timestamp,
                'frame_count': self.frame_count,
                'error_count': self.error_count,
                'is_valid': True,
                'raw_data': ''  # 正常记录不包含原始数据
            }

            # 添加通道数据
            for i in range(SBUSConstants.CHANNELS):
                row_data[f'ch{i+1}'] = self.channels[i]

            row_data['ch17'] = self.digital_channels[0]
            row_data['ch18'] = self.digital_channels[1]
            row_data['frame_lost'] = self.frame_lost
            row_data['failsafe'] = self.failsafe

            self.csv_writer.writerow(row_data)
            self.csv_file.flush()

        except Exception as e:
            self._handle_error("记录数据", e)
            # 如果记录失败，停止记录
            self.stop_logging()

    def get_status(self):
        """获取连接状态"""
        return {
            'connected': self.is_connected,
            'monitoring': self.is_monitoring,
            'logging': self.is_logging,
            'frame_count': self.frame_count,
            'error_count': self.error_count,
            'last_frame_time': self.last_frame_time,
            'frame_lost': self.frame_lost,
            'failsafe': self.failsafe,
            'log_file': self.log_file if self.is_logging else None
        }

    def display_current_data(self):
        """显示当前通道数据 - 立即接收并解析一个完整帧"""
        if not self.is_connected:
            print("请先连接串口")
            return

        print("\n正在接收SBUS数据...")
        print("等待接收一个完整的SBUS帧...")

        # 尝试接收并解析一个完整帧
        if self._receive_and_process_frames(max_frames=1, timeout=SBUSConstants.FRAME_TIMEOUT):
            print("成功接收到一个完整的SBUS帧!")
            self.frame_count += 1
            self.last_frame_time = time.time()

            # 显示解析后的数据
            self._display_frame_data_with_header("SBUS帧数据")

            # 如果正在记录，记录当前数据
            if self.is_logging:
                self._log_frame_data()
                print(f"\n数据已记录到: {self.log_file}")
        else:
            self._display_timeout_message()

    def _receive_and_process_frames(self, max_frames=None, timeout=SBUSConstants.FRAME_TIMEOUT):
        """
        通用的串口数据接收和帧处理函数

        Args:
            max_frames: 最大处理帧数，None表示无限制
            timeout: 超时时间（秒）

        Returns:
            bool: 成功处理至少一帧返回True，否则返回False
        """
        temp_buffer = bytearray()
        start_time = time.time()
        frames_processed = 0

        while time.time() - start_time < timeout:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    temp_buffer.extend(data)

                    # 持续处理帧
                    while True:
                        frame = self._find_frame_in_buffer(temp_buffer, find_first=True)
                        if frame is not None:
                            if self.parse_sbus_frame(frame):
                                frames_processed += 1
                                if max_frames and frames_processed >= max_frames:
                                    return True
                                # 移除已处理的帧数据
                                temp_buffer = temp_buffer[SBUSConstants.FRAME_LENGTH:]
                            else:
                                # 解析失败，移除第一个字节继续
                                temp_buffer = temp_buffer[1:]
                        else:
                            break

                time.sleep(0.01)  # 10ms间隔

            except Exception as e:
                self._handle_error("接收数据", e)
                return False

        return frames_processed > 0

    def _find_frame_in_buffer(self, buffer, find_first=True):
        """
        从缓冲区中查找SBUS帧的通用函数

        Args:
            buffer: 数据缓冲区
            find_first: True返回第一个帧，False返回最后一个帧

        Returns:
            bytes or None: 找到帧返回帧数据，否则返回None
        """
        frames_found = []

        while len(buffer) >= SBUSConstants.FRAME_LENGTH:
            # 查找起始字节
            start_idx = -1
            for i in range(len(buffer) - SBUSConstants.FRAME_LENGTH + 1):
                if buffer[i] == SBUSConstants.START_BYTE:
                    start_idx = i
                    break

            if start_idx == -1:
                # 没找到起始字节，保留最后几个字节
                buffer = buffer[-SBUSConstants.FRAME_LENGTH:]
                break

            # 移除起始字节之前的数据
            if start_idx > 0:
                buffer = buffer[start_idx:]

            # 检查是否有完整的帧
            if len(buffer) >= SBUSConstants.FRAME_LENGTH:
                frame = buffer[:SBUSConstants.FRAME_LENGTH]
                frames_found.append(frame)

                if find_first:
                    return frame

                # 移除已处理的帧数据，继续查找
                buffer = buffer[SBUSConstants.FRAME_LENGTH:]
            else:
                break

        # 如果查找最后一个帧，返回最后一个
        if not find_first and frames_found:
            return frames_found[-1]

        return None

    def _display_timeout_message(self):
        """显示超时提示信息"""
        print("超时：5秒内未接收到有效的SBUS帧")
        print("请检查:")
        print("1. SBUS信号源是否正常工作")
        print("2. 串口连接是否正常")
        print("3. 信号格式是否正确")

    def _display_frame_data_with_header(self, header="SBUS数据"):
        """
        通用的帧数据显示函数

        Args:
            header: 显示标题
        """
        print("\n" + "="*60)
        print(f"              {header}")
        print("="*60)

        # 显示通道数据
        self._display_channels()

        # 显示数字通道和状态
        self._display_digital_channels()
        self._display_status_info()

        print("="*60)

    def _display_channels(self):
        """显示模拟通道数据"""
        print("模拟通道 (PWM值):")
        for i in range(SBUSConstants.CHANNELS):
            if self.channels[i] > 0:
                print(f"CH{i+1:2d}: {self.channels[i]:4d}us", end="  ")
            else:
                print(f"CH{i+1:2d}: ----us", end="  ")
            if (i + 1) % SBUSConstants.DISPLAY_COLUMNS == 0:
                print()

        if SBUSConstants.CHANNELS % SBUSConstants.DISPLAY_COLUMNS != 0:
            print()

    def _display_digital_channels(self):
        """显示数字通道数据"""
        print(f"\n数字通道:")
        print(f"CH17: {'ON' if self.digital_channels[0] else 'OFF'}")
        print(f"CH18: {'ON' if self.digital_channels[1] else 'OFF'}")

    def _display_status_info(self):
        """显示状态信息"""
        print(f"\n状态信息:")
        print(f"帧丢失: {'是' if self.frame_lost else '否'}")
        print(f"故障保护: {'是' if self.failsafe else '否'}")
        print(f"接收帧数: {self.frame_count}")
        print(f"错误帧数: {self.error_count}")

    def set_port(self):
        """设置串口号"""
        try:
            print(f"当前串口: {self.port}")
            new_port = input("请输入新的串口号 (如 COM1, COM2, COM3 等): ").strip()

            if not new_port:
                print("使用默认串口 COM3")
                return True

            # 验证串口格式
            if self._validate_port_format(new_port):
                self.port = new_port.upper()
                print(f"串口已设置为: {self.port}")
                return True
            else:
                return False

        except Exception as e:
            self._handle_error("设置串口", e)
            return False

    def _validate_port_format(self, port):
        """
        验证串口格式

        Args:
            port: 串口号字符串

        Returns:
            bool: 格式正确返回True，否则返回False
        """
        if not port.upper().startswith('COM'):
            print("串口格式错误，请使用 COM1, COM2, COM3 等格式")
            return False

        # 检查是否包含数字
        port_num = port[3:]
        if not port_num.isdigit():
            print("串口格式错误，请使用 COM1, COM2, COM3 等格式")
            return False

        return True


def show_menu():
    """显示主菜单"""
    print("\n" + "="*60)
    print("              SBUS协议分析器")
    print("="*60)
    print("1. 连接串口 (100000波特率)")
    print("2. 开始监控")
    print("3. 停止监控")
    print("4. 开始记录数据")
    print("5. 停止记录数据")
    print("6. 显示当前通道数据")
    print("7. 显示连接状态")
    print("8. 清屏")
    print("9. 设置串口号")
    print("10. 断开连接")
    print("11. 退出")
    print("-"*60)


def main():
    """主函数"""
    analyzer = SBUSAnalyzer()

    print("欢迎使用SBUS协议分析器!")
    print("集成了SBUS协议解析和数据记录功能")
    print("专为Futaba SBUS信号解析设计")
    print("支持电平取反的硬件实现")

    while True:
        show_menu()
        choice = input("请选择操作 (1-11): ").strip()

        if choice == '1':
            analyzer.connect()

        elif choice == '2':
            analyzer.start_monitoring()

        elif choice == '3':
            analyzer.stop_monitoring()

        elif choice == '4':
            log_file = input("请输入记录文件名 (默认: sbus_data.csv): ").strip()
            if not log_file:
                log_file = "sbus_data.csv"
            analyzer.start_logging(log_file)

        elif choice == '5':
            analyzer.stop_logging()

        elif choice == '6':
            analyzer.display_current_data()

        elif choice == '7':
            status = analyzer.get_status()
            print(f"\n连接状态:")
            print(f"当前串口: {analyzer.port}")
            print(f"已连接: {'是' if status['connected'] else '否'}")
            print(f"监控中: {'是' if status['monitoring'] else '否'}")
            print(f"记录中: {'是' if status['logging'] else '否'}")
            if status['logging']:
                print(f"记录文件: {status['log_file']}")
            print(f"接收帧数: {status['frame_count']}")
            print(f"错误帧数: {status['error_count']}")
            print(f"帧丢失: {'是' if status['frame_lost'] else '否'}")
            print(f"故障保护: {'是' if status['failsafe'] else '否'}")

        elif choice == '8':
            import os
            os.system('cls' if os.name == 'nt' else 'clear')

        elif choice == '9':
            analyzer.set_port()

        elif choice == '10':
            analyzer.disconnect()

        elif choice == '11':
            analyzer.disconnect()
            print("感谢使用SBUS协议分析器!")
            break

        else:
            print("无效选择，请重新输入")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
        sys.exit(0)
    except Exception as e:
        print(f"\n程序运行错误: {e}")
        sys.exit(1)
