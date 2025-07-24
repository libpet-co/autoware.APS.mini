#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray

# 文件路径，用于存储诊断信息
LOG_FILE = 'diagnostics_log.txt'

class DiagnosticsLoggerNode(Node):
    """
    ROS2 节点类：订阅诊断话题并存储到文件中
    """
    def __init__(self):
        super().__init__('diagnostics_logger_node')
        
        # 订阅 /diagnostics 话题
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10  # QoS 深度
        )
        
        self.get_logger().info('Diagnostics logger node started. Waiting for messages...')

    def diagnostics_callback(self, msg):
        """
        回调函数：接收诊断消息并存储到文件中
        """
        try:
            # 将消息转换为可读字符串
            log_content = f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}\n"
            for status in msg.status:
                log_content += f"Level: {status.level}, Name: {status.name}, Message: {status.message}\n"
                log_content += f"Hardware ID: {status.hardware_id}\n"
                for kv in status.values:
                    log_content += f"  {kv.key}: {kv.value}\n"
                log_content += "-" * 50 + "\n"
            
            # 追加写入文件
            with open(LOG_FILE, 'a') as f:
                f.write(log_content + "\n")
            
            self.get_logger().info("Diagnostics message logged to file.")
        except Exception as e:
            self.get_logger().error(f"Error logging diagnostics: {e}")

def main(args=None):
    """
    主函数：初始化节点并运行
    """
    rclpy.init(args=args)
    
    node = DiagnosticsLoggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
