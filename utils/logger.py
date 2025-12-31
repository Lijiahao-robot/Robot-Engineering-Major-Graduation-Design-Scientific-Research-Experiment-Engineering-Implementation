#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import logging
import os
from datetime import datetime
import yaml

class RobotLogger:
    def __init__(self, node_name):
        self.node_name = node_name
        self.load_log_config()
        self.init_logger()

    def load_log_config(self):
        with open("./config/log_config.yaml", "r") as f:
            self.config = yaml.safe_load(f)
        self.log_dir = self.config["log_settings"]["log_dir"]
        os.makedirs(self.log_dir, exist_ok=True)

    def init_logger(self):
        # 初始化日志器
        self.logger = logging.getLogger(self.node_name)
        # 获取节点专属日志级别
        node_log_level = self.config["log_settings"]["log_level"]
        for node in self.config["nodes"]:
            if node["name"] == self.node_name:
                node_log_level = node["log_level"]
                break
        self.logger.setLevel(getattr(logging, node_log_level))

        # 日志格式
        log_format = self.config["log_settings"]["log_format"]
        formatter = logging.Formatter(
            log_format,
            datefmt="%Y-%m-%d %H:%M:%S"
        )

        # 文件处理器（按日期保存）
        today = datetime.now().strftime("%Y%m%d")
        log_file = os.path.join(self.log_dir, f"{self.node_name}_{today}.log")
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(formatter)

        # 控制台处理器
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)

        self.logger.addHandler(file_handler)
        self.logger.addHandler(console_handler)

    def debug(self, msg):
        self.logger.debug(msg)

    def info(self, msg):
        self.logger.info(msg)

    def warn(self, msg):
        self.logger.warning(msg)

    def error(self, msg):
        self.logger.error(msg)

    def critical(self, msg):
        self.logger.critical(msg)
