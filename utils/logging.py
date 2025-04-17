#!/usr/bin/env python3

"""
Logging utilities for the Turtlebot4 controller application
"""

import os
import logging
from datetime import datetime

def setup_logger(name, log_dir="logs"):
    """
    Set up a logger with file and console handlers
    
    Args:
        name (str): Name of the logger
        log_dir (str): Directory to store log files
        
    Returns:
        logging.Logger: Configured logger instance
    """
    # Create logs directory if it doesn't exist
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # Create a logger
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    
    # Create a file handler
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_file = os.path.join(log_dir, f"{name}_{timestamp}.log")
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.INFO)
    
    # Create a console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    
    # Create a formatter and set it for both handlers
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)
    
    # Add handlers to logger
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger

def log_command(logger, command_type, command_values, result, node_id="controller"):
    """
    Log a robot command with standardized format
    
    Args:
        logger (logging.Logger): Logger instance
        command_type (str): Type of command (e.g., "move", "dock")
        command_values (dict): Dictionary of command parameters
        result (bool): Whether the command was successful
        node_id (str): Identifier for the node issuing the command
    """
    timestamp = datetime.now().isoformat()
    
    # Create a standardized log entry
    log_entry = {
        "timestamp": timestamp,
        "node_id": node_id,
        "command_type": command_type,
        "parameters": command_values,
        "result": "success" if result else "failure"
    }
    
    logger.info(f"COMMAND: {log_entry}")

def log_security_event(logger, event_type, details, severity="INFO"):
    """
    Log a security-related event
    
    Args:
        logger (logging.Logger): Logger instance
        event_type (str): Type of security event
        details (dict): Additional details about the event
        severity (str): Severity level (INFO, WARNING, ERROR)
    """
    timestamp = datetime.now().isoformat()
    
    # Create a standardized security log entry
    log_entry = {
        "timestamp": timestamp,
        "event_type": event_type,
        "details": details,
        "severity": severity
    }
    
    if severity == "WARNING":
        logger.warning(f"SECURITY: {log_entry}")
    elif severity == "ERROR":
        logger.error(f"SECURITY: {log_entry}")
    else:
        logger.info(f"SECURITY: {log_entry}")