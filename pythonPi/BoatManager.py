import os
import signal
import yaml
import logging
import queue
from datetime import datetime

from RfManager import RfManager
from MotorManager import MotorManager

def main():
    configPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "config.yaml")
    with open(configPath, 'r') as configStream:
        config = yaml.safe_load(configStream)

    logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    loggerRf = logging.getLogger('loggerRf')
    loggerMotor = logging.getLogger('loggerMotor')

    # Build timestamped log file paths
    timestamp = datetime.now().strftime('%Y-%m-%d_%H:%M.%S')
    rfLogDir = os.path.dirname(config['rfManager']['rfLoggerFilePath'])
    motorLogDir = os.path.dirname(config['motorManager']['motorLoggerFilePath'])
    rfLogPath = os.path.join(rfLogDir, f'rfLog_{timestamp}.log')
    motorLogPath = os.path.join(motorLogDir, f'motorLog_{timestamp}.log')

    # Create logging handlers
    rfFileHandler = logging.FileHandler(rfLogPath)
    rfFileHandler.setLevel(logging.DEBUG)
    motorFileHandler = logging.FileHandler(motorLogPath)
    motorFileHandler.setLevel(logging.DEBUG)

    logFormatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    rfFileHandler.setFormatter(logFormatter)
    motorFileHandler.setFormatter(logFormatter)

    loggerRf.addHandler(rfFileHandler)
    loggerMotor.addHandler(motorFileHandler)

    # Create queues for thread comms
    motorToRfQueue = queue.Queue()
    rfToMotorQueue = queue.Queue()

    # Create manager objects
    rfManager = RfManager(config['rfManager'], loggerRf, motorToRfQueue, rfToMotorQueue)
    motorManager = MotorManager(config['motorManager'], loggerMotor, rfToMotorQueue, motorToRfQueue)

    def shutdownHandler(signum, frame):
        """Handle SIGINT/SIGTERM for graceful shutdown."""
        logger = logging.getLogger('BoatManager')
        logger.info('Shutdown signal received (signal %s)', signum)
        motorManager.shutdown()
        rfManager.shutdown()

    signal.signal(signal.SIGINT, shutdownHandler)
    signal.signal(signal.SIGTERM, shutdownHandler)

    # Start all threads running
    rfManager.daemon = True
    motorManager.daemon = True
    rfManager.start()
    motorManager.start()

    rfManager.join()
    motorManager.join()


if __name__ == '__main__':
    main()
