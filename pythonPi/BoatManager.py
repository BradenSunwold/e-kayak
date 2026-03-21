import os
import signal
import yaml
import logging
import queue

from RfManager import RfManager
from MotorManager import MotorManager

def main():
    configPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "config.yaml")
    with open(configPath, 'r') as configStream:
        config = yaml.safe_load(configStream)

    logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', filemode='w')
    loggerRf = logging.getLogger('loggerRf')
    loggerMotor = logging.getLogger('loggerMotor')

    # Create logging handlers
    rfFileHandler = logging.FileHandler(config['rfManager']['rfLoggerFilePath'])
    rfFileHandler.setLevel(logging.DEBUG)
    motorFileHandler = logging.FileHandler(config['motorManager']['motorLoggerFilePath'])
    motorFileHandler.setLevel(logging.DEBUG)

    rfLoggerFormatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    rfFileHandler.setFormatter(rfLoggerFormatter)
    motorLoggerFormatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    motorFileHandler.setFormatter(motorLoggerFormatter)

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
