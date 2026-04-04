import os
import signal
import yaml
import json
import logging
import logging.handlers
import queue
from datetime import datetime

from RfManager import RfManager
from MotorManager import MotorManager
from InfluxWriter import InfluxWriter

def main():
    configPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "config.yaml")
    with open(configPath, 'r') as configStream:
        config = yaml.safe_load(configStream)

    logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    LOG_LEVEL_MAP = {
        'DEBUG': logging.DEBUG,
        'INFO': logging.INFO,
        'WARN': logging.WARNING,
        'ERROR': logging.ERROR,
    }

    loggerRf = logging.getLogger('loggerRf')
    loggerRf.setLevel(LOG_LEVEL_MAP[config['rfManager']['rfLoggerVerbosityLevel']])
    loggerRf.propagate = False      # Don't propagate to root logger's synchronous StreamHandler
    loggerMotor = logging.getLogger('loggerMotor')
    loggerMotor.setLevel(LOG_LEVEL_MAP[config['motorManager']['motorLoggerVerbosityLevel']])
    loggerMotor.propagate = False   # Don't propagate to root logger's synchronous StreamHandler

    # Build timestamped log file paths
    timestamp = datetime.now().strftime('%Y-%m-%d_%H:%M.%S')
    rfLogDir = os.path.dirname(config['rfManager']['rfLoggerFilePath'])
    motorLogDir = os.path.dirname(config['motorManager']['motorLoggerFilePath'])
    rfLogPath = os.path.join(rfLogDir, f'rfLog_{timestamp}.log')
    motorLogPath = os.path.join(motorLogDir, f'motorLog_{timestamp}.log')

    # Create logging handlers using QueueHandler/QueueListener so file I/O
    # happens in background threads and never blocks the RF or Motor schedulers.
    logFormatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    rfFileHandler = logging.FileHandler(rfLogPath)
    rfFileHandler.setLevel(logging.DEBUG)
    rfFileHandler.setFormatter(logFormatter)
    rfLogQueue = queue.Queue()
    rfQueueHandler = logging.handlers.QueueHandler(rfLogQueue)
    rfLogListener = logging.handlers.QueueListener(rfLogQueue, rfFileHandler, respect_handler_level=True)
    loggerRf.addHandler(rfQueueHandler)

    motorFileHandler = logging.FileHandler(motorLogPath)
    motorFileHandler.setLevel(logging.DEBUG)
    motorFileHandler.setFormatter(logFormatter)
    motorLogQueue = queue.Queue()
    motorQueueHandler = logging.handlers.QueueHandler(motorLogQueue)
    motorLogListener = logging.handlers.QueueListener(motorLogQueue, motorFileHandler, respect_handler_level=True)
    loggerMotor.addHandler(motorQueueHandler)

    # Start log listeners before manager threads
    rfLogListener.start()
    motorLogListener.start()

    # Initialize InfluxDB writer
    influxConfigPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "influxdb_setup.json")
    influxWriter = InfluxWriter(influxConfigPath)

    # Create queues for thread comms
    motorToRfQueue = queue.Queue()
    rfToMotorQueue = queue.Queue()

    # Create manager objects
    rfManager = RfManager(config['rfManager'], loggerRf, motorToRfQueue, rfToMotorQueue, influxWriter)
    motorManager = MotorManager(config['motorManager'], loggerMotor, rfToMotorQueue, motorToRfQueue, influxWriter)

    def shutdownHandler(signum, frame):
        """Handle SIGINT/SIGTERM for graceful shutdown."""
        logger = logging.getLogger('BoatManager')
        logger.info('Shutdown signal received (signal %s)', signum)
        motorManager.shutdown()
        rfManager.shutdown()
        influxWriter.close()
        rfLogListener.stop()
        motorLogListener.stop()

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
