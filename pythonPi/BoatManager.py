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
from ImuManager import ImuManager
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
    loggerImu = logging.getLogger('loggerImu')
    loggerImu.setLevel(LOG_LEVEL_MAP[config['imuManager']['imuLoggerVerbosityLevel']])
    loggerImu.propagate = False

    # Build timestamped log file paths
    timestamp = datetime.now().strftime('%Y-%m-%d_%H:%M.%S')
    rfLogDir = os.path.dirname(config['rfManager']['rfLoggerFilePath'])
    motorLogDir = os.path.dirname(config['motorManager']['motorLoggerFilePath'])
    imuLogDir = os.path.dirname(config['imuManager']['imuLoggerFilePath'])
    rfLogPath = os.path.join(rfLogDir, f'rfLog_{timestamp}.log')
    motorLogPath = os.path.join(motorLogDir, f'motorLog_{timestamp}.log')
    imuLogPath = os.path.join(imuLogDir, f'imuLog_{timestamp}.log')

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

    imuFileHandler = logging.FileHandler(imuLogPath)
    imuFileHandler.setLevel(logging.DEBUG)
    imuFileHandler.setFormatter(logFormatter)
    imuLogQueue = queue.Queue()
    imuQueueHandler = logging.handlers.QueueHandler(imuLogQueue)
    imuLogListener = logging.handlers.QueueListener(imuLogQueue, imuFileHandler, respect_handler_level=True)
    loggerImu.addHandler(imuQueueHandler)

    # Start log listeners before manager threads
    rfLogListener.start()
    motorLogListener.start()
    imuLogListener.start()

    # Initialize InfluxDB writer
    influxConfigPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "influxdb_setup.json")
    influxWriter = InfluxWriter(influxConfigPath)

    # Create queues for thread comms
    motorToRfQueue = queue.Queue()
    rfToMotorQueue = queue.Queue()
    oarImuToMotorQueue = queue.Queue()
    imuToMotorQueue = queue.Queue()
    imuRawToMlQueue = queue.Queue()

    # Create manager objects
    rfManager = RfManager(config['rfManager'], loggerRf, motorToRfQueue, rfToMotorQueue, oarImuToMotorQueue, influxWriter)
    imuManager = ImuManager(config['imuManager'], loggerImu, imuToMotorQueue, rawQueue=imuRawToMlQueue, influxWriter=influxWriter)
    motorManager = MotorManager(config['motorManager'], loggerMotor, rfToMotorQueue, motorToRfQueue,
                                imuQueue=imuToMotorQueue, oarImuQueue=oarImuToMotorQueue, influxWriter=influxWriter)

    def shutdownHandler(signum, frame):
        """Handle SIGINT/SIGTERM for graceful shutdown."""
        logger = logging.getLogger('BoatManager')
        logger.info('Shutdown signal received (signal %s)', signum)
        motorManager.shutdown()
        rfManager.shutdown()
        imuManager.shutdown()
        influxWriter.close()
        rfLogListener.stop()
        motorLogListener.stop()
        imuLogListener.stop()

    signal.signal(signal.SIGINT, shutdownHandler)
    signal.signal(signal.SIGTERM, shutdownHandler)

    # Start all threads running
    rfManager.daemon = True
    motorManager.daemon = True
    imuManager.daemon = True
    rfManager.start()
    motorManager.start()
    imuManager.start()

    rfManager.join()
    motorManager.join()
    imuManager.join()


if __name__ == '__main__':
    main()
