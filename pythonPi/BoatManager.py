import os
import signal
import yaml
import json
import logging
import logging.handlers
import queue
import multiprocessing
from datetime import datetime

from RfManager import RfManager
from MotorManager import MotorManager
from ImuManager import ImuManager
from MlManager import MlManager
from InfluxWriter import InfluxWriter

def main():
    # Spawn (not fork) for the ML process. Fork would duplicate the parent's
    # open file descriptors and background threads (RF radio, UART, InfluxDB
    # client's write thread) into the child, which can corrupt I/O or leak
    # resources. Spawn starts the child with a fresh Python interpreter and
    # only the args we explicitly pass to Process(...).
    multiprocessing.set_start_method('spawn', force=True)

    configPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "config.yaml")
    with open(configPath, 'r') as configStream:
        config = yaml.safe_load(configStream)

    # Pin the main (parent) process to its own set of cores, leaving the ML
    # process free to use the cores assigned in its own config block. The OS
    # scheduler can still move individual threads inside this process across
    # the allowed cores, but it can't touch the ML cores.
    boatAffinity = config.get('boatManager', {}).get('cpuAffinity')
    if boatAffinity:
        try:
            os.sched_setaffinity(0, set(boatAffinity))
        except (AttributeError, OSError):
            pass  # Non-Linux host; skip

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
    mlLogDir = os.path.dirname(config['mlManager']['mlLoggerFilePath'])
    rfLogPath = os.path.join(rfLogDir, f'rfLog_{timestamp}.log')
    motorLogPath = os.path.join(motorLogDir, f'motorLog_{timestamp}.log')
    imuLogPath = os.path.join(imuLogDir, f'imuLog_{timestamp}.log')
    # ML log path is built here so its filename lines up with the others, but
    # the actual FileHandler is created inside the ML child process — see
    # MlManager._setupLogger.
    mlLogPath = os.path.join(mlLogDir, f'mlLog_{timestamp}.log')

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

    # One session string shared by every process's InfluxWriter so all
    # telemetry from this run carries the same session tag in Grafana.
    influxSession = datetime.now().strftime('%Y-%m-%d_%H:%M:%S')

    # Initialize InfluxDB writer
    influxConfigPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "influxdb_setup.json")
    influxWriter = InfluxWriter(influxConfigPath, session=influxSession)

    # Create queues for thread comms. The two ML-touching queues must be
    # multiprocessing.Queue so they work across the process boundary; the rest
    # stay plain queue.Queue for the in-process threads.
    motorToRfQueue = queue.Queue()
    rfToMotorQueue = queue.Queue()
    oarImuToMotorQueue = queue.Queue()
    imuToMotorQueue = queue.Queue()
    imuRawToMlQueue = queue.Queue()
    oarRawToMlQueue = multiprocessing.Queue()
    mlToMotorQueue = multiprocessing.Queue()  # Reserved for future stroke events to the motor

    # Event object the parent uses to ask the ML child to shut down cleanly.
    mlShutdownEvent = multiprocessing.Event()

    # Create manager objects
    rfManager = RfManager(config['rfManager'], loggerRf, motorToRfQueue, rfToMotorQueue,
                          oarImuToMotorQueue, oarRawToMlQueue=oarRawToMlQueue,
                          influxWriter=influxWriter)
    imuManager = ImuManager(config['imuManager'], loggerImu, imuToMotorQueue, rawQueue=imuRawToMlQueue, influxWriter=influxWriter)
    motorManager = MotorManager(config['motorManager'], loggerMotor, rfToMotorQueue, motorToRfQueue,
                                imuQueue=imuToMotorQueue, oarImuQueue=oarImuToMotorQueue, influxWriter=influxWriter)
    # MlManager runs in its own process. No logger object is passed — the child
    # builds its own FileHandler-backed logger from the log path + level.
    mlManager = MlManager(
        config['mlManager'],
        oarRawToMlQueue,
        mlShutdownEvent,
        mlLogPath,
        LOG_LEVEL_MAP[config['mlManager']['mlLoggerVerbosityLevel']],
        influxConfigPath,
        influxSession,
        strokeOutQueue=mlToMotorQueue,
    )

    def shutdownHandler(signum, frame):
        """Handle SIGINT/SIGTERM for graceful shutdown."""
        logger = logging.getLogger('BoatManager')
        logger.info('Shutdown signal received (signal %s)', signum)
        motorManager.shutdown()
        rfManager.shutdown()
        imuManager.shutdown()
        # ML runs in a child process: signal via Event, then wait briefly and
        # hard-kill if it doesn't exit on its own.
        mlShutdownEvent.set()
        mlManager.join(timeout=5)
        if mlManager.is_alive():
            logger.warning('ML process did not exit within 5s — terminating')
            mlManager.terminate()
            mlManager.join(timeout=2)
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
    mlManager.daemon = True
    rfManager.start()
    motorManager.start()
    imuManager.start()
    mlManager.start()

    rfManager.join()
    motorManager.join()
    imuManager.join()
    mlManager.join()


if __name__ == '__main__':
    main()
