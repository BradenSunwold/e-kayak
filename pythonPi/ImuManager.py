import json
import time
import struct
import sched
import threading
import queue
from pathlib import Path
from adafruit_extended_bus import ExtendedI2C
import adafruit_bno055
from EventTimer import EventTimer


class ImuManager(threading.Thread):
    def __init__(self, configDictionary, logger, outgoingQueue, rawQueue=None, influxWriter=None):
        super().__init__()
        self.mLogger = logger
        self.mInfluxWriter = influxWriter
        self.mLogger.info('**** IMU Manager Starting Up *****')
        self.mLogger.info('*************************************')

        # Queue to send heading data to motor thread
        self.mOutgoingQueue = outgoingQueue

        # Queue to send raw accel/gyro data to ML thread
        self.mRawQueue = rawQueue

        # Parse configs
        self.mConfigurator = configDictionary
        self.mI2cBus = self.mConfigurator['i2cBus']
        self.mCalibrationFile = self.mConfigurator['calibrationFilePath']

        # Set up scheduler
        self.mScheduler = sched.scheduler(time.time, time.sleep)
        self.mReadInterval = self.mConfigurator['imuReadRateInMilliseconds'] / 1000
        self.mNextReadTime = 0

        # Rate tracking
        self.mEventTimer = EventTimer(self.mLogger)
        self.mReadTotalCount = 0
        self.mReadDropCount = 0
        self.mReadDropLogInterval = 100
        self.mReadDropThresholdHz = self.mConfigurator['imuReadDropThresholdHz']

        # Shutdown flag
        self.mShutdownRequested = False

        # Initialize BNO055 over I2C
        i2c = ExtendedI2C(self.mI2cBus)
        self.mSensor = adafruit_bno055.BNO055_I2C(i2c)

        # Mounting orientation: chip→kayak transform is Rz(-90)·Ry(0)·Rx(180) (intrinsic ZYX).
        # Yields kayak X = -chip Y (forward), Y = -chip X (side), Z = -chip Z (up).
        self.mSensor.axis_remap = (
            adafruit_bno055.AXIS_REMAP_Y,
            adafruit_bno055.AXIS_REMAP_X,
            adafruit_bno055.AXIS_REMAP_Z,
            adafruit_bno055.AXIS_REMAP_NEGATIVE,
            adafruit_bno055.AXIS_REMAP_NEGATIVE,
            adafruit_bno055.AXIS_REMAP_NEGATIVE,
        )

        # Load saved calibration if available
        self._LoadCalibration()

    def _LoadCalibration(self):
        """Load saved calibration offsets and radii from JSON file."""
        calPath = Path(self.mCalibrationFile)
        if calPath.exists():
            try:
                with open(calPath, 'r') as f:
                    cal = json.load(f)
                self.mSensor.offsets_accelerometer = tuple(cal['offsets_accelerometer'])
                self.mSensor.offsets_magnetometer = tuple(cal['offsets_magnetometer'])
                self.mSensor.offsets_gyroscope = tuple(cal['offsets_gyroscope'])
                self.mSensor.radius_accelerometer = cal['radius_accelerometer']
                self.mSensor.radius_magnetometer = cal['radius_magnetometer']
                self.mLogger.info('Loaded BNO055 calibration from %s', calPath)
            except Exception as e:
                self.mLogger.error('Failed to load calibration: %s', e)
        else:
            self.mLogger.warning('No calibration file found at %s', calPath)

    def ReadImu(self):
        if self.mShutdownRequested:
            return

        try:
            euler = self.mSensor.euler
            accel = self.mSensor.linear_acceleration
            gyro = self.mSensor.gyro
            sys_cal, gyro_cal, accel_cal, mag_cal = self.mSensor.calibration_status

            if euler[0] is not None:
                heading, roll, pitch = euler
                ax, ay, az = accel
                gx, gy, gz = gyro

                # Pack heading as a float and send to motor thread
                payload = struct.pack('f', heading)
                self.mOutgoingQueue.put(payload)

                # Pack linear accel (gravity-removed) and gyro, send to ML thread
                if self.mRawQueue is not None:
                    rawPayload = struct.pack('ffffff', ax, ay, az, gx, gy, gz)
                    self.mRawQueue.put(rawPayload)

                self.mLogger.debug('Kayak IMU  heading=%.2f  roll=%.2f  pitch=%.2f  '
                                   'accel=[%.2f, %.2f, %.2f]  gyro=[%.2f, %.2f, %.2f]  '
                                   'cal[sys=%d gyro=%d accel=%d mag=%d]',
                                   heading, roll, pitch,
                                   ax, ay, az, gx, gy, gz,
                                   sys_cal, gyro_cal, accel_cal, mag_cal)

                # Rate tracking
                result = self.mEventTimer.mark('imu_read')
                if result is not None:
                    delta, rate, stats = result
                    self.mReadTotalCount += 1
                    if rate < self.mReadDropThresholdHz:
                        self.mReadDropCount += 1
                    if self.mReadTotalCount % self.mReadDropLogInterval == 0:
                        self.mLogger.info(
                            'imu_read drops below %g Hz: %d / %d (%.1f%%)',
                            self.mReadDropThresholdHz,
                            self.mReadDropCount, self.mReadTotalCount,
                            100.0 * self.mReadDropCount / self.mReadTotalCount
                        )

                    if self.mInfluxWriter:
                        dropPct = (100.0 * self.mReadDropCount / self.mReadTotalCount) if self.mReadTotalCount > 0 else 0.0
                        self.mInfluxWriter.write_point("kayak_imu", {
                            "heading": heading,
                            "roll": roll,
                            "pitch": pitch,
                            "accel_x": ax,
                            "accel_y": ay,
                            "accel_z": az,
                            "gyro_x": gx,
                            "gyro_y": gy,
                            "gyro_z": gz,
                            "cal_sys": sys_cal,
                            "cal_gyro": gyro_cal,
                            "cal_accel": accel_cal,
                            "cal_mag": mag_cal,
                            "read_rate_hz": rate,
                            "read_delta_ms": delta * 1000.0,
                            "read_drop_count": self.mReadDropCount,
                            "read_total_count": self.mReadTotalCount,
                            "read_drop_pct": dropPct,
                            "read_jitter_ms": stats['std'] * 1000.0,
                        })
            else:
                self.mLogger.debug('Kayak IMU not ready yet')

        except Exception as e:
            self.mLogger.error('ReadImu exception: %s', e)

        self.mNextReadTime += self.mReadInterval
        self.mScheduler.enterabs(self.mNextReadTime, 1, self.ReadImu)

    def StartScheduler(self):
        now = time.time()
        self.mNextReadTime = now

        self.mScheduler.enterabs(self.mNextReadTime, 1, self.ReadImu)
        self.mScheduler.run()

    def shutdown(self):
        """Signal the IMU manager to stop."""
        self.mLogger.info('IMU shutdown requested')
        self.mShutdownRequested = True

    def run(self):
        self.StartScheduler()
