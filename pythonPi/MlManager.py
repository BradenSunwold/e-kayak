import json
import os
import time
import struct
import queue
import logging
import multiprocessing
from pathlib import Path

import numpy as np
import onnxruntime as ort

from InfluxWriter import InfluxWriter


class MlManager(multiprocessing.Process):
    """Runs the assist regression ONNX model in its own OS process.

    Why a Process instead of a Thread: CPython's Global Interpreter Lock
    (the "GIL") serializes Python bytecode across threads in a single
    interpreter. ONNX and numpy release the GIL around their native kernels,
    but a busy main thread (RF + motor scheduling) can still cause multi-ms
    stalls on inference latency. Moving ML into its own process gives it an
    independent interpreter and lets us pin it to a dedicated pair of CPU
    cores, leaving the other cores for RF/motor/IMU work.

    Lifecycle:
      - __init__ runs in the parent. Only stores picklable config/paths so
        the parent isn't holding ML resources when the child spawns.
      - run() runs in the child. Sets CPU affinity, creates the logger,
        InfluxWriter, and ONNX InferenceSession, then enters the sample loop.

    Inputs arrive from RfManager as 6-float packets in the training CSV order:
    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z.
    """

    def __init__(self, configDictionary, oarRawQueue, shutdownEvent,
                 logFilePath, logLevel, influxConfigPath, influxSession,
                 assistOutQueue=None):
        super().__init__()
        self.mConfigurator = configDictionary
        self.mIncomingQueue = oarRawQueue
        self.mShutdownEvent = shutdownEvent
        self.mAssistOutQueue = assistOutQueue  # assist predictions to MotorManager
        self.mLogFilePath = logFilePath
        self.mLogLevel = logLevel
        self.mInfluxConfigPath = influxConfigPath
        self.mInfluxSession = influxSession

        # Queue read timeout — short enough that shutdown is responsive
        self.mQueueReadTimeoutSeconds = 0.2

    def _setupLogger(self):
        """Build a child-process logger that writes directly to its own file.

        The parent uses QueueHandler/QueueListener for its threads so file I/O
        doesn't block scheduling, but that pattern is in-process only — the
        listener thread lives in the parent and the child can't reach it.
        A plain FileHandler in the child is simpler and keeps ML logs
        self-contained.
        """
        logger = logging.getLogger('loggerMl')
        logger.setLevel(self.mLogLevel)
        logger.propagate = False
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        fileHandler = logging.FileHandler(self.mLogFilePath)
        fileHandler.setLevel(logging.DEBUG)
        fileHandler.setFormatter(formatter)
        logger.addHandler(fileHandler)
        return logger

    def _applyCpuAffinity(self):
        """Pin this process to a subset of cores so the OS scheduler can't
        bounce inference onto CPUs saturated by RF/motor work in the main
        process."""
        affinity = self.mConfigurator.get('cpuAffinity')
        if not affinity:
            return
        try:
            os.sched_setaffinity(0, set(affinity))
            self.mLogger.info('CPU affinity set to cores %s', sorted(affinity))
        except (AttributeError, OSError) as e:
            # sched_setaffinity is Linux-only; silently skip elsewhere.
            self.mLogger.warning('Could not set CPU affinity: %s', e)

    def _loadArtifacts(self):
        """Resolve paths, load metadata + norm stats + ONNX model."""
        baseDir = Path(__file__).parent
        modelPath = (baseDir / self.mConfigurator['assistModelPath']).resolve()
        metaPath = (baseDir / self.mConfigurator['assistModelMetaPath']).resolve()
        normStatsPath = (baseDir / self.mConfigurator['assistNormStatsPath']).resolve()

        # Metadata sidecar describes the model's input shape and label scaling
        # so the Pi never duplicates training-side constants.
        with open(metaPath) as f:
            self.mMeta = json.load(f)
        self.mInputName = self.mMeta['input_name']
        self.mOutputName = self.mMeta['output_name']
        self.mNumChannels = self.mMeta['num_channels']
        self.mWindowSize = self.mMeta['window_size']
        # Multiply predictions by this to recover physical units (m/s^2 of
        # future kayak forward acceleration). The raw normalized prediction is
        # what the motor mapping consumes; physical units are for logging.
        self.mLabelNormScale = self.mMeta['label_norm_scale']
        self.mLogger.info('Loaded model metadata from %s', metaPath.name)
        self.mLogger.info(
            '  model_type=%s input_name=%s output_name=%s num_channels=%d window_size=%d',
            self.mMeta['model_type'], self.mInputName, self.mOutputName,
            self.mNumChannels, self.mWindowSize)
        self.mLogger.info('  label_normalization_scale=%.4f', self.mLabelNormScale)
        self.mLogger.info(
            '  trained epoch=%d validation_loss=%.4f validation_mean_absolute_error=%.4f',
            self.mMeta['checkpoint_epoch'],
            self.mMeta['checkpoint_validation_loss'],
            self.mMeta['checkpoint_validation_mae'])

        # Paddle idle gate parameters. Training drops paddle-idle samples, so
        # the model has never seen a still paddle — instead of asking it, we
        # flag those samples idle and MotorManager takes the assist to zero.
        # The gate config rides in the metadata sidecar so the thresholds
        # stay in lockstep with the training pipeline (single source of
        # truth: mlTraining/config.py, reference implementation
        # mlTraining/idle_gate.py — this streaming version must match it
        # sample-for-sample).
        idleGate = self.mMeta.get('idle_gate')
        if idleGate is None:
            self.mIdleGateEnabled = False
            self.mLogger.warning(
                'Metadata has no idle_gate block (model exported before the '
                'gate existed) — idle gate DISABLED, all samples treated as '
                'active paddling.')
        else:
            self.mIdleGateEnabled = True
            self.mIdleGateWindowSamples = int(round(
                idleGate['window_seconds'] * self.mMeta['sample_rate_hz']))
            self.mIdleGateEnterThreshold = idleGate['enter_threshold']
            self.mIdleGateExitThreshold = idleGate['exit_threshold']
            self.mLogger.info(
                '  idle gate: window=%d samples, enter<%.3f, exit>%.3f',
                self.mIdleGateWindowSamples,
                self.mIdleGateEnterThreshold, self.mIdleGateExitThreshold)

        # Per-channel mean / standard deviation computed from the training set.
        # Same preprocessing the model saw during training must be applied here.
        with open(normStatsPath) as f:
            stats = json.load(f)
        self.mChannelMeans = np.array(stats['means'], dtype=np.float32)
        self.mChannelStandardDeviations = np.array(stats['stds'], dtype=np.float32)
        self.mLogger.info('Loaded normalization statistics from %s', normStatsPath.name)

        # onnxruntime threading knobs. With a small CNN and a 2-core pool,
        # intra_op=1 is faster than letting onnxruntime spin up a thread pool:
        # the synchronization overhead outweighs any parallel-kernel win, and
        # a single inference thread keeps latency variance low.
        sessionOptions = ort.SessionOptions()
        sessionOptions.intra_op_num_threads = 1
        sessionOptions.inter_op_num_threads = 1
        self.mSession = ort.InferenceSession(
            modelPath.as_posix(),
            sess_options=sessionOptions,
            providers=['CPUExecutionProvider'],
        )
        self.mLogger.info('Loaded ONNX model from %s', modelPath.name)

    def _updateIdleGate(self, gx, gy, gz):
        """Feed one gyro sample into the idle-gate state machine.

        Mirrors mlTraining/idle_gate.py exactly: energy is the standard
        deviation (ddof=1, matching pandas' rolling .std() used in training)
        of gyro magnitude over the trailing window; hysteresis enters idle
        below the enter threshold, exits above the exit threshold, holds
        state in between. An unfilled window counts as idle — the safe
        startup state (no assist until paddling is confirmed).
        """
        if not self.mIdleGateEnabled:
            return

        magnitude = float(np.sqrt(gx * gx + gy * gy + gz * gz))
        self.mGyroMagnitudeBuffer[:-1] = self.mGyroMagnitudeBuffer[1:]
        self.mGyroMagnitudeBuffer[-1] = magnitude
        # Count THIS sample before the warm check: the window is full (and
        # energy computable) on the very sample that fills it — one sample
        # earlier than a check-then-increment would allow. Matches the
        # min_periods behavior of the pandas rolling std in idle_gate.py.
        self.mGyroMagnitudeFillCount = min(self.mGyroMagnitudeFillCount + 1,
                                           self.mIdleGateWindowSamples)
        if self.mGyroMagnitudeFillCount < self.mIdleGateWindowSamples:
            self.mPaddleIdle = True
            return

        self.mPaddleMotionEnergy = float(np.std(self.mGyroMagnitudeBuffer, ddof=1))
        if self.mPaddleIdle and self.mPaddleMotionEnergy > self.mIdleGateExitThreshold:
            self.mPaddleIdle = False
            self.mLogger.info('Idle gate OPEN (paddling detected, '
                              'motion_energy=%.3f)', self.mPaddleMotionEnergy)
        elif not self.mPaddleIdle and self.mPaddleMotionEnergy < self.mIdleGateEnterThreshold:
            self.mPaddleIdle = True
            self.mLogger.info('Idle gate CLOSED (paddle idle, '
                              'motion_energy=%.3f)', self.mPaddleMotionEnergy)

    def _ingestSample(self, payload):
        """Unpack one 6-float sample, push into the rolling buffer, run inference."""
        ax, ay, az, gx, gy, gz = struct.unpack('ffffff', payload)
        sample = np.array([ax, ay, az, gx, gy, gz], dtype=np.float32)

        self._updateIdleGate(gx, gy, gz)

        # Shift the buffer one sample to the left (dropping the oldest column)
        # and write the new sample into the rightmost column. Done in place to
        # avoid allocating a fresh array every sample.
        self.mBuffer[:, :-1] = self.mBuffer[:, 1:]
        self.mBuffer[:, -1] = sample

        if self.mBufferFillCount < self.mWindowSize:
            self.mBufferFillCount += 1
            if self.mBufferFillCount == self.mWindowSize:
                self.mLogger.info(
                    'Rolling buffer warm (%d samples) — inference starting',
                    self.mWindowSize)
            return

        # Normalize: (sample - channel_mean) / channel_standard_deviation
        # Means/stds are shape (num_channels,); broadcast across the time axis.
        normalized = ((self.mBuffer - self.mChannelMeans[:, None])
                      / self.mChannelStandardDeviations[:, None])
        modelInput = normalized[None, :, :]   # add batch dimension: (1, channels, time)

        start = time.perf_counter()
        (output,) = self.mSession.run(None, {self.mInputName: modelInput})
        latencyMilliseconds = (time.perf_counter() - start) * 1000.0

        # The regression model outputs the prediction scalar directly — no
        # softmax, no classes. Units are normalized label units: roughly
        # [-1, 1] for typical paddling, where 1.0 means "kayak about to
        # accelerate at 95th-percentile stroke intensity" and negative means
        # decelerating (coasting drag). The motor side clamps and maps this;
        # here it is passed through raw so logs show the true model output.
        assistPrediction = float(output[0])
        assistPredictionPhysical = assistPrediction * self.mLabelNormScale

        self.mInferenceCounter += 1
        if self.mInferenceCounter % self.mInferenceLogInterval == 0:
            self.mLogger.info(
                'Prediction: assist=%+.3f (%.3f m/s^2 forward acceleration) '
                'inference_latency=%.2fms',
                assistPrediction, assistPredictionPhysical, latencyMilliseconds)
        else:
            self.mLogger.debug(
                'Prediction: assist=%+.3f inference_latency=%.2fms',
                assistPrediction, latencyMilliseconds)

        # Send prediction + idle flag to the motor manager. The prediction is
        # always the true model output (logs and Influx stay honest); the
        # motor side is responsible for taking assist to zero while idle —
        # a still paddle is a state the model never trained on, so its
        # output there is meaningless.
        if self.mAssistOutQueue is not None:
            self.mAssistOutQueue.put(struct.pack('f?', assistPrediction, self.mPaddleIdle))

        if self.mInfluxWriter:
            # Fields only, no tags — tagging by any per-sample value would
            # split every field into separate series and break Grafana plots.
            # paddle_idle as int and the raw motion energy make threshold
            # tuning possible from a Grafana pane after an on-water session.
            self.mInfluxWriter.write_point('assist_model', {
                'assist_prediction': assistPrediction,
                'assist_prediction_physical': assistPredictionPhysical,
                'inference_latency_milliseconds': latencyMilliseconds,
                'paddle_idle': int(self.mPaddleIdle),
                'paddle_motion_energy': self.mPaddleMotionEnergy,
            })

    def run(self):
        """Child-process entry point. Builds logger/writer/session, then loops."""
        self.mLogger = self._setupLogger()
        self.mLogger.info('**** ML Manager process starting up, pid=%d ****', os.getpid())

        self._applyCpuAffinity()
        self._loadArtifacts()

        # InfluxWriter owns a background thread for batching. Constructing it
        # here (in the child) keeps that thread in this process, not forked
        # from the parent's write_api.
        self.mInfluxWriter = InfluxWriter(self.mInfluxConfigPath, session=self.mInfluxSession)

        # Rolling buffer shaped (num_channels, window_size). Newest sample at [:, -1].
        # Seeded to zeros; we report "warming up" until it has been fully filled.
        self.mBuffer = np.zeros((self.mNumChannels, self.mWindowSize), dtype=np.float32)
        self.mBufferFillCount = 0

        # Idle-gate state. Starts idle (gate closed) so the boat gives no
        # assist until paddling is positively detected — mirrors the
        # incomplete-window-is-idle rule in mlTraining/idle_gate.py.
        self.mPaddleIdle = True
        self.mPaddleMotionEnergy = 0.0
        if self.mIdleGateEnabled:
            self.mGyroMagnitudeBuffer = np.zeros(self.mIdleGateWindowSamples,
                                                 dtype=np.float64)
            self.mGyroMagnitudeFillCount = 0
        else:
            # Gate disabled (old metadata): never report idle, behave as before.
            self.mPaddleIdle = False

        # Inference rate is tied to incoming sample rate (~20 Hz). To keep the
        # log from getting spammy, only emit INFO predictions every N samples;
        # DEBUG still fires every time.
        self.mInferenceCounter = 0
        self.mInferenceLogInterval = self.mConfigurator.get('inferenceLogIntervalSamples', 5)

        self.mLogger.info('ML Manager process running')
        while not self.mShutdownEvent.is_set():
            try:
                payload = self.mIncomingQueue.get(timeout=self.mQueueReadTimeoutSeconds)
            except queue.Empty:
                continue
            try:
                self._ingestSample(payload)
            except Exception as e:
                self.mLogger.error('Exception in ML inference: %s', e, exc_info=True)

        self.mInfluxWriter.close()
        self.mLogger.info('ML Manager process exiting')
