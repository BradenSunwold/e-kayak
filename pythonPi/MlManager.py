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

    def _ingestSample(self, payload):
        """Unpack one 6-float sample, push into the rolling buffer, run inference."""
        ax, ay, az, gx, gy, gz = struct.unpack('ffffff', payload)
        sample = np.array([ax, ay, az, gx, gy, gz], dtype=np.float32)

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

        # Send the raw prediction to the motor manager for auto-mode RPM control
        if self.mAssistOutQueue is not None:
            self.mAssistOutQueue.put(struct.pack('f', assistPrediction))

        if self.mInfluxWriter:
            # Fields only, no tags — tagging by any per-sample value would
            # split every field into separate series and break Grafana plots.
            self.mInfluxWriter.write_point('assist_model', {
                'assist_prediction': assistPrediction,
                'assist_prediction_physical': assistPredictionPhysical,
                'inference_latency_milliseconds': latencyMilliseconds,
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
