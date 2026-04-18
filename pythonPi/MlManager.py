import json
import time
import struct
import threading
import queue
from pathlib import Path

import numpy as np
import onnxruntime as ort


class MlManager(threading.Thread):
    """Runs the stroke-detector ONNX model on incoming oar IMU samples.

    Maintains a rolling buffer of the last N samples (N = model window size).
    On every new sample the buffer shifts left, the new sample is appended,
    and inference runs once the buffer is fully populated. Predictions are
    logged and published to InfluxDB so they can be watched live in Grafana.

    Inputs arrive from RfManager as 6-float packets in the training CSV order:
    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z.

    For now the predicted class / stroke probability is only logged — the motor
    does not yet react. The strokeOutQueue placeholder is wired for the future
    MotorManager integration.
    """

    def __init__(self, configDictionary, logger,
                 oarRawQueue, strokeOutQueue=None, influxWriter=None):
        super().__init__()
        self.mLogger = logger
        self.mInfluxWriter = influxWriter
        self.mLogger.info('**** ML Manager Starting Up *****')
        self.mLogger.info('*************************************')

        self.mIncomingQueue = oarRawQueue
        self.mStrokeOutQueue = strokeOutQueue   # reserved for future motor integration
        self.mShutdownRequested = False
        self.mConfigurator = configDictionary

        # Resolve artifact paths relative to this file so the Pi can reference the
        # mlTraining/checkpoints directory without hard-coding the user's home path.
        baseDir = Path(__file__).parent
        modelPath = (baseDir / self.mConfigurator['modelPath']).resolve()
        metaPath = (baseDir / self.mConfigurator['modelMetaPath']).resolve()
        normStatsPath = (baseDir / self.mConfigurator['normStatsPath']).resolve()

        # Model metadata sidecar describes the model's input shape and class labels
        # so the Pi never duplicates training-side constants.
        with open(metaPath) as f:
            self.mMeta = json.load(f)
        self.mInputName = self.mMeta['input_name']
        self.mOutputName = self.mMeta['output_name']
        self.mNumChannels = self.mMeta['num_channels']
        self.mWindowSize = self.mMeta['window_size']
        self.mClassNames = self.mMeta['class_names']
        self.mLogger.info('Loaded model metadata from %s', metaPath.name)
        self.mLogger.info(
            '  input_name=%s output_name=%s num_channels=%d window_size=%d',
            self.mInputName, self.mOutputName, self.mNumChannels, self.mWindowSize)
        self.mLogger.info('  class_names=%s', self.mClassNames)
        self.mLogger.info(
            '  trained epoch=%d validation_accuracy=%.4f',
            self.mMeta['checkpoint_epoch'],
            self.mMeta['checkpoint_validation_accuracy'])

        # Per-channel mean / standard deviation computed from the training set.
        # Same preprocessing the model saw during training must be applied here.
        with open(normStatsPath) as f:
            stats = json.load(f)
        self.mChannelMeans = np.array(stats['means'], dtype=np.float32)
        self.mChannelStandardDeviations = np.array(stats['stds'], dtype=np.float32)
        self.mLogger.info('Loaded normalization statistics from %s', normStatsPath.name)

        # Load ONNX model into an onnxruntime inference session
        self.mSession = ort.InferenceSession(
            modelPath.as_posix(),
            providers=['CPUExecutionProvider'],
        )
        self.mLogger.info('Loaded ONNX model from %s', modelPath.name)

        # Rolling buffer shaped (num_channels, window_size). Newest sample lives at [:, -1].
        # Seeded to zeros; we report "warming up" until it has been fully filled.
        self.mBuffer = np.zeros((self.mNumChannels, self.mWindowSize), dtype=np.float32)
        self.mBufferFillCount = 0

        # Inference rate is tied to incoming sample rate (~20 Hz). To keep the log
        # from getting spammy, only emit INFO predictions every N samples; DEBUG still fires every time.
        self.mInferenceCounter = 0
        self.mInferenceLogInterval = self.mConfigurator.get('inferenceLogIntervalSamples', 5)

        # Queue read timeout — short enough that shutdown is responsive
        self.mQueueReadTimeoutSeconds = 0.2

    def shutdown(self):
        self.mLogger.info('ML shutdown requested')
        self.mShutdownRequested = True

    def _IngestSample(self, payload):
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
        (logits,) = self.mSession.run(None, {self.mInputName: modelInput})
        latencyMilliseconds = (time.perf_counter() - start) * 1000.0

        # Softmax converts unnormalized "logits" (raw model scores) into
        # probabilities in [0, 1] that sum to 1.
        rawLogits = logits[0]
        shifted = rawLogits - rawLogits.max()
        expo = np.exp(shifted)
        probabilities = expo / expo.sum()
        predictedIndex = int(np.argmax(rawLogits))
        predictedName = self.mClassNames[predictedIndex]

        # Pull the stroke-class probability out as a convenient scalar for the
        # motor side. If the model is ever reshaped without a "stroke" class,
        # fall back to the max-probability value.
        if 'stroke' in self.mClassNames:
            strokeProbability = float(probabilities[self.mClassNames.index('stroke')])
        else:
            strokeProbability = float(probabilities.max())

        self.mInferenceCounter += 1
        if self.mInferenceCounter % self.mInferenceLogInterval == 0:
            probabilityReport = ' '.join(
                f'{name}={prob:.3f}' for name, prob in zip(self.mClassNames, probabilities))
            self.mLogger.info(
                'Prediction: %s (%s) inference_latency=%.2fms',
                predictedName, probabilityReport, latencyMilliseconds)
        else:
            self.mLogger.debug(
                'Prediction: %s (stroke_probability=%.3f) inference_latency=%.2fms',
                predictedName, strokeProbability, latencyMilliseconds)

        if self.mInfluxWriter:
            fields = {
                'stroke_probability': strokeProbability,
                'predicted_class_index': predictedIndex,
                'inference_latency_milliseconds': latencyMilliseconds,
            }
            for name, prob in zip(self.mClassNames, probabilities):
                fields[f'probability_{name}'] = float(prob)
            self.mInfluxWriter.write_point(
                'stroke_detector', fields,
                tags={'predicted_class': predictedName})

    def run(self):
        self.mLogger.info('ML Manager thread running')
        while not self.mShutdownRequested:
            try:
                payload = self.mIncomingQueue.get(timeout=self.mQueueReadTimeoutSeconds)
            except queue.Empty:
                continue
            try:
                self._IngestSample(payload)
            except Exception as e:
                self.mLogger.error('Exception in ML inference: %s', e, exc_info=True)
        self.mLogger.info('ML Manager thread exiting')
