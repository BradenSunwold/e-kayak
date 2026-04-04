import json
import logging
from datetime import datetime
from influxdb_client import InfluxDBClient, WriteOptions

logger = logging.getLogger("InfluxWriter")


class InfluxWriter:
    """Thin wrapper around influxdb-client for writing telemetry points.

    Usage:
        writer = InfluxWriter(config)
        writer.write_point("some_measurement", {"field1": 1.2, "field2": 0.5}, tags={"tag1": "value"})
        writer.close()
    """

    def __init__(self, config):
        """Initialize from config dict or path to JSON config file.

        Config keys:
            url, org, bucket, token       — required
            batchSize (int, default 10)   — points buffered before flush
            flushIntervalMs (int, default 50)  — max ms between flushes
            enabled (bool, default True)  — set False to disable writes
        """
        if isinstance(config, str):
            with open(config, "r") as f:
                config = json.load(f)

        self._enabled = config.get("enabled", True)
        if not self._enabled:
            logger.info("InfluxDB writes disabled by config")
            return

        self._org = config["org"]
        self._bucket = config["bucket"]
        batch_size = config.get("batchSize", 10)
        flush_interval = config.get("flushIntervalMs", 50)

        self._client = InfluxDBClient(
            url=config["url"],
            token=config["token"],
            org=self._org,
        )

        self._write_api = self._client.write_api(
            write_options=WriteOptions(
                batch_size=batch_size,
                flush_interval=flush_interval,
            )
        )

        # Session tag lets you filter Grafana dashboards to a specific run
        self._session = datetime.now().strftime('%Y-%m-%d_%H:%M:%S')

        logger.info(
            "InfluxDB writer ready (bucket=%s, batch=%d, flush=%dms, session=%s)",
            self._bucket, batch_size, flush_interval, self._session,
        )

    def write_point(self, measurement, fields, tags=None):
        """Write a single data point.

        Args:
            measurement: InfluxDB measurement name (any string)
            fields: dict of field_name -> numeric value
            tags: optional dict of tag_name -> string value
        """
        if not self._enabled:
            return

        all_tags = {"session": self._session}
        if tags:
            all_tags.update(tags)

        record = {"measurement": measurement, "fields": fields, "tags": all_tags}

        self._write_api.write(bucket=self._bucket, record=record)

    def close(self):
        """Flush remaining points and close the connection."""
        if not self._enabled:
            return
        self._write_api.close()
        self._client.close()
        logger.info("InfluxDB writer closed")
