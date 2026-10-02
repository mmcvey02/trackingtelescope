"""Atomic JSON persistence for the map document."""

import json
import os
import tempfile
import threading
import time

FORMAT_VERSION = 2


class MapStore:
    def __init__(self, path):
        self.path = os.path.abspath(path)
        self._lock = threading.Lock()

    def load(self):
        """Return the saved document, or None if there is no usable map yet.

        A map written by an older version is moved aside to ``<name>.v1.bak``
        rather than overwritten.
        """
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                doc = json.load(fh)
        except FileNotFoundError:
            return None
        if doc.get("format") == FORMAT_VERSION:
            return doc
        if doc.get("format") == 1:
            os.replace(self.path, self.path + ".v1.bak")
            return None
        raise ValueError(f"{self.path}: unsupported map format {doc.get('format')!r}")

    def save(self, doc):
        doc = dict(doc, format=FORMAT_VERSION, saved_at=time.time())
        directory = os.path.dirname(self.path) or "."
        os.makedirs(directory, exist_ok=True)
        with self._lock:
            fd, tmp = tempfile.mkstemp(prefix=".roomba-map-", suffix=".tmp", dir=directory)
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as fh:
                    json.dump(doc, fh, separators=(",", ":"))
                    fh.flush()
                    os.fsync(fh.fileno())
                os.replace(tmp, self.path)
            except BaseException:
                if os.path.exists(tmp):
                    os.unlink(tmp)
                raise
