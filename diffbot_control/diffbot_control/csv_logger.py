"""Tiny CSV logger so controller runs can be plotted (scripts/plot_pid_log.py)."""

import csv
import os


class CsvLogger:
    def __init__(self, path: str, header) -> None:
        directory = os.path.dirname(os.path.abspath(path))
        os.makedirs(directory, exist_ok=True)
        self._file = open(path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(header)

    def write(self, row) -> None:
        self._writer.writerow(row)
        self._file.flush()

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()
