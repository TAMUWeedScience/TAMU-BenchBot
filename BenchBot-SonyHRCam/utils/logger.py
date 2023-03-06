import logging


class CustomFormatter(logging.Formatter):

    grey = "\x1b[38;20m"
    yellow = "\x1b[33:20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format = '[%(asctime)s] %(levelname)6s - %(message)s ' + '(%(filename)s:%(lineno)d)'

    FORMATS = {
        logging.DEBUG: grey + format + reset,
        logging.INFO: grey + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt, datefmt='%m/%d/%Y %I:%M:%S %p')
        return formatter.format(record)


logFormatter = logging.Formatter(
    '[%(asctime)s] %(levelname)6s - %(message)s ' +
    '(%(filename)s:%(lineno)d)',
    datefmt='%m/%d/%Y %I:%M:%S %p')

log = logging.getLogger()
log.setLevel(logging.DEBUG)

fileHandler = logging.FileHandler('LOG.log')
fileHandler.setFormatter(CustomFormatter())
# fileHandler.setFormatter(logFormatter)
log.addHandler(fileHandler)

consoleHandler = logging.StreamHandler()
consoleHandler.setFormatter(CustomFormatter())
# consoleHandler.setFormatter(logFormatter)
log.addHandler(consoleHandler)
