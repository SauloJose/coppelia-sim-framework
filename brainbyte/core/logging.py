"""Sistema profissional de logging padronizado para CoppeliaSim framework."""

import logging
import os 


class ProfessionalFormatter(logging.Formatter):
    """Formatter profissional para logs sem emojis e com prefixos claros."""
    
    LEVEL_NAMES = {
        logging.DEBUG: '[DEBUG]',
        logging.INFO: '[INFO]',
        logging.WARNING: '[WARNING]',
        logging.ERROR: '[ERROR]',
        logging.CRITICAL: '[CRITICAL]'
    }
    
    def __init__(self, origin_prefix='[APP]'):
        """
        Args:
            origin_prefix: Prefixo para indicar origem ([MAIN] ou [APP])
        """
        self.origin_prefix = origin_prefix
        super().__init__()
    
    def format(self, record):
        """Formata o log com prefixos profissionais."""
        level_name = self.LEVEL_NAMES.get(record.levelno, '[INFO]')
        timestamp = self.formatTime(record, '%H:%M:%S')
        message = record.getMessage()
        return f"{level_name} {self.origin_prefix} [{timestamp}] {message}"


def setup_logger(name, origin_prefix='[APP]', log_file=None):
    """
    Cria e configura um logger profissional.
    
    Args:
        name: Nome do logger (tipicamente __name__)
        origin_prefix: [MAIN] ou [APP]
        log_file: Caminho opcional para um arquivo de log.
                  Se fornecido, os logs também serão escritos nesse arquivo.
    
    Returns:
        Logger configurado
    """
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    logger.handlers.clear()
    
    # Formatter comum
    formatter = ProfessionalFormatter(origin_prefix)
    
    # Handler de console (existente)
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.DEBUG)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # Handler de arquivo (novo)
    if log_file:
        os.makedirs(os.path.dirname(log_file), exist_ok=True)
        file_handler = logging.FileHandler(log_file, encoding='utf-8')
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    
    return logger