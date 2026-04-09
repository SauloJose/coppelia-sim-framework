"""Sistema profissional de logging padronizado para CoppeliaSim framework."""

import logging


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


def setup_logger(name, origin_prefix='[APP]'):
    """
    Cria e configura um logger profissional.
    
    Args:
        name: Nome do logger (tipicamente __name__)
        origin_prefix: [MAIN] ou [APP]
    
    Returns:
        Logger configurado
    """
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    
    # Remover handlers anteriores para evitar duplicação
    logger.handlers.clear()
    
    # Criar handler para console
    handler = logging.StreamHandler()
    handler.setLevel(logging.DEBUG)
    handler.setFormatter(ProfessionalFormatter(origin_prefix))
    
    logger.addHandler(handler)
    return logger
