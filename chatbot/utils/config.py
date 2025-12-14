import os
from dotenv import load_dotenv
from typing import Optional


class Config:
    """
    Configuration class to manage environment variables for both frontend and backend
    """
    
    def __init__(self):
        # Load environment variables from .env file
        load_dotenv()
    
    # Database configurations
    @property
    def QDRANT_URL(self) -> str:
        return os.getenv('QDRANT_URL', 'http://localhost:6333')
    
    @property
    def QDRANT_API_KEY(self) -> Optional[str]:
        return os.getenv('QDRANT_API_KEY')
    
    @property
    def QDRANT_COLLECTION_NAME(self) -> str:
        return os.getenv('QDRANT_COLLECTION_NAME', 'textbook_content')
    
    # Application settings
    @property
    def DEBUG(self) -> bool:
        return os.getenv('DEBUG', 'False').lower() == 'true'
    
    @property
    def LOG_LEVEL(self) -> str:
        return os.getenv('LOG_LEVEL', 'INFO')
    
    # API settings
    @property
    def API_HOST(self) -> str:
        return os.getenv('API_HOST', '0.0.0.0')
    
    @property
    def API_PORT(self) -> int:
        return int(os.getenv('API_PORT', '8000'))
    
    # Content settings
    @property
    def MIN_PEER_REVIEWED_SOURCES(self) -> float:
        """Minimum percentage of peer-reviewed sources required (0.0-1.0)"""
        return float(os.getenv('MIN_PEER_REVIEWED_SOURCES', '0.5'))
    
    @property
    def MIN_TOTAL_SOURCES(self) -> int:
        """Minimum number of total sources required"""
        return int(os.getenv('MIN_TOTAL_SOURCES', '25'))
    
    @property
    def MIN_WORDS(self) -> int:
        """Minimum word count for the textbook"""
        return int(os.getenv('MIN_WORDS', '15000'))
    
    @property
    def MAX_WORDS(self) -> int:
        """Maximum word count for the textbook"""
        return int(os.getenv('MAX_WORDS', '20000'))
    
    # Language settings
    @property
    def SUPPORTED_LANGUAGES(self) -> str:
        """Comma-separated list of supported languages"""
        return os.getenv('SUPPORTED_LANGUAGES', 'en,ur')


# Create a global config instance
config = Config()