"""
Configuration module for loading environment variables.
"""
import os
from dotenv import load_dotenv

# Load environment variables from .env file in the backend directory
# First try to load the backend .env file specifically
backend_env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '.env')
backend_env_path = os.path.abspath(backend_env_path)
if os.path.exists(backend_env_path):
    load_dotenv(backend_env_path, override=True)  # Override any existing environment variables
else:
    # Fall back to loading from the project root
    load_dotenv(override=True)

class Config:
    """
    Configuration class to manage environment variables.
    """
    # Gemini configuration
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

    # Qdrant configuration
    QDRANT_URL = os.getenv("QDRANT_URL")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "new_book_vectors")

    # Groq configuration
    GROQ_API_KEY = os.getenv("GROQ_API_KEY")

    # Validation
    @classmethod
    def validate(cls):
        """
        Validate that all required environment variables are set.
        """
        required_vars = [
            'GEMINI_API_KEY',
            'QDRANT_URL',
            'QDRANT_API_KEY',
            'QDRANT_COLLECTION_NAME',
            'GROQ_API_KEY'
        ]

        missing_vars = []
        for var in required_vars:
            value = getattr(cls, var)
            if not value:
                missing_vars.append(var)

        if missing_vars:
            raise ValueError(f"Missing required environment variables: {', '.join(missing_vars)}")