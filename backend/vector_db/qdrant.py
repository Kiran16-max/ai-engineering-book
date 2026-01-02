"""
Qdrant vector database connection module.
"""
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams
from config.config import Config
import logging

logger = logging.getLogger(__name__)

class QdrantConnection:
    """
    Class to handle Qdrant database connections and operations.
    """
    def __init__(self):
        """
        Initialize Qdrant client with configuration from environment variables.
        """
        if not Config.QDRANT_URL or not Config.QDRANT_API_KEY:
            raise ValueError("Qdrant URL and API key must be set in environment variables")
        
        self.client = QdrantClient(
            url=Config.QDRANT_URL,
            api_key=Config.QDRANT_API_KEY,
            timeout=60
        )
        self.collection_name = Config.QDRANT_COLLECTION_NAME
        
    def get_client(self):
        """
        Return the Qdrant client instance.
        """
        return self.client
        
    def get_collection_name(self):
        """
        Return the collection name.
        """
        return self.collection_name
        
    def collection_exists(self):
        """
        Check if the collection exists.
        """
        try:
            self.client.get_collection(self.collection_name)
            return True
        except:
            return False
            
    def create_collection(self, vector_size: int = 1024, distance: Distance = Distance.COSINE):
        """
        Create a new collection with specified parameters.
        """
        if self.collection_exists():
            logger.info(f"Collection '{self.collection_name}' already exists")
            return
            
        self.client.create_collection(
            collection_name=self.collection_name,
            vectors_config=VectorParams(size=vector_size, distance=distance),
        )
        logger.info(f"Collection '{self.collection_name}' created successfully")