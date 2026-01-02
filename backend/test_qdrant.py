"""
Test script to verify Qdrant connection and collection creation
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.config import Config
from vector_db.qdrant import QdrantConnection
from qdrant_client.http.models import Distance

# Test the Qdrant connection
try:
    print(f"Using collection name: {Config.QDRANT_COLLECTION_NAME}")
    
    qdrant_conn = QdrantConnection()
    print("Successfully connected to Qdrant")
    
    # Check if collection exists
    if qdrant_conn.collection_exists():
        print(f"Collection '{Config.QDRANT_COLLECTION_NAME}' already exists")
    else:
        print(f"Collection '{Config.QDRANT_COLLECTION_NAME}' does not exist, creating it...")
        # Create collection with appropriate vector size for text-embedding-004 (768 dimensions)
        qdrant_conn.create_collection(vector_size=768, distance=Distance.COSINE)
        print(f"Collection '{Config.QDRANT_COLLECTION_NAME}' created successfully")
    
    # Get collection info
    collection_info = qdrant_conn.get_client().get_collection(Config.QDRANT_COLLECTION_NAME)
    print(f"Collection '{Config.QDRANT_COLLECTION_NAME}' contains {collection_info.points_count} vectors")
    
    print("Qdrant connection and collection verification successful!")
    
except Exception as e:
    print(f"Error connecting to Qdrant or working with collection: {e}")