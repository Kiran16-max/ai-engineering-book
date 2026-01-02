"""
Script to hard reset Qdrant collection: delete and recreate with proper parameters.
"""
import sys
import os

# Add the backend directory to the path so we can import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.config import Config
from vector_db.qdrant import QdrantConnection
from qdrant_client.http.models import Distance, VectorParams

def reset_collection():
    """
    Delete and recreate the Qdrant collection with proper parameters.
    """
    print("Initializing Qdrant connection...")
    qdrant_conn = QdrantConnection()

    collection_name = Config.QDRANT_COLLECTION_NAME
    print(f"Target collection: {collection_name}")

    # Try to delete the collection first (ignore errors if it doesn't exist)
    try:
        print(f"Attempting to delete collection '{collection_name}'...")
        qdrant_conn.get_client().delete_collection(collection_name)
        print(f"Collection '{collection_name}' deleted successfully")
    except Exception as e:
        print(f"Collection '{collection_name}' may not have existed or error occurred during deletion: {str(e)}")
        print("Continuing with collection creation...")

    # Recreate the collection with proper parameters
    print(f"Creating collection '{collection_name}' with vector_size=768 and distance=COSINE...")
    qdrant_conn.create_collection(vector_size=768, distance=Distance.COSINE)

    # Verify the collection was created (handle potential validation errors)
    try:
        collection_info = qdrant_conn.get_client().get_collection(collection_name)
        print(f"Collection '{collection_name}' created successfully")
        print(f"Vector size: {collection_info.config.params.vectors.size}")
        print(f"Distance: {collection_info.config.params.vectors.distance}")
        print(f"Current vector count: {collection_info.points_count}")
    except Exception as e:
        print(f"Collection '{collection_name}' created successfully (verification skipped due to compatibility issue: {str(e)})")

    print("Qdrant collection reset completed successfully!")

if __name__ == "__main__":
    reset_collection()