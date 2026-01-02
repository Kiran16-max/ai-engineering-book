"""
Script to clear all vectors from the 'new_book_vectors' collection in Qdrant.
"""
import sys
import os

# Add the backend directory to the path so we can import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.config import Config
from vector_db.qdrant import QdrantConnection

def clear_collection():
    """
    Clear all vectors from the 'new_book_vectors' collection.
    """
    print("Initializing Qdrant connection...")
    qdrant_conn = QdrantConnection()
    
    collection_name = Config.QDRANT_COLLECTION_NAME
    print(f"Connected to collection: {collection_name}")
    
    # Get current count
    collection_info = qdrant_conn.get_client().get_collection(collection_name)
    current_count = collection_info.points_count
    print(f"Current vector count: {current_count}")
    
    if current_count > 0:
        print(f"Clearing all {current_count} vectors from collection...")
        
        # Delete all points in the collection using the correct method
        from qdrant_client.http import models
        qdrant_conn.get_client().delete(
            collection_name=collection_name,
            points_selector=models.FilterSelector(
                filter=models.Filter(
                    must=[]
                )
            )
        )
        
        # Verify deletion
        collection_info_after = qdrant_conn.get_client().get_collection(collection_name)
        new_count = collection_info_after.points_count
        print(f"Vector count after clearing: {new_count}")
        
        print(f"Successfully cleared {current_count} vectors from '{collection_name}' collection.")
    else:
        print(f"Collection '{collection_name}' is already empty.")
    
    print("Clearing process completed.")

if __name__ == "__main__":
    clear_collection()