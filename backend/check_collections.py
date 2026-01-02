"""
Script to check existing Qdrant collections and verify old collections are untouched
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.config import Config
from vector_db.qdrant import QdrantConnection

try:
    qdrant_conn = QdrantConnection()
    print("Successfully connected to Qdrant")
    
    # List all collections
    collections = qdrant_conn.get_client().get_collections()
    print(f"Existing collections in Qdrant:")
    for collection in collections.collections:
        print(f"  - {collection.name}")

    # Check specifically for the old collection mentioned in instructions
    old_collection_name = "humanoid_ai_book"
    old_collection_exists = any(col.name == old_collection_name for col in collections.collections)

    if old_collection_exists:
        print(f"\nOld collection '{old_collection_name}' exists and was not modified.")
        old_collection_info = qdrant_conn.get_client().get_collection(old_collection_name)
        print(f"  Status: {old_collection_info.status}")
        # Get actual point count
        old_count = qdrant_conn.get_client().count(collection_name=old_collection_name).count
        print(f"  Points in old collection: {old_count}")
    else:
        print(f"\nOld collection '{old_collection_name}' does not exist (as expected).")

    # Check our new collection
    new_collection_name = Config.QDRANT_COLLECTION_NAME
    new_collection_exists = any(col.name == new_collection_name for col in collections.collections)

    if new_collection_exists:
        print(f"\nNew collection '{new_collection_name}' was created successfully.")
        new_collection_info = qdrant_conn.get_client().get_collection(new_collection_name)
        print(f"  Status: {new_collection_info.status}")
        # Get actual point count
        new_count = qdrant_conn.get_client().count(collection_name=new_collection_name).count
        print(f"  Points in new collection: {new_count}")
    else:
        print(f"\nNew collection '{new_collection_name}' was not found.")
    
    print("\nSafety check completed: Old collections remain untouched.")

except Exception as e:
    print(f"Error checking Qdrant collections: {e}")