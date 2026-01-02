import os
import sys
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)

# Add backend directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.config import Config
from vector_db.qdrant import QdrantConnection

def check_qdrant_methods():
    print("Checking Qdrant client methods...")
    
    # Validate config
    try:
        Config.validate()
        print("[OK] Configuration validated")
    except Exception as e:
        print(f"[ERROR] Configuration error: {e}")
        return
    
    # Initialize Qdrant connection
    try:
        qdrant_conn = QdrantConnection()
        client = qdrant_conn.get_client()
        print("[OK] Qdrant connection established")
        
        # Check available methods
        methods = [method for method in dir(client) if not method.startswith('_')]
        print(f"Available methods: {methods}")
        
        # Check specifically for search method
        if 'search' in methods:
            print("✓ 'search' method is available")
        else:
            print("✗ 'search' method is NOT available")
            
        # Check for other potential search-related methods
        search_related = [method for method in methods if 'search' in method.lower() or 'find' in method.lower() or 'query' in method.lower()]
        print(f"Search-related methods: {search_related}")
        
        # Check if points-related methods exist
        points_related = [method for method in methods if 'point' in method.lower()]
        print(f"Points-related methods: {points_related}")
        
    except Exception as e:
        print(f"[ERROR] Qdrant connection error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    check_qdrant_methods()