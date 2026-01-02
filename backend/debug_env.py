import os
from dotenv import load_dotenv

# Check the backend .env file
backend_env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.env')
print(f"Backend .env path: {backend_env_path}")
print(f"Backend .env exists: {os.path.exists(backend_env_path)}")

if os.path.exists(backend_env_path):
    with open(backend_env_path, 'r') as f:
        print("Backend .env content:")
        print(f.read())

# Check the root .env file
root_env_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), '.env')
print(f"\nRoot .env path: {root_env_path}")
print(f"Root .env exists: {os.path.exists(root_env_path)}")

if os.path.exists(root_env_path):
    with open(root_env_path, 'r') as f:
        print("Root .env content:")
        print(f.read())

# Load backend .env specifically
print("\nLoading backend .env specifically...")
load_dotenv(backend_env_path)
print(f"QDRANT_COLLECTION_NAME after loading backend .env: {os.getenv('QDRANT_COLLECTION_NAME')}")

# Now load root .env to see if it overrides
print("\nLoading root .env to see if it overrides...")
load_dotenv(root_env_path)
print(f"QDRANT_COLLECTION_NAME after loading root .env: {os.getenv('QDRANT_COLLECTION_NAME')}")