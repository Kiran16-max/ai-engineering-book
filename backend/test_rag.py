import os
import sys
import logging

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Add backend directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.config import Config
from rag_system import RAGSystem

def test_rag():
    print("Starting RAG test...")

    # Validate config
    try:
        Config.validate()
        print("[OK] Configuration validated")
    except Exception as e:
        print(f"[ERROR] Configuration error: {e}")
        return

    # Initialize RAG system
    try:
        rag = RAGSystem()
        print("[OK] RAG system initialized")
    except Exception as e:
        print(f"[ERROR] RAG system initialization error: {e}")
        import traceback
        traceback.print_exc()
        return

    # Test query
    try:
        print("Testing query: 'What is Physical AI?'")
        result = rag.answer_question('What is Physical AI?', top_k=3)
        print(f"[OK] Query processed successfully")
        print(f"Query: {result['query']}")
        print(f"Answer length: {len(result['answer'])}")
        print(f"Number of sources: {result['num_sources']}")
        if result['sources']:
            print(f"First source preview: {result['sources'][0]['text'][:100]}...")
            print(f"First source score: {result['sources'][0]['score']}")
    except Exception as e:
        print(f"[ERROR] Query processing error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_rag()