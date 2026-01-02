"""
Script to test retrieval directly from rag_system with specified queries.
"""
import sys
import os

# Add the backend directory to the path so we can import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from rag_system import RAGSystem

def test_retrieval():
    """
    Test retrieval with specified queries.
    """
    print("Initializing RAG System...")
    rag_system = RAGSystem()
    
    # Test queries
    test_queries = [
        "what is ai",
        "tell me about AI", 
        "robots kis kaam aate hain"
    ]
    
    print("\nRunning retrieval tests...")
    
    for i, query in enumerate(test_queries, 1):
        print(f"\n--- Test {i}: '{query}' ---")
        
        try:
            # Test retrieval only
            print(f"Testing retrieval for: {query}")
            context_chunks = rag_system.retrieve_relevant_chunks(query, top_k=3)
            
            print(f"Number of chunks retrieved: {len(context_chunks)}")
            
            if len(context_chunks) > 0:
                print("Chunks retrieved successfully:")
                for j, chunk in enumerate(context_chunks):
                    print(f"  Chunk {j+1} (score: {chunk['score']:.3f}): {chunk['text'][:100]}...")
                    print(f"    Source: {chunk['source']}, Chunk ID: {chunk['chunk_id']}")
                
                # Test full answer generation
                print(f"Testing full answer generation for: {query}")
                result = rag_system.answer_question(query, top_k=3)
                
                print(f"Answer: {result['answer'][:200]}{'...' if len(result['answer']) > 200 else ''}")
                print(f"Number of sources used: {result['num_sources']}")
                
                # Check if answer is "Not covered"
                if result['answer'] == "Not covered":
                    print("  WARNING: Query returned 'Not covered'")
                else:
                    print("  SUCCESS: Query returned relevant answer")
            else:
                print("  ERROR: No chunks retrieved for this query")
                
        except Exception as e:
            print(f"  ERROR: {str(e)}")
    
    print("\nRetrieval tests completed!")

if __name__ == "__main__":
    test_retrieval()