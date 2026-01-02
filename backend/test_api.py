"""
Script to test the FastAPI server with the same queries.
"""
import requests
import time
import sys
import os

def test_api():
    """
    Test the FastAPI server with the same queries.
    """
    # Wait a bit for the server to start
    print("Waiting for server to start...")
    time.sleep(8)
    
    # API endpoint
    url = "http://127.0.0.1:8000/api/v1/chat"
    
    # Test queries
    test_queries = [
        "what is ai",
        "tell me about AI", 
        "robots kis kaam aate hain"
    ]
    
    print("Testing FastAPI server with queries...")
    
    for i, query in enumerate(test_queries, 1):
        print(f"\n--- API Test {i}: '{query}' ---")
        
        try:
            response = requests.post(
                url,
                json={"query": query},
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                print(f"Status: {response.status_code}")
                print(f"Answer: {result.get('answer', 'No answer field')[:200]}{'...' if len(result.get('answer', '')) > 200 else ''}")
                print(f"Sources used: {result.get('num_sources', 'N/A')}")
                
                # Check if answer is "Not covered"
                if result.get('answer') == "Not covered":
                    print("  WARNING: Query returned 'Not covered'")
                else:
                    print("  SUCCESS: Query returned relevant answer")
            else:
                print(f"Error: Status code {response.status_code}")
                print(f"Response: {response.text}")
                
        except requests.exceptions.ConnectionError:
            print("Error: Could not connect to server. Make sure it's running on http://127.0.0.1:8000")
        except Exception as e:
            print(f"Error: {str(e)}")
    
    print("\nAPI tests completed!")

if __name__ == "__main__":
    test_api()