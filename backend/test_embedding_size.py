"""
Test script to verify the embedding size of the text-embedding-004 model
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.config import Config
import google.generativeai as genai

# Configure the API key
genai.configure(api_key=Config.GEMINI_API_KEY)

# Test the embedding model
model = "text-embedding-004"
test_text = "This is a test sentence to check embedding dimensions."

try:
    response = genai.embed_content(
        model=model,
        content=test_text,
        task_type="retrieval_document",
        title="Test"
    )
    
    embedding = response['embedding']
    print(f"Model: {model}")
    print(f"Embedding size: {len(embedding)}")
    print(f"First 5 values: {embedding[:5]}")
    
except Exception as e:
    print(f"Error: {e}")