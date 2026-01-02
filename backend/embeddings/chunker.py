"""
Text chunking module for splitting book content into manageable pieces.
"""
import re
from typing import List

def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200, min_chunk_size: int = 100) -> List[str]:
    """
    Split text into overlapping chunks.
    
    Args:
        text: The input text to chunk
        chunk_size: Maximum size of each chunk in characters
        overlap: Number of overlapping characters between chunks
        min_chunk_size: Minimum size for a chunk to be included
    
    Returns:
        List of text chunks
    """
    if len(text) <= chunk_size:
        if len(text) >= min_chunk_size:
            return [text]
        else:
            return []
    
    chunks = []
    start = 0
    
    while start < len(text):
        end = start + chunk_size
        
        # If this is the last chunk and it's smaller than min_chunk_size, include it with the previous chunk
        if end >= len(text):
            chunk = text[start:]
            if len(chunk) >= min_chunk_size:
                chunks.append(chunk)
            break
        
        # Extract the chunk
        chunk = text[start:end]
        
        # Try to break at sentence or paragraph boundaries if possible
        if end < len(text):
            # Look for a good breaking point (sentence end, paragraph end, or whitespace)
            remaining_text = text[end-200:end+200]  # Look at a wider context
            sentence_break = re.search(r'[.!?]\s+', remaining_text)
            paragraph_break = re.search(r'\n\s*\n', remaining_text)
            
            if sentence_break:
                break_point = sentence_break.end()
                actual_end = end - 200 + break_point
                chunk = text[start:actual_end]
            elif paragraph_break:
                break_point = paragraph_break.end()
                actual_end = end - 200 + break_point
                chunk = text[start:actual_end]
        
        # Only add chunk if it meets minimum size requirement
        if len(chunk) >= min_chunk_size:
            chunks.append(chunk)
        
        # Move start position with overlap
        start = max(start + chunk_size - overlap, start + 1)  # Ensure we make progress
        
        # If we're not making progress, move by at least one character
        if start >= len(text):
            break
    
    return chunks