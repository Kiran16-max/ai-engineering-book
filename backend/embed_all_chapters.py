"""
Script to process all book chapters: read, chunk, embed, and store in Qdrant.
"""
import sys
import os
import logging
from typing import List
import uuid

# Add the backend directory to the path so we can import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.config import Config
from embeddings.gemini import GeminiEmbedder
from vector_db.qdrant import QdrantConnection
from embeddings.chunker import chunk_text
from qdrant_client.http.models import PointStruct, Distance

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def read_all_chapters():
    """
    Read all chapter files from the chapters directory.
    """
    chapters_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "chapters")
    all_content = []
    
    if not os.path.exists(chapters_dir):
        logger.error(f"Chapters directory not found at path: {chapters_dir}")
        return []
    
    chapter_files = [f for f in os.listdir(chapters_dir) if f.endswith('.md')]
    chapter_files.sort()  # Sort to maintain order
    
    logger.info(f"Found {len(chapter_files)} chapter files to process")
    
    for filename in chapter_files:
        filepath = os.path.join(chapters_dir, filename)
        try:
            with open(filepath, 'r', encoding='utf-8') as file:
                content = file.read()
                all_content.append({
                    'content': content,
                    'source': filename
                })
            logger.info(f"Read chapter: {filename} ({len(content)} characters)")
        except Exception as e:
            logger.error(f"Error reading chapter {filename}: {e}")
    
    return all_content

def process_all_chapters():
    """
    Process all book chapters: read, chunk, embed, and store in Qdrant.
    """
    # Validate configuration
    try:
        Config.validate()
        logger.info("Configuration validated successfully")
    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        return

    # Print the collection name being used
    collection_name = Config.QDRANT_COLLECTION_NAME
    logger.info(f"Using Qdrant collection: {collection_name}")

    # Initialize components
    logger.info("Initializing components...")
    embedder = GeminiEmbedder()
    qdrant_conn = QdrantConnection()

    # Read all chapter content
    logger.info("Reading all chapter content...")
    chapters_data = read_all_chapters()
    
    if not chapters_data:
        logger.error("No chapter content found to process")
        return

    # Combine all content and chunk it
    all_text = ""
    source_mapping = []  # Track which source each chunk came from
    
    for chapter_data in chapters_data:
        chapter_text = chapter_data['content']
        source = chapter_data['source']
        
        # Add to combined text with source tracking
        start_pos = len(all_text)
        all_text += chapter_text + "\n\n"  # Add spacing between chapters
        end_pos = len(all_text)
        
        # Track the source for this range
        source_mapping.append({
            'start': start_pos,
            'end': end_pos,
            'source': source
        })

    logger.info(f"All chapters combined. Total characters: {len(all_text)}")

    # Chunk the combined content
    logger.info("Chunking all chapter content...")
    chunks = chunk_text(
        text=all_text,
        chunk_size=1000,
        overlap=200,
        min_chunk_size=100
    )
    logger.info(f"Content chunked into {len(chunks)} chunks")

    # Print sample of first 3 chunks (first 100 chars each)
    logger.info("Sample of first 3 chunks (first 100 chars):")
    for i, chunk in enumerate(chunks[:3]):
        sample = chunk[:100] + "..." if len(chunk) > 100 else chunk
        logger.info(f"  Chunk {i+1}: {sample}")

    # Determine which source each chunk came from
    chunk_sources = []
    for chunk in chunks:
        # Find which source this chunk belongs to
        chunk_start = all_text.find(chunk)
        if chunk_start != -1:
            chunk_end = chunk_start + len(chunk)
            
            # Find the source that contains this chunk
            source_found = False
            for source_info in source_mapping:
                if source_info['start'] <= chunk_start < source_info['end']:
                    chunk_sources.append(source_info['source'])
                    source_found = True
                    break
            
            if not source_found:
                chunk_sources.append("unknown")
        else:
            chunk_sources.append("unknown")

    # Generate embeddings in smaller batches to avoid timeout
    logger.info("Generating embeddings for chunks in batches...")
    embeddings = []
    batch_size = 10  # Smaller batch size to avoid timeout

    try:
        for i in range(0, len(chunks), batch_size):
            batch = chunks[i:i + batch_size]
            logger.info(f"Processing embedding batch {i//batch_size + 1} ({len(batch)} chunks)...")
            batch_embeddings = embedder.generate_embeddings(batch)
            embeddings.extend(batch_embeddings)
            logger.info(f"Completed batch {i//batch_size + 1}, total embeddings so far: {len(embeddings)}")

        logger.info(f"Generated {len(embeddings)} embeddings total")
    except Exception as e:
        logger.error(f"Error generating embeddings: {e}")
        return

    # Validate embeddings and prepare points for Qdrant
    points = []
    # Determine actual vector size from first embedding if available
    expected_vector_size = len(embeddings[0]) if embeddings else 768  # Default to 768 if no embeddings
    logger.info(f"Expected vector size: {expected_vector_size}")

    for i, (chunk, embedding, source) in enumerate(zip(chunks, embeddings, chunk_sources)):
        # Validate vector length
        if len(embedding) != expected_vector_size:
            logger.error(f"Embedding {i} has incorrect size: {len(embedding)}, expected: {expected_vector_size}")
            continue  # Skip invalid embeddings

        point = PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={
                "text": chunk, 
                "chunk_id": i, 
                "source": source
            }
        )
        points.append(point)

    logger.info(f"Validated {len(points)} embeddings with correct vector size ({expected_vector_size})")

    # Connect to Qdrant and create collection if it doesn't exist
    logger.info(f"Connecting to Qdrant collection: {collection_name}")

    # Create collection with vector size (text-embedding-004 = 768)
    try:
        qdrant_conn.create_collection(vector_size=expected_vector_size, distance=Distance.COSINE)
    except Exception as e:
        logger.error(f"Error creating Qdrant collection: {e}")
        return

    # Clear existing collection to re-index
    try:
        logger.info(f"Clearing existing collection: {collection_name}")
        qdrant_conn.get_client().delete_collection(collection_name)
        logger.info("Collection cleared")
    except Exception as e:
        logger.info(f"Collection didn't exist or couldn't be cleared (this is OK): {e}")
    
    # Recreate the collection
    try:
        qdrant_conn.create_collection(vector_size=expected_vector_size, distance=Distance.COSINE)
    except Exception as e:
        logger.error(f"Error recreating Qdrant collection: {e}")
        return

    # Store embeddings in Qdrant (batched)
    try:
        batch_size = 50
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            qdrant_conn.get_client().upsert(
                collection_name=collection_name,
                points=batch
            )
            logger.info(f"Uploaded batch {i//batch_size + 1} ({len(batch)} points) to Qdrant")
    except Exception as e:
        logger.error(f"Error storing embeddings in Qdrant: {e}")
        return

    logger.info("All embeddings stored successfully in Qdrant!")

    # Verify upload
    try:
        collection_info = qdrant_conn.get_client().get_collection(collection_name)
        logger.info(f"Collection '{collection_name}' now contains {collection_info.points_count} vectors")
    except Exception as e:
        logger.error(f"Error verifying collection: {e}")

    # Summary
    logger.info("\n" + "="*50)
    logger.info("CHAPTER PROCESSING COMPLETE")
    logger.info(f"Total chapters processed: {len(chapters_data)}")
    logger.info(f"Total chunks: {len(chunks)}")
    logger.info(f"Embeddings stored in Qdrant collection: {collection_name}")
    logger.info("="*50)

if __name__ == "__main__":
    process_all_chapters()