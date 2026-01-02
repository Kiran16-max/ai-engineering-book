"""
Script to process crawled book content, chunk it, and generate embeddings.
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

def process_crawled_content():
    """
    Process the crawled book content: read, chunk, embed, and store in Qdrant.
    """
    # Validate configuration
    try:
        Config.validate()
        logger.info("Configuration validated successfully")
    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        return

    # Initialize components
    logger.info("Initializing components...")
    embedder = GeminiEmbedder()
    qdrant_conn = QdrantConnection()

    # Read the crawled book content
    logger.info("Reading crawled book content...")
    content_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "crawled_book_content.txt")
    try:
        with open(content_path, 'r', encoding='utf-8') as file:
            book_content = file.read()
        logger.info(f"Crawled book content read successfully. Total characters: {len(book_content)}")
    except FileNotFoundError:
        logger.error(f"Crawled content file not found at path: {content_path}")
        return
    except Exception as e:
        logger.error(f"Error reading crawled content file: {e}")
        return

    # Chunk the book content
    logger.info("Chunking crawled book content...")
    chunks = chunk_text(
        text=book_content,
        chunk_size=1000,
        overlap=200,
        min_chunk_size=100
    )
    logger.info(f"Crawled book chunked into {len(chunks)} chunks")

    # Print sample of first 3 chunks (first 100 chars each)
    logger.info("Sample of first 3 chunks (first 100 chars):")
    for i, chunk in enumerate(chunks[:3]):
        sample = chunk[:100] + "..." if len(chunk) > 100 else chunk
        logger.info(f"  Chunk {i+1}: {sample}")

    # Generate embeddings
    logger.info("Generating embeddings for chunks...")
    try:
        embeddings = embedder.generate_embeddings(chunks)
        logger.info(f"Generated {len(embeddings)} embeddings")
    except Exception as e:
        logger.error(f"Error generating embeddings: {e}")
        return

    # Validate embeddings and prepare points for Qdrant
    points = []
    expected_vector_size = 768  # For text-embedding-004 model
    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        # Validate vector length
        if len(embedding) != expected_vector_size:
            logger.error(f"Embedding {i} has incorrect size: {len(embedding)}, expected: {expected_vector_size}")
            continue  # Skip invalid embeddings

        point = PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={"text": chunk, "chunk_id": i, "source": "crawled_book_content.txt"}
        )
        points.append(point)

    logger.info(f"Validated {len(points)} embeddings with correct vector size ({expected_vector_size})")

    # Clear existing vectors in the collection first
    collection_name = Config.QDRANT_COLLECTION_NAME
    logger.info(f"Clearing existing vectors from collection: {collection_name}")
    
    try:
        # Delete all points in the collection
        from qdrant_client.http import models
        qdrant_conn.get_client().delete(
            collection_name=collection_name,
            points_selector=models.FilterSelector(
                filter=models.Filter(
                    must=[]
                )
            )
        )
        logger.info("Successfully cleared existing vectors from collection")
    except Exception as e:
        logger.error(f"Error clearing collection: {e}")
        # Continue anyway, as this might be a new collection

    # Create collection with vector size (text-embedding-004 = 768)
    try:
        qdrant_conn.create_collection(vector_size=expected_vector_size, distance=Distance.COSINE)
    except Exception as e:
        logger.error(f"Error creating Qdrant collection: {e}")
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
    logger.info("PROCESSING COMPLETE")
    logger.info(f"Total chunks: {len(chunks)}")
    logger.info("Sample 3 chunks (first 100 chars):")
    for i, chunk in enumerate(chunks[:3]):
        sample = chunk[:100] + "..." if len(chunk) > 100 else chunk
        logger.info(f"  {i+1}. {sample}")
    logger.info(f"Embeddings stored in Qdrant collection: {collection_name}")
    logger.info("="*50)

if __name__ == "__main__":
    process_crawled_content()