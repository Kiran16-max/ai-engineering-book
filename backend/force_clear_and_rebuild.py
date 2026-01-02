"""
Script to force clear and rebuild the Qdrant collection with book content.
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
from qdrant_client.http import models

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def force_clear_and_rebuild():
    """
    Force clear the collection and rebuild with fresh book content.
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

    # Read the book content
    logger.info("Reading book content...")
    book_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "book.txt")
    try:
        with open(book_path, 'r', encoding='utf-8') as file:
            book_content = file.read()
        logger.info(f"Book content read successfully. Total characters: {len(book_content)}")
    except FileNotFoundError:
        logger.error(f"Book file not found at path: {book_path}")
        return
    except Exception as e:
        logger.error(f"Error reading book file: {e}")
        return

    # Clear existing collection
    collection_name = Config.QDRANT_COLLECTION_NAME
    logger.info(f"Clearing existing collection: {collection_name}")
    try:
        # Delete all points in the collection
        qdrant_conn.get_client().delete(
            collection_name=collection_name,
            points_selector=models.FilterSelector(
                filter=models.Filter(
                    must=[]
                )
            )
        )
        logger.info("Collection cleared successfully")
    except Exception as e:
        logger.warning(f"Could not clear collection (might not exist yet): {e}")
        # If collection doesn't exist, create it
        try:
            qdrant_conn.create_collection(vector_size=768, distance=Distance.COSINE)
            logger.info(f"Created new collection: {collection_name}")
        except Exception as create_error:
            logger.error(f"Error creating collection: {create_error}")
            return

    # Chunk the book content
    logger.info("Chunking book content...")
    chunks = chunk_text(
        text=book_content,
        chunk_size=1000,
        overlap=200,
        min_chunk_size=100
    )
    logger.info(f"Book chunked into {len(chunks)} chunks")

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
            payload={"text": chunk, "chunk_id": i, "source": "book.txt"}
        )
        points.append(point)

    logger.info(f"Validated {len(points)} embeddings with correct vector size ({expected_vector_size})")

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
    logger.info("FORCE CLEAR AND REBUILD COMPLETE")
    logger.info(f"Total chunks: {len(chunks)}")
    logger.info("Sample 3 chunks (first 100 chars):")
    for i, chunk in enumerate(chunks[:3]):
        sample = chunk[:100] + "..." if len(chunk) > 100 else chunk
        logger.info(f"  {i+1}. {sample}")
    logger.info(f"Embeddings stored in Qdrant collection: {collection_name}")
    logger.info("="*50)

if __name__ == "__main__":
    force_clear_and_rebuild()