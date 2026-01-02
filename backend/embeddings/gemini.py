"""
Gemini embedding module for generating text embeddings.
"""
import google.generativeai as genai
from config.config import Config
import logging
import time

logger = logging.getLogger(__name__)

class GeminiEmbedder:
    """
    Class to handle text embedding generation using Gemini API.
    """
    def __init__(self):
        """
        Initialize Gemini client with API key from environment variables.
        """
        if not Config.GEMINI_API_KEY:
            raise ValueError("Gemini API key must be set in environment variables")

        genai.configure(api_key=Config.GEMINI_API_KEY)
        # Use the embedding model
        self.model = "text-embedding-004"  # Google's latest embedding model

    def generate_embeddings(self, texts):
        """
        Generate embeddings for a list of texts.

        Args:
            texts: List of strings to embed

        Returns:
            List of embedding vectors
        """
        embeddings = []
        for i, text in enumerate(texts):
            # Skip empty texts
            if not text.strip():
                continue

            max_retries = 3
            retry_count = 0

            while retry_count < max_retries:
                try:
                    # Generate embedding for the text
                    response = genai.embed_content(
                        model=self.model,
                        content=text,
                        task_type="retrieval_document",  # Suitable for document retrieval
                        title="Book Content"  # Optional title for the content
                    )
                    embeddings.append(response['embedding'])
                    logger.info(f"Successfully generated embedding for text {i+1}/{len(texts)}")
                    break  # Success, exit retry loop
                except Exception as e:
                    retry_count += 1
                    logger.error(f"Error generating embedding for text {i+1}/{len(texts)}: {str(e)}")
                    if retry_count < max_retries:
                        logger.info(f"Retrying ({retry_count}/{max_retries})...")
                        time.sleep(2 ** retry_count)  # Exponential backoff
                    else:
                        logger.error(f"Failed to generate embedding after {max_retries} retries for text: {text[:50]}...")
                        # Instead of raising an exception, append a placeholder or skip
                        # For now, we'll skip this text and continue
                        continue

        return embeddings