"""
RAG (Retrieval-Augmented Generation) system for the book-based chatbot.
"""
import logging
import re
from typing import List, Tuple, Dict, Any
import os
import sys

# Add the backend directory to the path so we can import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.config import Config
from embeddings.gemini import GeminiEmbedder
from vector_db.qdrant import QdrantConnection
from llm.groq_client import GroqClient
from qdrant_client.http import models

logger = logging.getLogger(__name__)

class RAGSystem:
    """
    RAG system that retrieves relevant chunks from the book and generates answers.
    """
    def __init__(self):
        """
        Initialize the RAG system with embedder and vector database.
        """
        # Initialize components
        self.embedder = GeminiEmbedder()
        self.qdrant_conn = QdrantConnection()
        self.collection_name = Config.QDRANT_COLLECTION_NAME

        # Initialize the LLM for answer generation using Groq
        if not Config.GROQ_API_KEY:
            raise ValueError("GROQ_API_KEY must be set in environment variables")

        self.llm_client = GroqClient(Config.GROQ_API_KEY)

        logger.info("RAG System initialized successfully")

    def normalize_query(self, query: str) -> str:
        """
        Normalize vague or incomplete queries to be more specific.
        """
        # Convert to lowercase for comparison
        query_lower = query.lower().strip()

        # Define vague terms and their replacements
        vague_terms = [
            (r'\bthis\b', 'the topic'),
            (r'\bthat\b', 'the topic'),
            (r'\bit\b', 'the concept'),
            (r'\bthese\b', 'these topics'),
            (r'\bthose\b', 'those topics'),
        ]

        # Replace vague terms with more specific ones
        normalized_query = query
        for pattern, replacement in vague_terms:
            normalized_query = re.sub(pattern, replacement, normalized_query, flags=re.IGNORECASE)

        # Handle common vague phrases
        if 'importance of this' in query_lower:
            normalized_query = normalized_query.replace('importance of this', 'importance of Artificial Intelligence', 1)
        elif 'importance of that' in query_lower:
            normalized_query = normalized_query.replace('importance of that', 'importance of Artificial Intelligence', 1)
        elif 'uses and abuses of this' in query_lower:
            normalized_query = normalized_query.replace('uses and abuses of this', 'uses and abuses of Artificial Intelligence', 1)
        elif 'uses and abuses of that' in query_lower:
            normalized_query = normalized_query.replace('uses and abuses of that', 'uses and abuses of Artificial Intelligence', 1)
        elif query_lower.startswith('tell me ') or query_lower.startswith('tell me about '):
            # Expand "tell me" queries to be more specific
            topic = query[8:].strip() if query_lower.startswith('tell me about ') else query[7:].strip()
            if topic:
                # If the topic is too general, make it more specific to the book content
                if topic in ['ai', 'artificial intelligence', 'robotics', 'humanoid', 'this', 'that', 'it']:
                    normalized_query = f"Explain {topic} according to the book on Physical AI & Humanoid Robotics"
                else:
                    normalized_query = f"Explain {topic} according to the book"
            else:
                normalized_query = "Explain Artificial Intelligence according to the book on Physical AI & Humanoid Robotics"

        # Handle very short queries
        if len(normalized_query.strip()) < 5:
            if query_lower in ['ai', 'this', 'that', 'it']:
                normalized_query = f"Explain {query} according to the book on Physical AI & Humanoid Robotics"
            elif query_lower in ['yes', 'no', 'ok']:
                normalized_query = "Explain Artificial Intelligence according to the book on Physical AI & Humanoid Robotics"

        # Handle queries that are just keywords
        if '?' not in normalized_query and '.' not in normalized_query and len(normalized_query.split()) <= 3:
            normalized_query = f"What is {normalized_query} according to the book on Physical AI & Humanoid Robotics?"

        # If the query is still too vague, add book context
        if len(normalized_query.strip()) < 3:
            normalized_query = "Explain Artificial Intelligence according to the book on Physical AI & Humanoid Robotics"

        # Ensure the query is properly formatted
        if not normalized_query.endswith('?') and not normalized_query.endswith('.'):
            if any(word in normalized_query.lower() for word in ['what', 'how', 'why', 'when', 'where', 'who', 'explain', 'describe', 'define']):
                normalized_query += '?'
            else:
                normalized_query += '.'

        logger.info(f"Normalized query: '{query}' -> '{normalized_query}'")
        return normalized_query

    def retrieve_relevant_chunks(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant chunks from the vector database based on the query.

        Args:
            query: The user's question
            top_k: Number of top results to return

        Returns:
            List of relevant chunks with their metadata
        """
        try:
            logger.info(f"Retrieving relevant chunks for query: {query[:50]}...")

            # Generate embedding for the query
            query_embedding = self.embedder.generate_embeddings([query])[0]

            # Search in Qdrant using the collection search method (compatible with Qdrant 1.16.2)
            # Using the search method that was confirmed to exist in the client
            search_results = self.qdrant_conn.get_client().search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True
            )

            # Extract relevant chunks
            relevant_chunks = []
            for result in search_results:
                chunk_data = {
                    'text': result.payload.get('text', ''),
                    'source': result.payload.get('source', ''),
                    'chunk_id': result.payload.get('chunk_id', ''),
                    'score': result.score
                }
                relevant_chunks.append(chunk_data)

            # Limit to top 3-5 chunks as requested
            relevant_chunks = relevant_chunks[:top_k]
            logger.info(f"Retrieved {len(relevant_chunks)} relevant chunks")
            return relevant_chunks

        except AttributeError as e:
            # Handle case where search method doesn't exist
            logger.error(f"Search method not available on Qdrant client: {e}")

            # Try using the query_points method which is also available in Qdrant 1.16.2
            try:
                search_results = self.qdrant_conn.get_client().query_points(
                    collection_name=self.collection_name,
                    query=query_embedding,
                    limit=top_k,
                    with_payload=True
                )

                # Extract relevant chunks
                relevant_chunks = []
                for result in search_results.points:  # Note: query_points returns a different structure
                    chunk_data = {
                        'text': result.payload.get('text', '') if result.payload else '',
                        'source': result.payload.get('source', '') if result.payload else '',
                        'chunk_id': result.payload.get('chunk_id', '') if result.payload else '',
                        'score': result.score if hasattr(result, 'score') else 0
                    }
                    relevant_chunks.append(chunk_data)

                # Limit to top 3-5 chunks as requested
                relevant_chunks = relevant_chunks[:top_k]
                logger.info(f"Retrieved {len(relevant_chunks)} relevant chunks using query_points method")
                return relevant_chunks
            except Exception as query_error:
                logger.error(f"Query points method also failed: {query_error}")

                # If all search methods fail, return empty list
                logger.info("All search methods failed, returning empty results")
                return []
        except Exception as e:
            logger.error(f"Error retrieving relevant chunks: {e}")
            return []  # Return empty list if search fails

    def generate_answer(self, query: str, context_chunks: List[Dict[str, Any]]) -> str:
        """
        Generate an answer using the LLM based on the query and context chunks.

        Args:
            query: The user's question
            context_chunks: List of relevant context chunks

        Returns:
            Generated answer based on the context
        """
        try:
            logger.info(f"Generating answer for query: {query[:50]}...")

            # Combine context chunks into a single context string
            context_text = "\n\n".join([chunk['text'] for chunk in context_chunks])

            # Generate response using the Groq client
            answer = self.llm_client.generate_answer(query, context_text)

            logger.info("Answer generated successfully")
            return answer

        except Exception as e:
            logger.error(f"Error generating answer: {e}")
            return "Sorry, I encountered an error while generating the answer."

    def answer_question(self, query: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Complete RAG process: retrieve relevant chunks and generate an answer.

        Args:
            query: The user's question
            top_k: Number of top results to retrieve

        Returns:
            Dictionary containing the answer and relevant information
        """
        try:
            logger.info(f"Processing question: {query}")

            # Normalize the query to handle vague/incomplete queries
            normalized_query = self.normalize_query(query)
            logger.info(f"Using normalized query: {normalized_query}")

            # Retrieve relevant chunks
            context_chunks = self.retrieve_relevant_chunks(normalized_query, top_k)

            # Check if we have relevant context from the book
            if not context_chunks or len(context_chunks) == 0:
                logger.info("No relevant context found in the book")
                answer = "Not covered"
            else:
                # Generate answer based on context
                answer = self.generate_answer(normalized_query, context_chunks)

            # Prepare response
            response = {
                "query": query,  # Return the original query to the user
                "answer": answer,
                "sources": [chunk for chunk in context_chunks],
                "num_sources": len(context_chunks)
            }

            logger.info(f"Question processed successfully. Answer length: {len(answer)}")
            return response

        except Exception as e:
            logger.error(f"Error processing question: {e}")
            return {
                "query": query,
                "answer": "Sorry, I encountered an error while processing your question.",
                "sources": [],
                "num_sources": 0
            }