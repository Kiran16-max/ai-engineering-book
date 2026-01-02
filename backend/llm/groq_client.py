"""
Groq LLM client for answer generation in the RAG system.
"""
import logging
from typing import List, Dict, Any
import os
import groq

logger = logging.getLogger(__name__)

class GroqClient:
    """
    Client for interacting with Groq API for answer generation.
    """
    def __init__(self, api_key: str):
        """
        Initialize the Groq client.

        Args:
            api_key: The Groq API key
        """
        self.client = groq.Groq(api_key=api_key)
        self.model = "llama-3.1-8b-instant"  # Using the model mentioned in README
        logger.info("Groq client initialized successfully")

    def generate_answer(self, query: str, context: str) -> str:
        """
        Generate an answer based on the query and context using Groq.

        Args:
            query: The user's question
            context: The retrieved context from the vector database

        Returns:
            Generated answer
        """
        try:
            logger.info(f"Generating answer with Groq for query: {query[:50]}...")

            # Create the prompt for the LLM with multilingual support
            prompt = f"""
            You are an AI assistant that answers questions based on book content.
            Use ONLY the provided context to answer the question.
            If the exact wording of the question is not in the context, but related concepts are explained using different wording, summarize those concepts clearly.
            If the answer truly does not exist in the context, then politely say "Not covered" or "Sorry, I encountered an error".
            Answer in the same language as the question (English, Urdu, or Roman Urdu).
            Be concise, accurate, and reference book modules/chapters when possible.

            Context:
            {context}

            Question: {query}

            Answer:
            """

            # Create chat completion
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": "You are a multilingual assistant that answers questions based on provided context. Only use information from the context to answer questions. Do not hallucinate or make up information. Answer in the same language as the question (English, Urdu, or Roman Urdu). If the exact wording of the question is not in the context, but related concepts are explained using different wording, summarize those concepts clearly. If the answer truly does not exist in the context, then politely say 'Not covered' or 'Sorry, I encountered an error'."
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                temperature=0.3,
                max_tokens=500
            )

            answer = response.choices[0].message.content

            if not answer:
                answer = "Not covered"

            logger.info("Answer generated successfully with Groq")
            return answer

        except Exception as e:
            logger.error(f"Error generating answer with Groq: {e}")
            return "Sorry, I encountered an error"