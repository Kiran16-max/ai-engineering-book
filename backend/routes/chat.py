"""
Chat route for the RAG system.
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import logging
import sys
import os

# Add the backend directory to the path so we can import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.config import Config
from rag_system import RAGSystem

# Initialize router
router = APIRouter()

# Request/Response models
class ChatRequest(BaseModel):
    query: str
    top_k: Optional[int] = 5

class ChatResponse(BaseModel):
    query: str
    answer: str
    sources: List[dict]

# Initialize logging
logger = logging.getLogger(__name__)

# Initialize the RAG system
rag_system = None

def get_rag_system():
    global rag_system
    if rag_system is None:
        try:
            rag_system = RAGSystem()
            logger.info("RAG System initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize RAG System: {e}")
            raise HTTPException(status_code=500, detail="Failed to initialize RAG system")
    return rag_system

@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Chat endpoint that processes user queries using the RAG system.
    """
    try:
        query = request.query.strip()
        top_k = request.top_k

        if not query:
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        # Get the RAG system instance
        rag = get_rag_system()

        # Process the query using the RAG system
        result = rag.answer_question(query, top_k=top_k)

        # Create response
        response = ChatResponse(
            query=result["query"],
            answer=result["answer"],
            sources=result["sources"]
        )

        logger.info(f"Successfully processed query: {query[:50]}...")
        return response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")