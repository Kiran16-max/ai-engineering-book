"""
Main FastAPI application for the RAG chatbot backend.
This is the entry point for the application.
"""
import sys
import os

# Add the backend directory to the Python path to allow imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from routes.chat import router as chat_router

# Initialize FastAPI app
app = FastAPI(
    title="Book RAG Chatbot API",
    description="A RAG system that answers questions based on book content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Allow the frontend origin
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat_router, prefix="/api/v1")

@app.get("/")
def root():
    return {"status": "API running"}

@app.get("/health")
async def health_check():
    """
    Health check endpoint.
    """
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)