# Physical AI & Humanoid Robotics Textbook - RAG Chatbot

This is a RAG (Retrieval Augmented Generation) chatbot system that answers questions based on the Physical AI & Humanoid Robotics textbook content. The system uses vector embeddings to provide contextually relevant responses to user queries.

## Tech Stack
- **Backend Framework**: FastAPI
- **Embeddings**: Gemini
- **Vector Database**: Qdrant
- **LLM**: Groq (llama-3.1-8b-instant)
- **RAG System**: Custom implementation for textbook content

## Features
- Semantic search through textbook content
- Context-aware question answering
- Vector storage and retrieval
- API endpoints for chat interactions

## Installation and Setup

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Set up environment variables in `.env` file:
```
GEMINI_API_KEY=your_gemini_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=new_book_vectors
GROQ_API_KEY=your_groq_api_key
```

3. Run the application:
```bash
cd backend
uvicorn main:app --reload
```

## API Endpoints
- `GET /` - Root endpoint to check API status
- `GET /health` - Health check endpoint
- `POST /api/v1/chat` - Chat endpoint for question answering

## Environment Variables
- `GEMINI_API_KEY`: Your Gemini API key for embeddings
- `QDRANT_URL`: Your Qdrant Cloud URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `QDRANT_COLLECTION_NAME`: Name for the vector collection (default: new_book_vectors)
- `GROQ_API_KEY`: Your Groq API key for LLM responses

## Hugging Face Deployment
This application is configured for deployment on Hugging Face Spaces with a Python + FastAPI runtime. The main application entry point is `backend/main.py`.

## Embedding Pipeline
The project includes an automated pipeline for creating vector embeddings of the book content:
- Crawls content from the textbook
- Cleans and chunks the text content
- Generates semantic embeddings using Gemini
- Stores vectors in Qdrant Cloud with rich metadata