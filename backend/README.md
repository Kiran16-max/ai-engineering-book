# Book RAG Chatbot Backend

This is the backend for the book-based RAG chatbot system.

## Stack
- Embeddings: Gemini
- Vector DB: Qdrant
- LLM (answers): Groq (llama-3.1-8b-instant)
- Framework: FastAPI

## Installation

1. Install dependencies:
```bash
pip install fastapi uvicorn qdrant-client google-generativeai groq python-dotenv
```

2. Set up environment variables in `.env` file

3. Run the application:
```bash
cd backend
uvicorn main:app --reload
```

## Configuration

Make sure to set the following environment variables in your `.env` file:
- `GEMINI_API_KEY`: Your Gemini API key
- `QDRANT_URL`: Your Qdrant Cloud URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `QDRANT_COLLECTION_NAME`: Name for the new collection (default: new_book_vectors)
- `GROQ_API_KEY`: Your Groq API key