from fastapi import FastAPI, HTTPException, Query, Request
from pydantic import BaseModel
from typing import List, Optional
import os
import asyncio
from contextlib import asynccontextmanager
import uuid
from datetime import datetime
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import our RAG and database components
from rag_core import RAGSystem
from database import db_manager, ChatMessage

# Create lifespan context to initialize the RAG system and database
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Initialize the database
    await db_manager.init_pool()

    # Initialize the RAG system
    app.state.rag_system = RAGSystem(
        qdrant_url="qdrant_url",
        qdrant_api_key="qdrant_api_key",
        cohere_api_key=os.getenv("COHERE_API_KEY")
    )
    # Initialize the vector store with book content
    app.state.rag_system.initialize_vector_store()
    yield
    # Cleanup
    await db_manager.close()

app = FastAPI(
    title="Humanoid Robotics Book RAG API",
    description="RAG system for the Humanoid Robotics Book",
    version="1.0.0",
    lifespan=lifespan
)

# Request/Response models
class ChatRequest(BaseModel):
    query: str
    session_id: Optional[str] = None  # If not provided, a new session will be created
    selected_text: Optional[str] = None
    max_tokens: Optional[int] = 500
    temperature: Optional[float] = 0.7

class ChatResponse(BaseModel):
    response: str
    sources: List[dict]
    query: str
    session_id: str

class DocumentChunk(BaseModel):
    content: str
    title: str
    file_path: str
    section: str
    score: float

class SearchResponse(BaseModel):
    results: List[DocumentChunk]

@app.get("/")
async def root():
    return {"message": "Humanoid Robotics Book RAG API"}

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Main chat endpoint that processes user queries using RAG
    """
    try:
        if not app.state.rag_system.cohere_client:
            raise HTTPException(status_code=500, detail="Cohere API key not configured")

        # Create or use existing session
        session_id = request.session_id or str(uuid.uuid4())

        # Process the query using the RAG system
        response_text, sources = app.state.rag_system.answer_query(
            query=request.query,
            selected_text=request.selected_text,
            max_tokens=request.max_tokens,
            temperature=request.temperature
        )

        # Create response object
        response_obj = ChatResponse(
            response=response_text,
            sources=sources,
            query=request.query,
            session_id=session_id
        )

        # Save the user query and AI response to the database
        user_message = ChatMessage(
            message_id=str(uuid.uuid4()),
            session_id=session_id,
            role="user",
            content=request.query,
            timestamp=datetime.now().isoformat(),
            sources=None
        )

        ai_message = ChatMessage(
            message_id=str(uuid.uuid4()),
            session_id=session_id,
            role="assistant",
            content=response_text,
            timestamp=datetime.now().isoformat(),
            sources=sources
        )

        # Save messages to database (if database is available)
        if db_manager.pool:
            await db_manager.create_session(session_id)
            await db_manager.save_message(user_message)
            await db_manager.save_message(ai_message)

        return response_obj
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/search", response_model=SearchResponse)
async def search_endpoint(
    query: str = Query(..., min_length=1, max_length=500),
    limit: int = Query(5, ge=1, le=20)
):
    """
    Search endpoint to find relevant document chunks
    """
    try:
        results = app.state.rag_system.search_documents(query, limit=limit)
        return SearchResponse(results=results)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/sessions/{session_id}/messages")
async def get_session_messages(session_id: str):
    """
    Get all messages for a specific session
    """
    try:
        messages = await db_manager.get_session_messages(session_id)
        return {"messages": [msg.dict() for msg in messages]}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "service": "Humanoid Robotics Book RAG API"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)