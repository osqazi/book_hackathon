import os
import asyncpg
import asyncio
from typing import Optional, List, Dict, Any
from contextlib import asynccontextmanager
from pydantic import BaseModel


class ChatSession(BaseModel):
    session_id: str
    user_id: Optional[str] = None
    created_at: str
    last_accessed: str


class ChatMessage(BaseModel):
    message_id: str
    session_id: str
    role: str  # 'user' or 'assistant'
    content: str
    timestamp: str
    sources: Optional[List[Dict[str, Any]]] = None


class DatabaseManager:
    def __init__(self):
        self.pool = None
        self.database_url = os.getenv("NEON_DATABASE_URL")

    async def init_pool(self):
        """Initialize the connection pool"""
        if not self.database_url:
            print("Warning: NEON_DATABASE_URL not set, database features will be disabled")
            return

        try:
            self.pool = await asyncpg.create_pool(
                dsn=self.database_url,
                min_size=1,
                max_size=10,
                command_timeout=60
            )
            await self._create_tables()
            print("Database connection pool initialized successfully")
        except Exception as e:
            print(f"Error initializing database pool: {e}")
            self.pool = None

    async def _create_tables(self):
        """Create required tables if they don't exist"""
        if not self.pool:
            return

        async with self.pool.acquire() as conn:
            # Create chat_sessions table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS chat_sessions (
                    session_id TEXT PRIMARY KEY,
                    user_id TEXT,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    last_accessed TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)

            # Create chat_messages table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS chat_messages (
                    message_id TEXT PRIMARY KEY,
                    session_id TEXT REFERENCES chat_sessions(session_id),
                    role TEXT NOT NULL,
                    content TEXT NOT NULL,
                    sources JSONB,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)

            # Create indexes
            await conn.execute("""
                CREATE INDEX IF NOT EXISTS idx_chat_messages_session_id
                ON chat_messages(session_id)
            """)
            await conn.execute("""
                CREATE INDEX IF NOT EXISTS idx_chat_sessions_user_id
                ON chat_sessions(user_id)
            """)

    async def create_session(self, session_id: str, user_id: Optional[str] = None) -> bool:
        """Create a new chat session"""
        if not self.pool:
            return False

        try:
            async with self.pool.acquire() as conn:
                await conn.execute("""
                    INSERT INTO chat_sessions (session_id, user_id)
                    VALUES ($1, $2)
                    ON CONFLICT (session_id) DO UPDATE
                    SET last_accessed = CURRENT_TIMESTAMP
                """, session_id, user_id)
            return True
        except Exception as e:
            print(f"Error creating session: {e}")
            return False

    async def save_message(self, message: ChatMessage) -> bool:
        """Save a chat message to the database"""
        if not self.pool:
            return False

        try:
            import json
            from datetime import datetime
            # Convert timestamp string to datetime object if it's a string
            timestamp_obj = message.timestamp
            if isinstance(message.timestamp, str):
                timestamp_obj = datetime.fromisoformat(message.timestamp.replace('Z', '+00:00'))

            async with self.pool.acquire() as conn:
                # Serialize sources to JSON string if it's a list/dict, otherwise use as is
                sources_json = json.dumps(message.sources) if message.sources is not None else None

                await conn.execute("""
                    INSERT INTO chat_messages (message_id, session_id, role, content, sources, timestamp)
                    VALUES ($1, $2, $3, $4, $5, $6)
                """, message.message_id, message.session_id, message.role,
                message.content, sources_json, timestamp_obj)
            return True
        except Exception as e:
            print(f"Error saving message: {e}")
            return False

    async def get_session_messages(self, session_id: str) -> List[ChatMessage]:
        """Retrieve all messages for a session"""
        if not self.pool:
            return []

        try:
            async with self.pool.acquire() as conn:
                rows = await conn.fetch("""
                    SELECT message_id, session_id, role, content, sources, timestamp
                    FROM chat_messages
                    WHERE session_id = $1
                    ORDER BY timestamp ASC
                """, session_id)

                messages = []
                for row in rows:
                    messages.append(ChatMessage(
                        message_id=row['message_id'],
                        session_id=row['session_id'],
                        role=row['role'],
                        content=row['content'],
                        sources=row['sources'],
                        timestamp=str(row['timestamp'])
                    ))
                return messages
        except Exception as e:
            print(f"Error retrieving messages: {e}")
            return []

    async def close(self):
        """Close the connection pool"""
        if self.pool:
            await self.pool.close()


# Global database manager instance
db_manager = DatabaseManager()


@asynccontextmanager
async def lifespan(app):
    """Lifespan context manager for the database"""
    await db_manager.init_pool()
    yield
    await db_manager.close()