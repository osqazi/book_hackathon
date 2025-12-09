import os
import asyncio
from typing import List, Tuple, Optional, Dict, Any
import hashlib
from pathlib import Path
import re

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct, Filter, FieldCondition, MatchValue
import cohere
import tiktoken


class DocumentChunk:
    def __init__(self, content: str, title: str, file_path: str, section: str, score: float = 0.0):
        self.content = content
        self.title = title
        self.file_path = file_path
        self.section = section
        self.score = score


class RAGSystem:
    def __init__(self, qdrant_url: str, qdrant_api_key: str, cohere_api_key: Optional[str] = None):
        """
        Initialize the RAG system with Qdrant and Cohere clients.

        Args:
            qdrant_url: URL of the Qdrant cluster
            qdrant_api_key: API key for Qdrant
            cohere_api_key: API key for Cohere (can be None initially)
        """
        self.qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            timeout=60
        )

        self.cohere_api_key = cohere_api_key
        self.cohere_client = cohere.Client(api_key=cohere_api_key) if cohere_api_key else None

        # Collection name for the book content
        self.collection_name = "humanoid_robotics_book"

        # Initialize tokenizer for content chunking
        self.tokenizer = tiktoken.encoding_for_model("gpt-3.5-turbo")

    def initialize_vector_store(self):
        """
        Initialize the Qdrant collection and populate it with book content if it doesn't exist.
        """
        # Check if collection exists
        try:
            collections = self.qdrant_client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)
        except:
            collection_exists = False

        if not collection_exists:
            print(f"Creating collection: {self.collection_name}")
            # Create collection with appropriate vector size (1024 for Cohere embeddings)
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE)
            )

            # Add the book content to the collection
            self._add_book_content_to_qdrant()
        else:
            print(f"Collection {self.collection_name} already exists, skipping initialization")

    def extract_content_from_markdown(self, file_path: str) -> List[Dict[str, Any]]:
        """
        Extract content from a markdown file, splitting into meaningful chunks.

        Args:
            file_path: Path to the markdown file

        Returns:
            List of content chunks with metadata
        """
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Remove frontmatter (content between ---)
        content = re.sub(r'---.*?---', '', content, flags=re.DOTALL)

        # Split content into chunks based on headings
        chunks = []

        # Split by H1, H2, H3 headings
        sections = re.split(r'\n(?=#)', content)

        current_section_title = "Introduction"
        current_section_content = ""

        for section in sections:
            if section.strip():
                # Check if this section starts with a heading
                heading_match = re.match(r'#* (.+)', section.strip())

                if heading_match:
                    # Save previous section if it exists
                    if current_section_content.strip():
                        chunks.append({
                            'title': current_section_title.strip(),
                            'content': current_section_content.strip(),
                            'file_path': file_path,
                            'section': current_section_title.strip()
                        })

                    # Start new section
                    current_section_title = heading_match.group(1)
                    current_section_content = section.strip()
                else:
                    current_section_content += section

        # Add the last section
        if current_section_content.strip():
            chunks.append({
                'title': current_section_title.strip(),
                'content': current_section_content.strip(),
                'file_path': file_path,
                'section': current_section_title.strip()
            })

        # Further split large chunks to ensure they're not too large for embeddings
        final_chunks = []
        for chunk in chunks:
            content = chunk['content']
            # Use token count to determine if chunk is too large
            token_count = len(self.tokenizer.encode(content))

            if token_count > 1000:  # If chunk has more than ~1000 tokens
                # Split by paragraphs
                paragraphs = content.split('\n\n')
                current_chunk = ""
                current_tokens = 0

                for para in paragraphs:
                    para_tokens = len(self.tokenizer.encode(para))
                    if current_tokens + para_tokens < 1000:
                        current_chunk += "\n\n" + para
                        current_tokens += para_tokens
                    else:
                        if current_chunk.strip():
                            final_chunks.append({
                                'title': chunk['title'],
                                'content': current_chunk.strip(),
                                'file_path': chunk['file_path'],
                                'section': chunk['section']
                            })
                        current_chunk = para
                        current_tokens = para_tokens

                # Add remaining content
                if current_chunk.strip():
                    final_chunks.append({
                        'title': chunk['title'],
                        'content': current_chunk.strip(),
                        'file_path': chunk['file_path'],
                        'section': chunk['section']
                    })
            else:
                final_chunks.append(chunk)

        return final_chunks

    def _add_book_content_to_qdrant(self):
        """
        Process all markdown files in the docs directory and add to Qdrant collection.
        """
        print("Processing book content and adding to Qdrant...")

        # Find all markdown files in the docs directory
        docs_dir = Path("docs")
        if not docs_dir.exists():
            # Try relative to current working directory
            docs_dir = Path("../docs")

        if docs_dir.exists():
            md_files = list(docs_dir.rglob("*.md"))
        else:
            # If docs directory doesn't exist relative to backend, check project root
            md_files = list(Path("../..").rglob("docs/*.md"))

        print(f"Found {len(md_files)} markdown files to process")

        points = []
        processed_count = 0

        for file_path in md_files:
            print(f"Processing: {file_path}")

            try:
                # Extract content chunks from the file
                chunks = self.extract_content_from_markdown(str(file_path))

                for chunk in chunks:
                    # Get embedding for the content (using a mock if Cohere not available during indexing)
                    if self.cohere_client:
                        try:
                            response = self.cohere_client.embed(
                                texts=[chunk['content']],
                                model="embed-english-v3.0",
                                input_type="search_document"
                            )
                            embedding = response.embeddings[0]  # Cohere returns embeddings as a list
                        except Exception as e:
                            print(f"Error getting embedding for chunk in {file_path}: {str(e)}")
                            continue
                    else:
                        # Use a mock embedding during initialization if Cohere key not available
                        # Cohere's embed-english-v3.0 returns 1024-dimensional embeddings
                        embedding = [0.0] * 1024

                    # Create a unique ID for this chunk
                    content_hash = hashlib.md5(f"{chunk['file_path']}_{chunk['title']}_{chunk['content'][:100]}".encode()).hexdigest()
                    point_id = int(content_hash[:8], 16)  # Convert first 8 hex chars to int

                    # Create point for Qdrant
                    point = PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload={
                            'title': chunk['title'],
                            'content': chunk['content'],
                            'file_path': str(chunk['file_path']),
                            'section': chunk['section'],
                            'source_file': os.path.basename(chunk['file_path'])
                        }
                    )

                    points.append(point)
                    processed_count += 1

                    # Batch insert every 50 points
                    if len(points) >= 50:
                        self.qdrant_client.upsert(
                            collection_name=self.collection_name,
                            points=points
                        )
                        print(f"Upserted {len(points)} points to collection")
                        points = []

            except Exception as e:
                print(f"Error processing {file_path}: {str(e)}")
                continue

        # Insert any remaining points
        if points:
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            print(f"Upserted final {len(points)} points to collection")

        print(f"Successfully processed and added {processed_count} content chunks to Qdrant collection: {self.collection_name}")

    def search_documents(self, query: str, limit: int = 5) -> List[DocumentChunk]:
        """
        Search the vectorized book content for relevant documents.

        Args:
            query: Search query
            limit: Number of results to return

        Returns:
            List of matching document chunks
        """
        if not self.cohere_client:
            # If Cohere is not available, return some sample results or empty results
            # In a real implementation, you might use a local embedding model
            print("Cohere client not initialized. Returning empty results.")
            return []

        try:
            # Get embedding for the query using Cohere
            response = self.cohere_client.embed(
                texts=[query],
                model="embed-english-v3.0",
                input_type="search_query"  # Using search_query for queries vs search_document for documents
            )
            query_embedding = response.embeddings[0]  # Cohere returns embeddings as a list

            # Search in Qdrant - using the correct method name for the newer API
            from qdrant_client.http.models import SearchRequest
            search_result = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit
            )
            hits = search_result

            results = []
            for hit in hits:
                payload = hit.payload
                results.append(DocumentChunk(
                    content=payload.get('content', ''),
                    title=payload.get('title', ''),
                    file_path=payload.get('file_path', ''),
                    section=payload.get('section', ''),
                    score=hit.score
                ))

            return results
        except Exception as e:
            print(f"Error during search: {e}")
            return []

    def answer_query(self, query: str, selected_text: Optional[str] = None,
                          max_tokens: int = 500, temperature: float = 0.7) -> Tuple[str, List[dict]]:
        """
        Answer a query using RAG approach.

        Args:
            query: User query
            selected_text: Optional text selected by user to constrain search
            max_tokens: Maximum tokens for the response
            temperature: Temperature for the LLM

        Returns:
            Tuple of (response text, list of source documents)
        """
        if not self.cohere_client:
            # If Cohere is not available, return a helpful message
            return "I'm sorry, but I need a Cohere API key to generate responses. Please configure the API key to use this feature.", []

        try:
            # Search for relevant documents
            relevant_docs = self.search_documents(query, limit=5)

            # If user selected specific text, prioritize results from that context
            if selected_text:
                # This is a simplified approach - in a real implementation you'd want to
                # match the selected text to document sections
                filtered_docs = []
                for doc in relevant_docs:
                    if selected_text.lower() in doc.content.lower():
                        filtered_docs.append(doc)
                # If we found matching docs with selected text, use those; otherwise use all relevant
                if filtered_docs:
                    relevant_docs = filtered_docs

            # Prepare context from retrieved documents
            context_parts = []
            sources = []

            for doc in relevant_docs:
                context_parts.append(f"Section: {doc.title}\nContent: {doc.content}\n---")
                sources.append({
                    'title': doc.title,
                    'file_path': doc.file_path,
                    'section': doc.section,
                    'score': doc.score
                })

            context = "\n".join(context_parts)

            # Prepare the prompt for the LLM
            if selected_text:
                prompt = f"""
                You are an expert assistant for the Humanoid Robotics Book. Answer the user's question based on the provided context.
                The user has selected specific text, so focus your answer on that content.

                Selected Text: {selected_text}

                Context from the book:
                {context}

                Question: {query}

                Please provide an accurate answer based on the context. If the context doesn't contain the information needed to answer the question, say so.
                """
            else:
                prompt = f"""
                You are an expert assistant for the Humanoid Robotics Book. Answer the user's question based on the provided context.

                Context from the book:
                {context}

                Question: {query}

                Please provide an accurate answer based on the context. If the context doesn't contain the information needed to answer the question, say so.
                """

            # Call the Cohere API to generate the response
            response = self.cohere_client.chat(
                message=prompt,
                model="command-r-08-2024",  # Using Cohere's command-r model from 08-2024
                temperature=temperature,
                max_tokens=max_tokens
            )

            return response.text, sources
        except Exception as e:
            print(f"Error during answer generation: {e}")
            return f"I encountered an error while processing your request: {str(e)}", []